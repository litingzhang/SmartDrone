#include "adapters/slam/xfeat_native_extractor.h"

#include "adapters/slam/xfeat_frontend_client.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <utility>

#include <opencv2/imgproc.hpp>

#if defined(SMART_DRONE_XFEAT_NATIVE_AVAILABLE)
#include <torch/torch.h>

#include "adapters/slam/xfeat_native_model.h"
#endif

namespace smartdrone::adapters::slam {

XFeatNativeExtractor::XFeatNativeExtractor() = default;

namespace {

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

bool XFeatNativeExtractor::PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err)
{
    if (gray.type() == CV_8UC1 && gray.isContinuous()) {
        gray8 = gray;
        return true;
    }
    if (gray.channels() == 1) {
        gray.convertTo(gray8, CV_8UC1);
    } else {
        cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
    }
    if (!gray8.isContinuous()) {
        gray8 = gray8.clone();
    }
    if (gray8.empty()) {
        if (err != nullptr) {
            *err = "xfeat native gray image preparation failed";
        }
        return false;
    }
    return true;
}

#if defined(SMART_DRONE_XFEAT_NATIVE_AVAILABLE)

namespace {

constexpr int kDescriptorDim = 64;

torch::Device ResolveTorchDevice(const std::string &deviceText, std::string *err)
{
    const std::string normalized = [&]() {
        std::string out = deviceText;
        std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return out;
    }();
    if (normalized.empty() || normalized == "auto") {
        return torch::cuda::is_available() ? torch::Device(torch::kCUDA) : torch::Device(torch::kCPU);
    }
    if (normalized == "cuda") {
        if (!torch::cuda::is_available()) {
            if (err != nullptr) {
                *err = "xfeat native requested cuda but torch::cuda::is_available() is false";
            }
            return torch::Device(torch::kCPU);
        }
        return torch::Device(torch::kCUDA);
    }
    if (normalized == "cpu") {
        return torch::Device(torch::kCPU);
    }
    if (err != nullptr) {
        *err = "xfeat native unsupported device: " + deviceText;
    }
    return torch::Device(torch::kCPU);
}

std::filesystem::path ResolveWeightsPath(const std::string &repoPath)
{
    std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        const std::filesystem::path candidate = repo / "weights" / "xfeat.pt";
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return std::filesystem::path();
}

} // namespace

struct XFeatNativeExtractor::Impl {
    torch::Device device{torch::kCPU};
    XFeatModel model;
    std::shared_ptr<XFeatSparseInterpolator> bilinear;
    std::shared_ptr<XFeatSparseInterpolator> nearest;

    bool Load(const std::string &repoPath, const std::string &deviceText, std::string *err)
    {
        device = ResolveTorchDevice(deviceText, err);
        if (deviceText == "cuda" && device.is_cpu()) {
            return false;
        }
        model = XFeatModel();
        const std::filesystem::path weightsPath = ResolveWeightsPath(repoPath);
        if (weightsPath.empty()) {
            if (err != nullptr) {
                *err = "xfeat native weights not found under repo: " + repoPath;
            }
            return false;
        }
        torch::serialize::InputArchive archive;
        archive.load_from(weightsPath.string());
        model->load(archive);
        model->to(device);
        model->eval();
        bilinear = std::make_shared<XFeatSparseInterpolator>("bilinear");
        nearest = std::make_shared<XFeatSparseInterpolator>("nearest");
        return true;
    }

    torch::Tensor BuildBatch(const std::vector<cv::Mat> &images) const
    {
        if (images.empty()) {
            return {};
        }
        const int rows = images.front().rows;
        const int cols = images.front().cols;
        torch::Tensor batch = torch::empty({static_cast<long long>(images.size()), 1, rows, cols}, torch::kFloat32);
        for (size_t i = 0; i < images.size(); ++i) {
            torch::Tensor frame = torch::from_blob(const_cast<uint8_t *>(images[i].data), {rows, cols}, torch::kUInt8);
            batch.index_put_({static_cast<long long>(i), 0}, frame.to(torch::kFloat32).div_(255.0f));
        }
        return batch.to(device);
    }

    std::tuple<torch::Tensor, double, double> PreprocessTensor(torch::Tensor x) const
    {
        const int height = static_cast<int>(x.size(-2));
        const int width = static_cast<int>(x.size(-1));
        const int resizedHeight = std::max(32, (height / 32) * 32);
        const int resizedWidth = std::max(32, (width / 32) * 32);
        const double ratioH = static_cast<double>(height) / static_cast<double>(resizedHeight);
        const double ratioW = static_cast<double>(width) / static_cast<double>(resizedWidth);
        x = torch::nn::functional::interpolate(
            x, torch::nn::functional::InterpolateFuncOptions()
                   .size(std::vector<int64_t>{resizedHeight, resizedWidth})
                   .mode(torch::kBilinear)
                   .align_corners(false));
        return std::make_tuple(x, ratioH, ratioW);
    }

    torch::Tensor GetKptsHeatmap(torch::Tensor kpts, float softmaxTemp = 1.0f) const
    {
        torch::Tensor scores =
            torch::nn::functional::softmax(kpts * softmaxTemp, torch::nn::functional::SoftmaxFuncOptions(1));
        scores = scores.index(
            {torch::indexing::Slice(), torch::indexing::Slice(0, 64), torch::indexing::Slice(), torch::indexing::Slice()});
        const int batch = static_cast<int>(scores.size(0));
        const int height = static_cast<int>(scores.size(2));
        const int width = static_cast<int>(scores.size(3));
        torch::Tensor heatmap = scores.permute({0, 2, 3, 1}).reshape({batch, height, width, 8, 8});
        return heatmap.permute({0, 1, 3, 2, 4}).reshape({batch, 1, height * 8, width * 8});
    }

    torch::Tensor Nms(torch::Tensor x, float threshold = 0.05f, int kernelSize = 5) const
    {
        const int batch = static_cast<int>(x.size(0));
        const int pad = kernelSize / 2;
        torch::Tensor localMax = torch::nn::functional::max_pool2d(
            x, torch::nn::functional::MaxPool2dFuncOptions(kernelSize).stride(1).padding(pad));
        torch::Tensor pos = (x == localMax) & (x > threshold);
        std::vector<torch::Tensor> positions;
        positions.reserve(batch);
        int maxCount = 0;
        for (int b = 0; b < batch; ++b) {
            torch::Tensor coords = pos[b].nonzero();
            coords = coords.index({torch::indexing::Ellipsis, torch::indexing::Slice(1, torch::indexing::None)}).flip(-1);
            maxCount = std::max(maxCount, static_cast<int>(coords.size(0)));
            positions.push_back(coords);
        }
        torch::Tensor packed =
            torch::zeros({batch, maxCount, 2}, torch::TensorOptions().dtype(torch::kLong).device(x.device()));
        for (int b = 0; b < batch; ++b) {
            if (positions[b].size(0) > 0) {
                packed[b].narrow(0, 0, positions[b].size(0)) = positions[b];
            }
        }
        return packed;
    }

    bool DetectAndComputeBatch(const std::vector<cv::Mat> &grayImages, int maxPoints, std::vector<XFeatFeatureSet> &outputs,
                               std::string *err) const
    {
        outputs.assign(grayImages.size(), XFeatFeatureSet{});
        if (grayImages.empty()) {
            if (err != nullptr) {
                *err = "xfeat native batch is empty";
            }
            return false;
        }

        torch::Tensor batch = BuildBatch(grayImages);
        double ratioH = 1.0;
        double ratioW = 1.0;
        std::tie(batch, ratioH, ratioW) = PreprocessTensor(batch);
        const int resizedH = static_cast<int>(batch.size(2));
        const int resizedW = static_cast<int>(batch.size(3));

        torch::Tensor denseFeatures;
        torch::Tensor keypointLogits;
        torch::Tensor reliability;
        std::tie(denseFeatures, keypointLogits, reliability) = model->forward(batch);
        denseFeatures =
            torch::nn::functional::normalize(denseFeatures, torch::nn::functional::NormalizeFuncOptions().dim(1));

        torch::Tensor heatmap = GetKptsHeatmap(keypointLogits);
        torch::Tensor mkpts = Nms(heatmap, 0.05f, 5);
        torch::Tensor scores =
            (nearest->forward(heatmap, mkpts, resizedH, resizedW) *
             bilinear->forward(reliability, mkpts, resizedH, resizedW))
                .squeeze(-1);
        torch::Tensor mask = torch::all(mkpts == 0, -1);
        scores.masked_fill_(mask, -1);

        torch::Tensor idxs = scores.neg().argsort(-1, false);
        const int keepCount = std::max(1, maxPoints);
        torch::Tensor mkptsX = mkpts.index({torch::indexing::Ellipsis, 0})
                                   .gather(-1, idxs)
                                   .index({torch::indexing::Slice(), torch::indexing::Slice(0, keepCount)});
        torch::Tensor mkptsY = mkpts.index({torch::indexing::Ellipsis, 1})
                                   .gather(-1, idxs)
                                   .index({torch::indexing::Slice(), torch::indexing::Slice(0, keepCount)});
        mkpts = torch::cat({mkptsX.unsqueeze(-1), mkptsY.unsqueeze(-1)}, -1);
        scores = scores.gather(-1, idxs).index({torch::indexing::Slice(), torch::indexing::Slice(0, keepCount)});

        torch::Tensor descriptors = bilinear->forward(denseFeatures, mkpts, resizedH, resizedW);
        descriptors =
            torch::nn::functional::normalize(descriptors, torch::nn::functional::NormalizeFuncOptions().dim(-1));
        torch::Tensor scaleTensor = torch::tensor({ratioW, ratioH}, mkpts.options()).view({1, 1, 2});
        mkpts = mkpts * scaleTensor;

        torch::Tensor keypointsCpu = mkpts.to(torch::kCPU);
        torch::Tensor scoresCpu = scores.to(torch::kCPU);
        torch::Tensor descriptorsCpu = descriptors.to(torch::kCPU);

        for (size_t batchIndex = 0; batchIndex < grayImages.size(); ++batchIndex) {
            torch::Tensor valid = scoresCpu[static_cast<long long>(batchIndex)] > 0;
            torch::Tensor validPoints = keypointsCpu[static_cast<long long>(batchIndex)].index({valid});
            torch::Tensor validDescriptors = descriptorsCpu[static_cast<long long>(batchIndex)].index({valid});
            XFeatFeatureSet &output = outputs[batchIndex];
            const int count = static_cast<int>(validPoints.size(0));
            output.keypoints.reserve(static_cast<size_t>(count));
            if (count > 0) {
                output.descriptors = cv::Mat(count, kDescriptorDim, CV_32F);
            }
            for (int i = 0; i < count; ++i) {
                const float x = validPoints[i][0].item<float>();
                const float y = validPoints[i][1].item<float>();
                output.keypoints.emplace_back(x, y);
                std::memcpy(output.descriptors.ptr<float>(i), validDescriptors[i].data_ptr(), kDescriptorDim * sizeof(float));
            }
        }
        return true;
    }
};

#else

struct XFeatNativeExtractor::Impl {};

#endif

XFeatNativeExtractor::~XFeatNativeExtractor() = default;

bool XFeatNativeExtractor::Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints,
                                 std::string *err)
{
    m_repoPath = repoPath;
    m_device = device;
    m_topK = topK;
    m_maxPoints = maxPoints;
    m_lastStats = Stats{};
#if defined(SMART_DRONE_XFEAT_NATIVE_AVAILABLE)
    m_impl = std::make_unique<Impl>();
    if (!m_impl->Load(repoPath, device, err)) {
        m_impl.reset();
        m_running = false;
        return false;
    }
    m_running = true;
    return true;
#else
    m_running = false;
    if (err != nullptr) {
        *err = "native xfeat backend is compiled out; rebuild with SMART_DRONE_ENABLE_XFEAT_NATIVE=ON and libtorch";
    }
    return false;
#endif
}

void XFeatNativeExtractor::Stop()
{
    m_running = false;
    m_lastStats = Stats{};
    m_impl.reset();
}

bool XFeatNativeExtractor::Running() const { return m_running; }

XFeatNativeExtractor::Stats XFeatNativeExtractor::LastStats() const { return m_lastStats; }

bool XFeatNativeExtractor::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
{
    XFeatFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool XFeatNativeExtractor::DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err)
{
    outFeatures.keypoints.clear();
    outFeatures.descriptors.release();
    m_lastStats = Stats{};
#if defined(SMART_DRONE_XFEAT_NATIVE_AVAILABLE)
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "xfeat native backend not running";
        }
        return false;
    }
    const auto totalStartTp = std::chrono::steady_clock::now();
    cv::Mat gray8;
    const auto prepareStartTp = totalStartTp;
    if (!PrepareGrayImage(gray, gray8, err)) {
        return false;
    }
    const auto prepareEndTp = std::chrono::steady_clock::now();
    std::vector<XFeatFeatureSet> outputs;
    const auto inferStartTp = prepareEndTp;
    if (!m_impl->DetectAndComputeBatch({gray8}, m_maxPoints, outputs, err) || outputs.empty()) {
        return false;
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    outFeatures = std::move(outputs.front());
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.imageCount = 1;
    m_lastStats.payloadBytes = static_cast<uint32_t>(gray8.total());
    return true;
#else
    if (err != nullptr) {
        *err = "native xfeat backend is compiled out";
    }
    (void)gray;
    return false;
#endif
}

bool XFeatNativeExtractor::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                  XFeatFeatureSet &leftFeatures, XFeatFeatureSet &rightFeatures,
                                                  std::string *err)
{
    leftFeatures.keypoints.clear();
    leftFeatures.descriptors.release();
    rightFeatures.keypoints.clear();
    rightFeatures.descriptors.release();
    m_lastStats = Stats{};
#if defined(SMART_DRONE_XFEAT_NATIVE_AVAILABLE)
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "xfeat native backend not running";
        }
        return false;
    }
    const auto totalStartTp = std::chrono::steady_clock::now();
    cv::Mat leftGray8;
    cv::Mat rightGray8;
    const auto prepareStartTp = totalStartTp;
    if (!PrepareGrayImage(leftGray, leftGray8, err) || !PrepareGrayImage(rightGray, rightGray8, err)) {
        return false;
    }
    const auto prepareEndTp = std::chrono::steady_clock::now();
    std::vector<XFeatFeatureSet> outputs;
    const auto inferStartTp = prepareEndTp;
    if (!m_impl->DetectAndComputeBatch({leftGray8, rightGray8}, m_maxPoints, outputs, err) || outputs.size() != 2) {
        return false;
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    leftFeatures = std::move(outputs[0]);
    rightFeatures = std::move(outputs[1]);
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.imageCount = 2;
    m_lastStats.payloadBytes = static_cast<uint32_t>(leftGray8.total() + rightGray8.total());
    return true;
#else
    if (err != nullptr) {
        *err = "native xfeat backend is compiled out";
    }
    (void)leftGray;
    (void)rightGray;
    return false;
#endif
}

} // namespace smartdrone::adapters::slam
