#include "adapters/slam/xfeat_frontend_client.h"
#include "adapters/slam/xfeat_native_extractor.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

namespace smartdrone::adapters::slam {

namespace {

constexpr std::array<char, 8> kReadyMagic{'X', 'F', 'W', 'K', 'R', 'D', 'Y', '1'};
constexpr float kTemporalMinSimilarity = 0.80f;
constexpr float kTemporalMinMargin = 0.02f;
constexpr size_t kTemporalMinStableCount = 64;
constexpr size_t kTemporalExtraBudget = 160;

struct RequestHeader {
    uint32_t seq{0};
    uint32_t imageCount{0};
};

struct ImageHeader {
    uint32_t rows{0};
    uint32_t cols{0};
    uint32_t bytes{0};
};

struct ResponseHeader {
    uint32_t seq{0};
    uint32_t imageCount{0};
};

struct FeatureHeader {
    uint32_t count{0};
    uint32_t descriptorDim{0};
};

std::string ErrnoMessage(const char *prefix)
{
    return std::string(prefix) + ": " + std::strerror(errno);
}

bool SetCloseOnExec(int fd, std::string *err)
{
    const int flags = fcntl(fd, F_GETFD);
    if (flags < 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("fcntl(F_GETFD) failed");
        }
        return false;
    }
    if (fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("fcntl(F_SETFD) failed");
        }
        return false;
    }
    return true;
}

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool WantsNativeBackend()
{
    const char *backend = std::getenv("SMART_DRONE_XFEAT_BACKEND");
    if (backend == nullptr) {
        return false;
    }
    return std::string(backend) == "native";
}

bool HasCompatibleDescriptors(const XFeatFeatureSet &a, const XFeatFeatureSet &b)
{
    return !a.descriptors.empty() && !b.descriptors.empty() && a.descriptors.type() == CV_32F &&
           b.descriptors.type() == CV_32F && a.descriptors.cols == b.descriptors.cols &&
           static_cast<int>(a.keypoints.size()) == a.descriptors.rows &&
           static_cast<int>(b.keypoints.size()) == b.descriptors.rows;
}

float DescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs)
{
    return lhs.dot(rhs);
}

} // namespace

XFeatFrontendClient::XFeatFrontendClient() = default;

XFeatFrontendClient::~XFeatFrontendClient() { Stop(); }

bool XFeatFrontendClient::Start(const std::string &pythonBin, const std::string &workerScript,
                                const std::string &repoPath, const std::string &device, int topK, int maxPoints,
                                std::string *err)
{
    Stop();
    m_lastStats = Stats{};

    if (WantsNativeBackend()) {
        if (!m_nativeExtractor) {
            m_nativeExtractor = std::make_unique<XFeatNativeExtractor>();
        }
        if (!m_nativeExtractor->Start(repoPath, device, topK, maxPoints, err)) {
            m_nativeExtractor.reset();
            m_backendMode = BackendMode::None;
            return false;
        }
        m_backendMode = BackendMode::Native;
        return true;
    }

    int stdinPipe[2]{-1, -1};
    int stdoutPipe[2]{-1, -1};
    if (pipe(stdinPipe) != 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("pipe(stdin) failed");
        }
        return false;
    }
    if (pipe(stdoutPipe) != 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("pipe(stdout) failed");
        }
        close(stdinPipe[0]);
        close(stdinPipe[1]);
        return false;
    }

    std::string pipeErr;
    if (!SetCloseOnExec(stdinPipe[1], &pipeErr) || !SetCloseOnExec(stdoutPipe[0], &pipeErr)) {
        if (err != nullptr) {
            *err = pipeErr;
        }
        close(stdinPipe[0]);
        close(stdinPipe[1]);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("fork failed");
        }
        close(stdinPipe[0]);
        close(stdinPipe[1]);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);
        return false;
    }

    if (pid == 0) {
        dup2(stdinPipe[0], STDIN_FILENO);
        dup2(stdoutPipe[1], STDOUT_FILENO);

        close(stdinPipe[0]);
        close(stdinPipe[1]);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);

        const std::string topKText = std::to_string(std::max(1, topK));
        const std::string maxPointsText = std::to_string(std::max(1, maxPoints));

        std::vector<char *> argv;
        argv.push_back(const_cast<char *>(pythonBin.c_str()));
        argv.push_back(const_cast<char *>(workerScript.c_str()));
        argv.push_back(const_cast<char *>("--repo"));
        argv.push_back(const_cast<char *>(repoPath.c_str()));
        argv.push_back(const_cast<char *>("--device"));
        argv.push_back(const_cast<char *>(device.c_str()));
        argv.push_back(const_cast<char *>("--top-k"));
        argv.push_back(const_cast<char *>(topKText.c_str()));
        argv.push_back(const_cast<char *>("--max-points"));
        argv.push_back(const_cast<char *>(maxPointsText.c_str()));
        argv.push_back(nullptr);

        execvp(pythonBin.c_str(), argv.data());
        _exit(127);
    }

    close(stdinPipe[0]);
    close(stdoutPipe[1]);

    m_stdinFd = stdinPipe[1];
    m_stdoutFd = stdoutPipe[0];
    m_pid = pid;
    m_requestSeq = 0;
    m_prevStereoLeftFeatures = XFeatFeatureSet{};
    m_havePrevStereoLeftFeatures = false;
    std::array<char, kReadyMagic.size()> ready{};
    if (!ReadExact(ready.data(), ready.size(), err)) {
        Stop();
        return false;
    }
    if (ready != kReadyMagic) {
        if (err != nullptr) {
            *err = "xfeat worker handshake mismatch";
        }
        Stop();
        return false;
    }
    m_backendMode = BackendMode::Worker;
    return true;
}

void XFeatFrontendClient::Stop()
{
    if (m_backendMode == BackendMode::Native && m_nativeExtractor) {
        m_nativeExtractor->Stop();
        m_nativeExtractor.reset();
    }
    if (m_stdinFd >= 0) {
        close(m_stdinFd);
        m_stdinFd = -1;
    }
    if (m_stdoutFd >= 0) {
        close(m_stdoutFd);
        m_stdoutFd = -1;
    }
    if (m_pid > 0) {
        kill(m_pid, SIGTERM);
        int status = 0;
        waitpid(m_pid, &status, 0);
        m_pid = -1;
    }
    m_requestSeq = 0;
    m_backendMode = BackendMode::None;
    m_prevStereoLeftFeatures = XFeatFeatureSet{};
    m_havePrevStereoLeftFeatures = false;
}

bool XFeatFrontendClient::Running() const
{
    if (m_backendMode == BackendMode::Native) {
        return m_nativeExtractor && m_nativeExtractor->Running();
    }
    return m_backendMode == BackendMode::Worker && m_pid > 0 && m_stdinFd >= 0 && m_stdoutFd >= 0;
}

XFeatFrontendClient::Stats XFeatFrontendClient::LastStats() const { return m_lastStats; }

bool XFeatFrontendClient::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
{
    if (m_backendMode == BackendMode::Native) {
        const bool ok = m_nativeExtractor && m_nativeExtractor->Detect(gray, outPoints, err);
        if (m_nativeExtractor) {
            const XFeatNativeExtractor::Stats nativeStats = m_nativeExtractor->LastStats();
            m_lastStats.prepareMs = nativeStats.prepareMs;
            m_lastStats.readMs = nativeStats.inferMs;
            m_lastStats.totalMs = nativeStats.totalMs;
            m_lastStats.imageCount = nativeStats.imageCount;
            m_lastStats.payloadBytes = nativeStats.payloadBytes;
        }
        return ok;
    }
    XFeatFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool XFeatFrontendClient::DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err)
{
    if (m_backendMode == BackendMode::Native) {
        m_lastStats = Stats{};
        const bool ok = m_nativeExtractor && m_nativeExtractor->DetectAndCompute(gray, outFeatures, err);
        if (m_nativeExtractor) {
            const XFeatNativeExtractor::Stats nativeStats = m_nativeExtractor->LastStats();
            m_lastStats.prepareMs = nativeStats.prepareMs;
            m_lastStats.readMs = nativeStats.inferMs;
            m_lastStats.totalMs = nativeStats.totalMs;
            m_lastStats.imageCount = nativeStats.imageCount;
            m_lastStats.payloadBytes = nativeStats.payloadBytes;
        }
        return ok;
    }
    outFeatures.keypoints.clear();
    outFeatures.descriptors.release();
    m_lastStats = Stats{};
    if (!Running()) {
        if (err != nullptr) {
            *err = "xfeat worker not running";
        }
        return false;
    }
    if (gray.empty()) {
        if (err != nullptr) {
            *err = "xfeat input frame is empty";
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

    const uint32_t seq = ++m_requestSeq;
    const RequestHeader header{seq, 1};
    const ImageHeader imageHeader{static_cast<uint32_t>(gray8.rows), static_cast<uint32_t>(gray8.cols),
                                  static_cast<uint32_t>(gray8.rows * gray8.cols)};
    const auto writeStartTp = std::chrono::steady_clock::now();
    if (!WriteExact(&header, sizeof(header), err)) {
        Stop();
        return false;
    }
    if (!WriteExact(&imageHeader, sizeof(imageHeader), err)) {
        Stop();
        return false;
    }
    if (!WriteExact(gray8.data, static_cast<size_t>(imageHeader.bytes), err)) {
        Stop();
        return false;
    }
    const auto writeEndTp = std::chrono::steady_clock::now();

    ResponseHeader response{};
    const auto readStartTp = writeEndTp;
    if (!ReadExact(&response, sizeof(response), err)) {
        Stop();
        return false;
    }
    if (response.seq != seq) {
        if (err != nullptr) {
            *err = "xfeat worker response sequence mismatch";
        }
        Stop();
        return false;
    }
    if (response.imageCount != 1) {
        if (err != nullptr) {
            *err = "xfeat worker response image count mismatch";
        }
        Stop();
        return false;
    }

    if (!ReadFeatureSet(outFeatures, err)) {
        Stop();
        return false;
    }
    const auto totalEndTp = std::chrono::steady_clock::now();
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.writeMs = DurationMs(writeStartTp, writeEndTp);
    m_lastStats.readMs = DurationMs(readStartTp, totalEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, totalEndTp);
    m_lastStats.imageCount = 1;
    m_lastStats.payloadBytes = imageHeader.bytes;
    return true;
}

std::vector<int> XFeatFrontendClient::ComputeTemporalStableIndices(const XFeatFeatureSet &previous,
                                                                   const XFeatFeatureSet &current)
{
    std::vector<int> stableIndices;
    if (!HasCompatibleDescriptors(previous, current)) {
        return stableIndices;
    }

    std::vector<int> bestCurrentForPrevious(static_cast<size_t>(previous.descriptors.rows), -1);
    std::vector<float> bestCurrentScore(static_cast<size_t>(previous.descriptors.rows), -1.0f);
    std::vector<float> secondCurrentScore(static_cast<size_t>(previous.descriptors.rows), -1.0f);
    std::vector<int> bestPreviousForCurrent(static_cast<size_t>(current.descriptors.rows), -1);
    std::vector<float> bestPreviousScore(static_cast<size_t>(current.descriptors.rows), -1.0f);

    for (int pi = 0; pi < previous.descriptors.rows; ++pi) {
        float bestScore = -1.0f;
        float secondScore = -1.0f;
        int bestIndex = -1;
        for (int ci = 0; ci < current.descriptors.rows; ++ci) {
            const float similarity = DescriptorSimilarity(previous.descriptors.row(pi), current.descriptors.row(ci));
            if (!std::isfinite(similarity)) {
                continue;
            }
            if (similarity > bestScore) {
                secondScore = bestScore;
                bestScore = similarity;
                bestIndex = ci;
            } else if (similarity > secondScore) {
                secondScore = similarity;
            }
        }
        if (bestIndex < 0 || bestScore < kTemporalMinSimilarity) {
            continue;
        }
        if (secondScore > -0.5f && (bestScore - secondScore) < kTemporalMinMargin) {
            continue;
        }
        bestCurrentForPrevious[static_cast<size_t>(pi)] = bestIndex;
        bestCurrentScore[static_cast<size_t>(pi)] = bestScore;
        secondCurrentScore[static_cast<size_t>(pi)] = secondScore;
        if (bestScore > bestPreviousScore[static_cast<size_t>(bestIndex)]) {
            bestPreviousScore[static_cast<size_t>(bestIndex)] = bestScore;
            bestPreviousForCurrent[static_cast<size_t>(bestIndex)] = pi;
        }
    }

    std::vector<std::pair<int, float>> ranked;
    ranked.reserve(current.keypoints.size());
    for (size_t pi = 0; pi < bestCurrentForPrevious.size(); ++pi) {
        const int ci = bestCurrentForPrevious[pi];
        if (ci < 0) {
            continue;
        }
        if (bestPreviousForCurrent[static_cast<size_t>(ci)] != static_cast<int>(pi)) {
            continue;
        }
        ranked.emplace_back(ci, bestCurrentScore[pi]);
    }

    std::sort(ranked.begin(), ranked.end(), [](const auto &lhs, const auto &rhs) { return lhs.second > rhs.second; });
    stableIndices.reserve(ranked.size());
    for (const auto &[index, _score] : ranked) {
        stableIndices.push_back(index);
    }
    return stableIndices;
}

void XFeatFrontendClient::ReorderFeaturesByIndices(const XFeatFeatureSet &source, const std::vector<int> &indices,
                                                   XFeatFeatureSet &dest)
{
    dest.keypoints.clear();
    dest.descriptors.release();
    if (indices.empty() || source.descriptors.empty()) {
        return;
    }

    const int descriptorDim = source.descriptors.cols;
    cv::Mat reordered(static_cast<int>(indices.size()), descriptorDim, source.descriptors.type());
    dest.keypoints.reserve(indices.size());
    for (size_t outIdx = 0; outIdx < indices.size(); ++outIdx) {
        const int srcIdx = indices[outIdx];
        if (srcIdx < 0 || srcIdx >= source.descriptors.rows || static_cast<size_t>(srcIdx) >= source.keypoints.size()) {
            continue;
        }
        dest.keypoints.push_back(source.keypoints[static_cast<size_t>(srcIdx)]);
        source.descriptors.row(srcIdx).copyTo(reordered.row(static_cast<int>(dest.keypoints.size() - 1)));
    }
    if (!dest.keypoints.empty()) {
        if (static_cast<int>(dest.keypoints.size()) != reordered.rows) {
            reordered = reordered.rowRange(0, static_cast<int>(dest.keypoints.size())).clone();
        }
        dest.descriptors = std::move(reordered);
    }
}

bool XFeatFrontendClient::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                 XFeatFeatureSet &leftFeatures, XFeatFeatureSet &rightFeatures,
                                                 std::string *err)
{
    if (m_backendMode == BackendMode::Native) {
        m_lastStats = Stats{};
        const bool ok = m_nativeExtractor &&
                        m_nativeExtractor->DetectAndComputeStereo(leftGray, rightGray, leftFeatures, rightFeatures, err);
        if (m_nativeExtractor) {
            const XFeatNativeExtractor::Stats nativeStats = m_nativeExtractor->LastStats();
            m_lastStats.prepareMs = nativeStats.prepareMs;
            m_lastStats.readMs = nativeStats.inferMs;
            m_lastStats.totalMs = nativeStats.totalMs;
            m_lastStats.imageCount = nativeStats.imageCount;
            m_lastStats.payloadBytes = nativeStats.payloadBytes;
        }
        if (ok) {
            const XFeatFeatureSet rawLeft = leftFeatures;
            const std::vector<int> stableIndices =
                m_havePrevStereoLeftFeatures ? ComputeTemporalStableIndices(m_prevStereoLeftFeatures, rawLeft)
                                             : std::vector<int>{};
            if (stableIndices.size() >= kTemporalMinStableCount) {
                std::vector<int> selected = stableIndices;
                const size_t extrasAllowed = std::min(kTemporalExtraBudget, rawLeft.keypoints.size());
                std::unordered_set<int> seen(selected.begin(), selected.end());
                for (int idx = 0; idx < static_cast<int>(rawLeft.keypoints.size()) &&
                                  selected.size() < stableIndices.size() + extrasAllowed;
                     ++idx) {
                    if (seen.insert(idx).second) {
                        selected.push_back(idx);
                    }
                }
                XFeatFeatureSet reordered;
                ReorderFeaturesByIndices(rawLeft, selected, reordered);
                if (!reordered.keypoints.empty() && !reordered.descriptors.empty()) {
                    leftFeatures = std::move(reordered);
                }
            }
            m_prevStereoLeftFeatures = rawLeft;
            m_havePrevStereoLeftFeatures = true;
        }
        return ok;
    }
    leftFeatures.keypoints.clear();
    leftFeatures.descriptors.release();
    rightFeatures.keypoints.clear();
    rightFeatures.descriptors.release();
    m_lastStats = Stats{};
    if (!Running()) {
        if (err != nullptr) {
            *err = "xfeat worker not running";
        }
        return false;
    }
    if (leftGray.empty() || rightGray.empty()) {
        if (err != nullptr) {
            *err = "xfeat stereo input frame is empty";
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

    const uint32_t seq = ++m_requestSeq;
    const RequestHeader header{seq, 2};
    const ImageHeader leftHeader{static_cast<uint32_t>(leftGray8.rows), static_cast<uint32_t>(leftGray8.cols),
                                 static_cast<uint32_t>(leftGray8.rows * leftGray8.cols)};
    const ImageHeader rightHeader{static_cast<uint32_t>(rightGray8.rows), static_cast<uint32_t>(rightGray8.cols),
                                  static_cast<uint32_t>(rightGray8.rows * rightGray8.cols)};
    const auto writeStartTp = std::chrono::steady_clock::now();
    if (!WriteExact(&header, sizeof(header), err) || !WriteExact(&leftHeader, sizeof(leftHeader), err) ||
        !WriteExact(leftGray8.data, static_cast<size_t>(leftHeader.bytes), err) ||
        !WriteExact(&rightHeader, sizeof(rightHeader), err) ||
        !WriteExact(rightGray8.data, static_cast<size_t>(rightHeader.bytes), err)) {
        Stop();
        return false;
    }
    const auto writeEndTp = std::chrono::steady_clock::now();

    ResponseHeader response{};
    const auto readStartTp = writeEndTp;
    if (!ReadExact(&response, sizeof(response), err)) {
        Stop();
        return false;
    }
    if (response.seq != seq) {
        if (err != nullptr) {
            *err = "xfeat worker response sequence mismatch";
        }
        Stop();
        return false;
    }
    if (response.imageCount != 2) {
        if (err != nullptr) {
            *err = "xfeat worker stereo response image count mismatch";
        }
        Stop();
        return false;
    }
    if (!ReadFeatureSet(leftFeatures, err) || !ReadFeatureSet(rightFeatures, err)) {
        Stop();
        return false;
    }

    const XFeatFeatureSet rawLeft = leftFeatures;
    const std::vector<int> stableIndices =
        m_havePrevStereoLeftFeatures ? ComputeTemporalStableIndices(m_prevStereoLeftFeatures, rawLeft)
                                     : std::vector<int>{};
    if (stableIndices.size() >= kTemporalMinStableCount) {
        std::vector<int> selected = stableIndices;
        const size_t extrasAllowed = std::min(kTemporalExtraBudget, rawLeft.keypoints.size());
        std::unordered_set<int> seen(selected.begin(), selected.end());
        for (int idx = 0; idx < static_cast<int>(rawLeft.keypoints.size()) &&
                          selected.size() < stableIndices.size() + extrasAllowed;
             ++idx) {
            if (seen.insert(idx).second) {
                selected.push_back(idx);
            }
        }
        XFeatFeatureSet reordered;
        ReorderFeaturesByIndices(rawLeft, selected, reordered);
        if (!reordered.keypoints.empty() && !reordered.descriptors.empty()) {
            leftFeatures = std::move(reordered);
        }
    }
    m_prevStereoLeftFeatures = rawLeft;
    m_havePrevStereoLeftFeatures = true;

    const auto totalEndTp = std::chrono::steady_clock::now();
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.writeMs = DurationMs(writeStartTp, writeEndTp);
    m_lastStats.readMs = DurationMs(readStartTp, totalEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, totalEndTp);
    m_lastStats.imageCount = 2;
    m_lastStats.payloadBytes = leftHeader.bytes + rightHeader.bytes;
    return true;
}

bool XFeatFrontendClient::PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err)
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
            *err = "xfeat gray image preparation failed";
        }
        return false;
    }
    return true;
}

bool XFeatFrontendClient::ReadFeatureSet(XFeatFeatureSet &outFeatures, std::string *err)
{
    FeatureHeader featureHeader{};
    if (!ReadExact(&featureHeader, sizeof(featureHeader), err)) {
        return false;
    }

    const size_t xyCount = static_cast<size_t>(featureHeader.count) * 2u;
    std::vector<float> xy(xyCount, 0.0f);
    if (!xy.empty() && !ReadExact(xy.data(), xy.size() * sizeof(float), err)) {
        return false;
    }
    const size_t descriptorValueCount =
        static_cast<size_t>(featureHeader.count) * static_cast<size_t>(featureHeader.descriptorDim);
    std::vector<float> descriptors(descriptorValueCount, 0.0f);
    if (!descriptors.empty() && !ReadExact(descriptors.data(), descriptors.size() * sizeof(float), err)) {
        return false;
    }

    outFeatures.keypoints.reserve(featureHeader.count);
    for (uint32_t i = 0; i < featureHeader.count; ++i) {
        outFeatures.keypoints.emplace_back(xy[static_cast<size_t>(i) * 2u], xy[static_cast<size_t>(i) * 2u + 1u]);
    }
    if (featureHeader.count > 0 && featureHeader.descriptorDim > 0) {
        cv::Mat desc(static_cast<int>(featureHeader.count), static_cast<int>(featureHeader.descriptorDim), CV_32F);
        std::memcpy(desc.data, descriptors.data(), descriptors.size() * sizeof(float));
        outFeatures.descriptors = std::move(desc);
    }
    return true;
}

bool XFeatFrontendClient::WriteExact(const void *data, size_t size, std::string *err)
{
    const uint8_t *cursor = static_cast<const uint8_t *>(data);
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t written = write(m_stdinFd, cursor, remaining);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (err != nullptr) {
                *err = ErrnoMessage("xfeat worker write failed");
            }
            return false;
        }
        if (written == 0) {
            if (err != nullptr) {
                *err = "xfeat worker write returned 0";
            }
            return false;
        }
        cursor += static_cast<size_t>(written);
        remaining -= static_cast<size_t>(written);
    }
    return true;
}

bool XFeatFrontendClient::ReadExact(void *data, size_t size, std::string *err)
{
    uint8_t *cursor = static_cast<uint8_t *>(data);
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t readBytes = read(m_stdoutFd, cursor, remaining);
        if (readBytes < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (err != nullptr) {
                *err = ErrnoMessage("xfeat worker read failed");
            }
            return false;
        }
        if (readBytes == 0) {
            if (err != nullptr) {
                *err = "xfeat worker closed stdout";
            }
            return false;
        }
        cursor += static_cast<size_t>(readBytes);
        remaining -= static_cast<size_t>(readBytes);
    }
    return true;
}

} // namespace smartdrone::adapters::slam
