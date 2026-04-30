#include "adapters/slam/xfeat_frontend_client.h"

#include "adapters/slam/superpoint_native_extractor.h"

#include <utility>

namespace smartdrone::adapters::slam {

XFeatFrontendClient::XFeatFrontendClient() = default;

XFeatFrontendClient::~XFeatFrontendClient() { Stop(); }

bool XFeatFrontendClient::Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints,
                                std::string *err)
{
    Stop();
    m_lastStats = Stats{};
    if (!m_superPointNativeExtractor) {
        m_superPointNativeExtractor = std::make_unique<SuperPointNativeExtractor>();
    }
    if (!m_superPointNativeExtractor->Start(repoPath, device, topK, maxPoints, err)) {
        m_superPointNativeExtractor.reset();
        return false;
    }
    return true;
}

void XFeatFrontendClient::Stop()
{
    if (m_superPointNativeExtractor) {
        m_superPointNativeExtractor->Stop();
        m_superPointNativeExtractor.reset();
    }
    m_lastStats = Stats{};
}

bool XFeatFrontendClient::Running() const
{
    return m_superPointNativeExtractor && m_superPointNativeExtractor->Running();
}

XFeatFrontendClient::Stats XFeatFrontendClient::LastStats() const { return m_lastStats; }

void XFeatFrontendClient::CopyNativeStats()
{
    if (!m_superPointNativeExtractor) {
        m_lastStats = Stats{};
        return;
    }
    const SuperPointNativeExtractor::Stats nativeStats = m_superPointNativeExtractor->LastStats();
    m_lastStats.prepareMs = nativeStats.prepareMs;
    m_lastStats.writeMs = nativeStats.inputMs;
    m_lastStats.readMs = nativeStats.forwardMs;
    m_lastStats.totalMs = nativeStats.totalMs;
    m_lastStats.imageCount = nativeStats.imageCount;
    m_lastStats.payloadBytes = nativeStats.payloadBytes;
}

bool XFeatFrontendClient::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
{
    if (!m_superPointNativeExtractor) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT frontend is not running";
        }
        return false;
    }
    const bool ok = m_superPointNativeExtractor->Detect(gray, outPoints, err);
    CopyNativeStats();
    return ok;
}

bool XFeatFrontendClient::DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err)
{
    if (!m_superPointNativeExtractor) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT frontend is not running";
        }
        return false;
    }
    const bool ok = m_superPointNativeExtractor->DetectAndCompute(gray, outFeatures, err);
    CopyNativeStats();
    return ok;
}

bool XFeatFrontendClient::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                 XFeatFeatureSet &leftFeatures, XFeatFeatureSet &rightFeatures,
                                                 std::string *err)
{
    if (!m_superPointNativeExtractor) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT frontend is not running";
        }
        return false;
    }
    const bool ok =
        m_superPointNativeExtractor->DetectAndComputeStereo(leftGray, rightGray, leftFeatures, rightFeatures, err);
    CopyNativeStats();
    return ok;
}

} // namespace smartdrone::adapters::slam
