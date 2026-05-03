#include "adapters/slam/superpoint_lightglue_frontend_client.h"

#include "adapters/slam/superpoint_native_extractor.h"

#include <utility>

namespace smartdrone::adapters::slam {

SuperPointLightGlueFrontendClient::SuperPointLightGlueFrontendClient() = default;

SuperPointLightGlueFrontendClient::~SuperPointLightGlueFrontendClient() { Stop(); }

bool SuperPointLightGlueFrontendClient::Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints,
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

void SuperPointLightGlueFrontendClient::Stop()
{
    if (m_superPointNativeExtractor) {
        m_superPointNativeExtractor->Stop();
        m_superPointNativeExtractor.reset();
    }
    m_lastStats = Stats{};
}

bool SuperPointLightGlueFrontendClient::Running() const
{
    return m_superPointNativeExtractor && m_superPointNativeExtractor->Running();
}

SuperPointLightGlueFrontendClient::Stats SuperPointLightGlueFrontendClient::LastStats() const { return m_lastStats; }

void SuperPointLightGlueFrontendClient::CopyNativeStats()
{
    if (!m_superPointNativeExtractor) {
        m_lastStats = Stats{};
        return;
    }
    const SuperPointNativeExtractor::Stats nativeStats = m_superPointNativeExtractor->LastStats();
    m_lastStats.prepareMs = nativeStats.prepareMs;
    m_lastStats.inputMs = nativeStats.inputMs;
    m_lastStats.forwardMs = nativeStats.forwardMs;
    m_lastStats.totalMs = nativeStats.totalMs;
    m_lastStats.imageCount = nativeStats.imageCount;
    m_lastStats.payloadBytes = nativeStats.payloadBytes;
}

bool SuperPointLightGlueFrontendClient::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
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

bool SuperPointLightGlueFrontendClient::DetectAndCompute(const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err)
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

bool SuperPointLightGlueFrontendClient::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                 SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures,
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
