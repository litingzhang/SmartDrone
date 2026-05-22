#include "adapters/slam/superpoint_lightglue_frontend_client.h"

#include "adapters/slam/superpoint_native_extractor.h"

#include <utility>

namespace SmartDrone::Adapters::Slam {

namespace {

std::unique_ptr<IManagedVisualFeatureFrontend>
CreateSuperPointLightGlueFrontendClient()
{
    return std::make_unique<SuperPointLightGlueFrontendClient>();
}

const VisualFeatureFrontendClientRegistrar
    kSuperPointLightGlueFrontendClientRegistration(
        FeatureFrontend::SuperPointLightGlue,
        &CreateSuperPointLightGlueFrontendClient);

} // namespace

SuperPointLightGlueFrontendClient::SuperPointLightGlueFrontendClient() =
    default;

SuperPointLightGlueFrontendClient::~SuperPointLightGlueFrontendClient()
{
    Stop();
}

bool SuperPointLightGlueFrontendClient::Start(
    const VisualFeatureFrontendRuntimeConfig &config, std::string *err)
{
    return Start(config.repoPath, config.device, config.topK, config.maxPoints,
                 err);
}

bool SuperPointLightGlueFrontendClient::Start(const std::string &repoPath,
                                              const std::string &device,
                                              int topK, int maxPoints,
                                              std::string *err)
{
    Stop();
    m_lastStats = Stats{};
    if (!m_superPointNativeExtractor) {
        m_superPointNativeExtractor = std::make_unique<SuperPointNativeExtractor>();
    }
    if (!m_superPointNativeExtractor->Start(repoPath, device, topK, maxPoints,
                                            err)) {
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

SuperPointLightGlueFrontendClient::Stats
SuperPointLightGlueFrontendClient::LastStats() const
{
    return m_lastStats;
}

void SuperPointLightGlueFrontendClient::CopyNativeStats()
{
    if (!m_superPointNativeExtractor) {
        m_lastStats = Stats{};
        return;
    }
    const SuperPointNativeExtractor::Stats nativeStats =
        m_superPointNativeExtractor->LastStats();
    m_lastStats.prepareMs = nativeStats.prepareMs;
    m_lastStats.inputMs = nativeStats.inputMs;
    m_lastStats.forwardMs = nativeStats.forwardMs;
    m_lastStats.totalMs = nativeStats.totalMs;
    m_lastStats.rawLeftCount = nativeStats.rawLeftCount;
    m_lastStats.rawRightCount = nativeStats.rawRightCount;
    m_lastStats.stereoLeftCount = nativeStats.stereoLeftCount;
    m_lastStats.stereoRightCount = nativeStats.stereoRightCount;
    m_lastStats.lightGlueUsed = nativeStats.lightGlueUsed;
    m_lastStats.descriptorFallbackUsed = nativeStats.descriptorFallbackUsed;
    m_lastStats.imageCount = nativeStats.imageCount;
    m_lastStats.payloadBytes = nativeStats.payloadBytes;
}

bool SuperPointLightGlueFrontendClient::Detect(
    const Core::Ports::VisualFeatureDetectRequest &request,
    Core::Ports::VisualFeatureDetectResult &result)
{
    result = {};
    if (!m_superPointNativeExtractor) {
        result.error = "SuperPoint TensorRT frontend is not running";
        return false;
    }
    if (request.gray == nullptr) {
        result.error = "SuperPoint detect input is null";
        return false;
    }
    const bool ok =
        m_superPointNativeExtractor->Detect(*request.gray, result.points,
                                            &result.error);
    CopyNativeStats();
    return ok;
}

bool SuperPointLightGlueFrontendClient::DetectAndCompute(
    const Core::Ports::VisualFeatureComputeRequest &request,
    Core::Ports::VisualFeatureComputeResult &result)
{
    result = {};
    if (!m_superPointNativeExtractor) {
        result.error = "SuperPoint TensorRT frontend is not running";
        return false;
    }
    if (request.gray == nullptr) {
        result.error = "SuperPoint feature input is null";
        return false;
    }
    const bool ok = m_superPointNativeExtractor->DetectAndCompute(
        *request.gray, result.features, &result.error);
    CopyNativeStats();
    return ok;
}

bool SuperPointLightGlueFrontendClient::DetectAndComputeStereo(
    const Core::Ports::StereoVisualFeatureComputeRequest &request,
    Core::Ports::StereoVisualFeatureComputeResult &result)
{
    result = {};
    if (!m_superPointNativeExtractor) {
        result.error = "SuperPoint TensorRT frontend is not running";
        return false;
    }
    if (request.leftGray == nullptr || request.rightGray == nullptr) {
        result.error = "SuperPoint stereo feature input is null";
        return false;
    }
    const bool ok = m_superPointNativeExtractor->DetectAndComputeStereo(
        *request.leftGray, *request.rightGray, result.leftFeatures,
        result.rightFeatures, &result.error);
    CopyNativeStats();
    return ok;
}

void SuperPointLightGlueFrontendClient::SetLightGlueEveryNOverride(int everyN)
{
    if (m_superPointNativeExtractor) {
        m_superPointNativeExtractor->SetLightGlueEveryNOverride(everyN);
    }
}

} // namespace SmartDrone::Adapters::Slam
