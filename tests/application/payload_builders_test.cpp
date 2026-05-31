#include "core/application/runtime/payload_builders.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_provider_metadata.h"

namespace {

using SmartDrone::Core::Application::BuildCapabilitiesPayload;
using SmartDrone::Core::Application::BuildConfigPayload;
using SmartDrone::Core::Application::CameraRuntimeProviderMetadata;
using SmartDrone::Core::Application::UnifiedConfig;

std::string PayloadText(const std::vector<uint8_t> &payload)
{
    return std::string(payload.begin(), payload.end());
}

bool Contains(const std::string &text, const std::string &needle)
{
    return text.find(needle) != std::string::npos;
}

CameraRuntimeProviderMetadata MakeCameraProvider()
{
    CameraRuntimeProviderMetadata provider{};
    provider.providerName = "uvc_stereo_opencv";
    provider.usesPackedStereo = true;
    return provider;
}

TEST(PayloadBuildersTest, ConfigPayloadIncludesAvoidanceSettings)
{
    UnifiedConfig config{};
    config.app.runtime.avoidanceRadiusM = 1.25f;
    config.app.runtime.avoidanceMinCloudPoints = 12;
    const std::string text = PayloadText(BuildConfigPayload(
        config, SmartDrone::Core::Domain::RuntimeMode::Slam,
        MakeCameraProvider()));

    EXPECT_TRUE(Contains(text, "avoidance.enabled="));
    EXPECT_TRUE(Contains(text, "avoidance.radius_m=1.25"));
    EXPECT_TRUE(Contains(text, "avoidance.max_point_age_ms="));
    EXPECT_TRUE(Contains(text, "avoidance.min_cloud_points=12"));
    EXPECT_TRUE(Contains(text, "avoidance.min_blocking_points="));
    EXPECT_TRUE(Contains(text, "avoidance.state_cmd=0xF6"));
}

TEST(PayloadBuildersTest, CapabilitiesPayloadIncludesAvoidanceEnvKeys)
{
    const std::string text =
        PayloadText(BuildCapabilitiesPayload(MakeCameraProvider()));

    EXPECT_TRUE(Contains(text, "avoidance.state_tlv_cmd=0xF6"));
    EXPECT_TRUE(Contains(text, "SMART_DRONE_AVOIDANCE_MIN_CLOUD_POINTS"));
    EXPECT_TRUE(Contains(text, "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD"));
}

} // namespace
