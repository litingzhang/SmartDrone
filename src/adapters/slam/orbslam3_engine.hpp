#pragma once

#include <utility>
#include <cstdint>
#include <memory>
#include <cmath>
#include <vector>

#include <sophus/se3.hpp>

#include "ImuTypes.h"
#include "TrackedVisualData.h"
#include "System.h"
#include "core/domain/runtime_mode.hpp"
#include "core/ports/slam_engine.hpp"

namespace smartdrone::adapters::slam {

enum class OrbInputMode : uint8_t {
    Stereo,
    MonoLeft,
    MonoRight,
};

class OrbSlam3Engine final : public core::ports::ISlamEngine {
public:
    OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu)
        : m_system(std::move(system)), m_inputMode(inputMode), m_useImu(useImu)
    {
    }

    bool Start() override { return static_cast<bool>(m_system); }

    void SetOperationMode(core::domain::SlamOperationMode mode)
    {
        if (!m_system || m_operationMode == mode) {
            return;
        }

        const bool localizationOnly =
            mode == core::domain::SlamOperationMode::Localization ||
            mode == core::domain::SlamOperationMode::Relocalization ||
            mode == core::domain::SlamOperationMode::TrackingOnly;

        if (localizationOnly) {
            m_system->ActivateLocalizationMode();
        } else {
            m_system->DeactivateLocalizationMode();
        }
        m_operationMode = mode;
    }

    void Stop() override
    {
        if (m_system) {
            m_system->Shutdown();
        }
    }

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch& input,
                                    bool extractFeatures,
                                    bool extractPointCloud) override
    {
        core::ports::SlamOutput out{};
        if (!m_system) {
            return out;
        }

        Sophus::SE3f tcw;
        const bool monoMode = (m_inputMode != OrbInputMode::Stereo);
        const cv::Mat& monoImage =
            (m_inputMode == OrbInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;

        if (m_useImu) {
            if (monoMode) {
                tcw = m_system->TrackMonocular(monoImage, input.frameTimeSec, input.imu);
            } else {
                tcw = m_system->TrackStereo(
                    input.stereo.left.gray,
                    input.stereo.right.gray,
                    input.frameTimeSec,
                    input.imu);
            }
        } else {
            if (monoMode) {
                tcw = m_system->TrackMonocular(monoImage, input.frameTimeSec);
            } else {
                tcw = m_system->TrackStereo(
                    input.stereo.left.gray,
                    input.stereo.right.gray,
                    input.frameTimeSec);
            }
        }

        out.trackingState = m_system->GetTrackingState();
        out.mapId = m_system->GetCurrentMapId();

        const Sophus::SE3f twc = tcw.inverse();
        const Eigen::Vector3f t = twc.translation();
        const Eigen::Quaternionf q(twc.so3().unit_quaternion());
        out.poseValid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                        std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
        out.pose.valid = out.poseValid;
        out.pose.x = t.x();
        out.pose.y = t.y();
        out.pose.z = t.z();
        out.pose.qw = q.w();
        out.pose.qx = q.x();
        out.pose.qy = q.y();
        out.pose.qz = q.z();

        if (!extractFeatures && !extractPointCloud) {
            return out;
        }

        const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
        const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
        ORB_SLAM3::TrackedVisualData visual = m_system->ExtractTrackedVisualData(
            leftWidth,
            leftHeight,
            monoMode ? 0 : input.stereo.right.gray.cols,
            monoMode ? 0 : input.stereo.right.gray.rows,
            extractPointCloud,
            120);
        if (extractFeatures) {
            if (m_inputMode == OrbInputMode::MonoRight) {
                out.rightFeatures = std::move(visual.leftFeatures);
            } else {
                out.leftFeatures = std::move(visual.leftFeatures);
                out.rightFeatures = std::move(visual.rightFeatures);
            }
        }
        if (extractPointCloud) {
            out.pointCloudXyz = std::move(visual.pointCloudXyz);
        }
        return out;
    }

private:
    std::unique_ptr<ORB_SLAM3::System> m_system;
    OrbInputMode m_inputMode{OrbInputMode::Stereo};
    bool m_useImu{false};
    core::domain::SlamOperationMode m_operationMode{core::domain::SlamOperationMode::Mapping};
};

}  // namespace smartdrone::adapters::slam
