#include "adapters/slam/orbslam3_mode_strategy.h"
#include "adapters/slam/orbslam3_engine_access.h"
#include "adapters/slam/orbslam3_mode_common.h"

namespace smartdrone::adapters::slam {

void OrbSlam3Engine::SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client)
{
    m_modeState->m_superpointFrontendClient = client;
}

void OrbSlam3Engine::SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight)
{
    m_modeState->m_superpointInputMaxWidth = std::max(0, maxWidth);
    m_modeState->m_superpointInputMaxHeight = std::max(0, maxHeight);
}

FeatureFrontend OrbModeStrategy::Frontend() const { return FeatureFrontend::Orb; }

core::ports::SlamOutput OrbModeStrategy::Process(OrbSlam3Engine &engine,
                                                const core::ports::SlamInputBatch &input,
                                                bool extractFeatures, bool extractPointCloud)
{
    return ProcessOrbSlamBackend(engine, input, extractFeatures, extractPointCloud, false);
}

core::ports::SlamOutput OrbModeStrategy::ProcessOrbSlamBackend(OrbSlam3Engine &engine,
                                                               const core::ports::SlamInputBatch &input,
                                                               bool extractFeatures, bool extractPointCloud,
                                                               bool enableSuperPointLightGlue)
{
    core::ports::SlamOutput out{};
    ORB_SLAM3::System *system = OrbSlam3EngineAccess::System(engine);
    if (system == nullptr) {
        return out;
    }
    SlamModeSharedState &state = OrbSlam3EngineAccess::ModeState(engine);
    const OrbInputMode inputMode = OrbSlam3EngineAccess::InputMode(engine);
    const bool useImu = OrbSlam3EngineAccess::UseImu(engine);
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    state.ResetSuperPointStats();

    Sophus::SE3f tcw;
    const bool monoMode = (inputMode != OrbInputMode::Stereo);
    const cv::Mat &monoImage =
        (inputMode == OrbInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;
    cv::Mat preparedLeftImage;
    cv::Mat preparedRightImage;
    out.usedSuperPointFrontend = false;
    state.CopySuperPointStatsToOutput(out);

    const bool useExternalStereoFrontend = enableSuperPointLightGlue && !monoMode && state.m_superpointFrontendClient != nullptr &&
                                           state.m_superpointFrontendClient->Running();
    bool trackedByExternalFrontend = false;
    if (useExternalStereoFrontend) {
        const auto externalStartTp = std::chrono::steady_clock::now();
        const auto prepareStartTp = externalStartTp;
        cv::Mat leftPrepared = EnsureGray8(input.stereo.left.gray);
        cv::Mat rightPrepared = EnsureGray8(input.stereo.right.gray);
        if (state.m_lkCalibrationLoaded && !leftPrepared.empty() && !rightPrepared.empty()) {
            state.EnsureStereoRectifier(leftPrepared.size());
            if (!state.m_lkMap1x.empty() && !state.m_lkMap2x.empty()) {
                cv::Mat leftRect;
                cv::Mat rightRect;
                cv::remap(leftPrepared, leftRect, state.m_lkMap1x, state.m_lkMap1y, cv::INTER_LINEAR);
                cv::remap(rightPrepared, rightRect, state.m_lkMap2x, state.m_lkMap2y, cv::INTER_LINEAR);
                leftPrepared = std::move(leftRect);
                rightPrepared = std::move(rightRect);
            }
        }
        float leftScaleX = 1.0f;
        float leftScaleY = 1.0f;
        float rightScaleX = 1.0f;
        float rightScaleY = 1.0f;
        const cv::Mat leftInput = BuildSuperPointInputImage(leftPrepared, state.m_superpointInputMaxWidth, state.m_superpointInputMaxHeight,
                                                       leftScaleX, leftScaleY);
        const cv::Mat rightInput = BuildSuperPointInputImage(rightPrepared, state.m_superpointInputMaxWidth, state.m_superpointInputMaxHeight,
                                                        rightScaleX, rightScaleY);
        const auto inputEndTp = std::chrono::steady_clock::now();
        out.inputPrepareMs = std::chrono::duration<double, std::milli>(inputEndTp - prepareStartTp).count();
        SuperPointFeatureSet leftFeatures;
        SuperPointFeatureSet rightFeatures;
        std::string featureErr;
        const auto frontendStartTp = std::chrono::steady_clock::now();
        if (state.m_superpointFrontendClient->DetectAndComputeStereo(leftInput, rightInput, leftFeatures, rightFeatures,
                                                          &featureErr)) {
            const auto frontendEndTp = std::chrono::steady_clock::now();
            const ExternalFeatureFrontendClient::Stats stats = state.m_superpointFrontendClient->LastStats();
            state.m_lastSuperPointPrepareMs = stats.prepareMs;
            state.m_lastSuperPointInputMs = stats.inputMs;
            state.m_lastSuperPointForwardMs = stats.forwardMs;
            state.m_lastSuperPointFrontendMs = stats.totalMs;
            state.m_lastSuperPointImageCount = stats.imageCount;
            state.m_lastSuperPointPayloadBytes = stats.payloadBytes;
            state.m_lastSuperPointRawLeftCount = static_cast<int>(leftFeatures.keypoints.size());
            state.m_lastSuperPointRawRightCount = static_cast<int>(rightFeatures.keypoints.size());
            out.frontendMs = std::chrono::duration<double, std::milli>(frontendEndTp - frontendStartTp).count();

            RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
            RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);
            const auto matchStartTp = std::chrono::steady_clock::now();
            const std::vector<StereoMatchPair> rawMatches =
                BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
            const std::vector<StereoMatchPair> matches = FilterStereoPairsByDisparityConsistency(rawMatches);
            state.m_lastSuperPointStereoMatchMs =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
            out.stereoPairMs = state.m_lastSuperPointStereoMatchMs;
            state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matches.size());

            ORB_SLAM3::ExternalStereoFrameData externalData;
            ORB_SLAM3::Tracking *tracker = system->GetTracker();
            const auto packStartTp = std::chrono::steady_clock::now();
            std::vector<cv::Point2f> matchedLeftPoints;
            std::vector<cv::Point2f> matchedRightPoints;
            matchedLeftPoints.reserve(matches.size());
            matchedRightPoints.reserve(matches.size());
            for (const StereoMatchPair &match : matches) {
                if (match.leftIndex < 0 || match.rightIndex < 0 ||
                    static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
                    static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
                    continue;
                }
                matchedLeftPoints.push_back(leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]);
                matchedRightPoints.push_back(rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]);
            }
            if (FinalizeStereoExternalFromPairs(tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
                                                tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr,
                                                leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints,
                                                externalData)) {
                const auto packEndTp = std::chrono::steady_clock::now();
                out.externalPackMs = std::chrono::duration<double, std::milli>(packEndTp - packStartTp).count();
                const bool initializedForMonoAugmentation =
                    tracker != nullptr && tracker->mState != ORB_SLAM3::Tracking::NO_IMAGES_YET &&
                    tracker->mState != ORB_SLAM3::Tracking::NOT_INITIALIZED;
                if (initializedForMonoAugmentation) {
                    const auto augmentStartTp = std::chrono::steady_clock::now();
                    const size_t maxLeftFeatures =
                        EnvSizeValueClamped("SMART_DRONE_EXTERNAL_STEREO_MAX_LEFT_FEATURES",
                                            kExternalStereoMaxLeftFeatures, kExternalStereoMaxLeftFeatures,
                                            kExternalStereoMaxLeftFeaturesLimit);
                    AppendOrbLeftOnlyFeatures(tracker->GetLeftORBExtractor(), leftPrepared, externalData,
                                              maxLeftFeatures);
                    out.monoAugmentMs =
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - augmentStartTp)
                            .count();
                }
                state.m_lastSuperPointInjectedLeftCount = static_cast<int>(externalData.leftKeypoints.size());
                state.m_lastSuperPointInjectedRightCount = static_cast<int>(externalData.rightKeypoints.size());
                const auto trackStartTp = std::chrono::steady_clock::now();
                if (useImu) {
                    tcw = system->TrackStereoPreparedWithFeatures(leftPrepared, rightPrepared, externalData,
                                                                  input.frameTimeSec, input.imu);
                } else {
                    tcw = system->TrackStereoPreparedWithFeatures(leftPrepared, rightPrepared, externalData,
                                                                  input.frameTimeSec);
                }
                const auto trackEndTp = std::chrono::steady_clock::now();
                state.m_lastSuperPointTotalMs =
                    std::chrono::duration<double, std::milli>(trackEndTp - externalStartTp).count();
                out.orbTrackMs = std::chrono::duration<double, std::milli>(trackEndTp - trackStartTp).count();
                out.frontendMs = state.m_lastSuperPointFrontendMs > 0.0 ? state.m_lastSuperPointFrontendMs : out.frontendMs;
                state.CopySuperPointStatsToOutput(out);
                out.usedSuperPointFrontend = true;
                out.leftFeatures.reserve(externalData.leftKeypoints.size());
                out.rightFeatures.reserve(externalData.rightKeypoints.size());
                for (const cv::KeyPoint &kp : externalData.leftKeypoints) {
                    out.leftFeatures.push_back(kp.pt);
                }
                for (const cv::KeyPoint &kp : externalData.rightKeypoints) {
                    out.rightFeatures.push_back(kp.pt);
                }
                trackedByExternalFrontend = true;
            }
        }
    }

    if (!trackedByExternalFrontend) {
        const auto orbTrackStartTp = std::chrono::steady_clock::now();
        if (useImu) {
            if (monoMode) {
                tcw = system->TrackMonocular(monoImage, input.frameTimeSec, input.imu);
            } else {
                tcw = system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec,
                                          input.imu);
            }
        } else {
            if (monoMode) {
                tcw = system->TrackMonocular(monoImage, input.frameTimeSec);
            } else {
                tcw = system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec);
            }
        }
        const auto orbTrackEndTp = std::chrono::steady_clock::now();
        out.orbTrackMs = std::chrono::duration<double, std::milli>(orbTrackEndTp - orbTrackStartTp).count();
    }
    if (const ORB_SLAM3::Tracking *tracker = system->GetTracker()) {
        out.orbExtractMs = tracker->mCurrentFrame.mTimeORB_Ext;
        out.orbStereoMatchMs = tracker->mCurrentFrame.mTimeStereoMatch;
    }

    out.trackingState = system->GetTrackingState();
    out.mapId = system->GetCurrentMapId();
    out.matchesInliers = system->GetMatchesInliers();
    out.trackedMapPointCount = static_cast<uint32_t>(system->GetTrackedMapPointCount());
    out.localMapPointCount = static_cast<uint32_t>(system->GetLocalMapPointCount());

    const Sophus::SE3f twc = tcw.inverse();
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    out.poseValid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) && std::isfinite(q.w()) &&
                    std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

    if (EnvFlagEnabled("SMART_DRONE_POSE_STABILIZER", false)) {
        OrbSlam3EngineAccess::StabilizeOutputPose(engine, out.pose, out.poseValid, input.frameTimeSec,
                                                  out.trackingState);
    }

    const bool needVisualExtraction = extractPointCloud || extractFeatures;
    if (!needVisualExtraction) {
        return out;
    }

    const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    ORB_SLAM3::TrackedVisualData visual =
        system->ExtractTrackedVisualData(leftWidth, leftHeight, monoMode ? 0 : input.stereo.right.gray.cols,
                                         monoMode ? 0 : input.stereo.right.gray.rows, extractPointCloud, 120);
    out.matchesInliers = visual.matchesInliers;
    out.trackedMapPointCount = static_cast<uint32_t>(visual.trackedMapPointCount);
    out.localMapPointCount = static_cast<uint32_t>(visual.localMapPointCount);
    if (extractFeatures && !out.usedSuperPointFrontend) {
        if (inputMode == OrbInputMode::MonoRight) {
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



std::unique_ptr<SlamModeStrategy> CreateOrbModeStrategy()
{
    return std::make_unique<OrbModeStrategy>();
}

} // namespace smartdrone::adapters::slam
