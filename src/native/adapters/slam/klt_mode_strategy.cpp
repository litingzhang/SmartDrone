#include "adapters/slam/slam_mode_strategy.h"
#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_mode_common.h"

#include <cctype>

namespace smartdrone::adapters::slam {

void SlamEngineAdapter::SetStereoVoLoopClosure(bool enabled, float scale, float relaxation)
{
    m_modeState->m_lkLoopClosureEnabled = enabled;
    m_modeState->m_lkLoopScale = std::clamp(scale, 0.25f, 4.0f);
    m_modeState->m_lkLoopRelaxation = std::clamp(relaxation, 0.0f, 4.0f);
}

void SlamEngineAdapter::SetStereoVoPerFrameAcceleration(std::string acceleration)
{
    std::transform(acceleration.begin(), acceleration.end(), acceleration.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (acceleration.empty()) {
        acceleration = "auto";
    }
    if (m_modeState->m_lkPerFrameAcceleration == acceleration) {
        return;
    }
    m_modeState->m_lkPerFrameAcceleration = std::move(acceleration);
    m_modeState->m_lkPerFrameVpi.reset();
    m_modeState->m_lkPerFrameAccelLogged = false;
}

FeatureFrontend KltModeStrategy::Frontend() const { return FeatureFrontend::LK; }

core::ports::SlamOutput KltModeStrategy::Process(SlamEngineAdapter &engine,
                                                const core::ports::SlamInputBatch &input,
                                                bool extractFeatures, bool extractPointCloud)
{
    (void)extractPointCloud;
    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);

    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.mapId = 1;
    out.trackingState = ORB_SLAM3::Tracking::OK;
    out.poseValid = true;
    out.pose.valid = true;
    out.usedSuperPointFrontend = false;
    state.ResetExternalFeatureStats();

    cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
    if (leftGray.empty() || rightGray.empty() || !state.m_lkCalibrationLoaded) {
        out.trackingState = ORB_SLAM3::Tracking::LOST;
        out.poseValid = false;
        out.pose.valid = false;
        return out;
    }

    state.EnsureStereoRectifier(leftGray.size());
    cv::Mat leftRect = leftGray;
    cv::Mat rightRect = rightGray;
    if (!state.m_lkMap1x.empty() && !state.m_lkMap2x.empty()) {
        cv::remap(leftGray, leftRect, state.m_lkMap1x, state.m_lkMap1y, cv::INTER_LINEAR);
        cv::remap(rightGray, rightRect, state.m_lkMap2x, state.m_lkMap2y, cv::INTER_LINEAR);
    }
    auto extractCurrentGfttSeedsIfNeeded = [&](bool force) {
        const bool cadenceDue = state.m_lkLastSeedFrameId == 0 ||
                                input.frameId >= state.m_lkLastSeedFrameId + kLkGridRefillIntervalFrames;
        if (!force && (!cadenceDue || !LkHasDegradedGridCell(state.m_lkTracks, leftRect.size()))) {
            return false;
        }
        std::vector<LkStereoTrack> seeds = BuildLkGfttStereoSeeds(leftRect, rightRect);
        state.m_lkLastSeedFrameId = input.frameId;
        if (seeds.empty()) {
            return false;
        }
        const size_t before = state.m_lkTracks.size();
        AppendLkSeedsForDegradedCells(seeds, leftRect.size(), state.m_lkTracks);
        return state.m_lkTracks.size() > before;
    };
    if (!state.m_lkHavePrev) {
        extractCurrentGfttSeedsIfNeeded(true);
        state.m_lkTracks = SelectLkTracksGridBalanced(state.m_lkTracks, leftRect.size());
        state.m_lkPrevLeft = leftRect.clone();
        state.m_lkPrevRight = rightRect.clone();
        state.m_lkTwc = Sophus::SE3f();
        state.m_lkLoopCorrection = Sophus::SE3f();
        state.m_lkLoopKeyframes.clear();
        state.m_lkLastLoopClosureFrameId = 0;
        state.m_lkHavePrev = true;
        state.m_lkFrameCount = 1;
        if (extractFeatures) {
            out.leftFeatures.reserve(state.m_lkTracks.size());
            out.rightFeatures.reserve(state.m_lkTracks.size());
            for (const LkStereoTrack &track : state.m_lkTracks) {
                out.leftFeatures.push_back(track.left);
                out.rightFeatures.push_back(track.right);
            }
        }
    } else {
        state.m_lkTracks = SelectLkTracksGridBalanced(state.m_lkTracks, state.m_lkPrevLeft.size());
        std::vector<cv::Point2f> pts0;
        pts0.reserve(state.m_lkTracks.size());
        for (const LkStereoTrack &track : state.m_lkTracks) {
            pts0.push_back(track.left);
        }
        std::vector<cv::Point2f> leftPts1;
        std::vector<cv::Point2f> rightPts1;
        std::vector<uchar> status;
        std::vector<uchar> rightStatus;
        if (!pts0.empty()) {
            std::vector<cv::Point2f> rightPts0;
            rightPts0.reserve(state.m_lkTracks.size());
            for (const LkStereoTrack &track : state.m_lkTracks) {
                rightPts0.push_back(track.right);
            }
            (void)TrackPointsWithForwardBackward(state.m_lkPrevLeft, leftRect, pts0, leftPts1, status);
            (void)TrackPointsWithForwardBackward(state.m_lkPrevRight, rightRect, rightPts0, rightPts1, rightStatus);
        }
        const bool horizontalLateralFlow = IsHorizontalLateralFlow(pts0, leftPts1, status, leftRect.size());

        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        objectPoints.reserve(pts0.size());
        imagePoints.reserve(pts0.size());
        std::vector<LkStereoTrack> trackedTracks;
        trackedTracks.reserve(state.m_lkTracks.size());
        cv::Mat leftRect32f;
        cv::Mat rightRect32f;
        leftRect.convertTo(leftRect32f, CV_32F);
        rightRect.convertTo(rightRect32f, CV_32F);
        for (size_t i = 0; i < state.m_lkTracks.size() && i < leftPts1.size() && i < rightPts1.size(); ++i) {
            if (i >= status.size() || i >= rightStatus.size() || !status[i] || !rightStatus[i]) {
                continue;
            }
            const LkStereoTrack &prevTrack = state.m_lkTracks[i];
            const cv::Point2f &p0 = prevTrack.left;
            const cv::Point2f &prevRight = prevTrack.right;
            const cv::Point2f &p1 = leftPts1[i];
            cv::Point2f right1 = rightPts1[i];
            if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= state.m_lkPrevLeft.cols - 1 || p0.y >= state.m_lkPrevLeft.rows - 1 ||
                p1.x < 1.0f || p1.y < 1.0f || p1.x >= leftRect.cols - 1 || p1.y >= leftRect.rows - 1 ||
                right1.x < 1.0f || right1.y < 1.0f || right1.x >= rightRect.cols - 1 ||
                right1.y >= rightRect.rows - 1) {
                continue;
            }
            if (cv::norm(p1 - p0) > kLkMaxFlowPx) {
                continue;
            }
            float zncc = -1.0f;
            if (!RefineRightPointByStereoZncc(leftRect32f, p1, rightRect32f, right1, right1, zncc)) {
                continue;
            }
            const float d = p0.x - prevRight.x;
            const float z = state.m_lkFx * state.m_lkBaseline / d;
            if (!(z >= kLkMinDepthMeters) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
                continue;
            }
            objectPoints.emplace_back((p0.x - state.m_lkCx) * z / state.m_lkFx, (p0.y - state.m_lkCy) * z / state.m_lkFy, z);
            imagePoints.push_back(p1);
            const float trackedQuality =
                std::clamp(prevTrack.quality * 0.92f + std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f) * 0.08f,
                           0.0f, 1.0f);
            trackedTracks.push_back(LkStereoTrack{p1, right1, trackedQuality, prevTrack.age + 1});
        }

        int inlierCount = 0;
        if (objectPoints.size() >= kLkMinPnPPoints) {
            cv::Mat rvec, tvec, inliers;
            const cv::Mat K = MakeCameraMatrix(state.m_lkFx, state.m_lkFy, state.m_lkCx, state.m_lkCy);
            bool ok = false;
            try {
                ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, tvec, false, 80, 4.0, 0.995,
                                        inliers, cv::SOLVEPNP_EPNP);
                inlierCount = inliers.rows;
            } catch (const cv::Exception &e) {
                std::cerr << "[lk_pnp] solvePnPRansac skipped points=" << objectPoints.size()
                          << " error=" << e.what() << "\n";
            }
            if (ok && inlierCount >= kLkMinPnPInliers) {
                std::vector<LkStereoTrack> inlierTracks;
                inlierTracks.reserve(static_cast<size_t>(inlierCount));
                for (int row = 0; row < inliers.rows; ++row) {
                    const int idx = inliers.at<int>(row, 0);
                    if (idx >= 0 && static_cast<size_t>(idx) < trackedTracks.size()) {
                        inlierTracks.push_back(trackedTracks[static_cast<size_t>(idx)]);
                    }
                }
                if (!inlierTracks.empty()) {
                    trackedTracks = std::move(inlierTracks);
                }
                cv::Mat Rcv;
                cv::Rodrigues(rvec, Rcv);
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f t = Eigen::Vector3f::Zero();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        R(r, c) = static_cast<float>(Rcv.at<double>(r, c));
                    }
                    t(r) = static_cast<float>(tvec.at<double>(r, 0));
                }
                if (std::isfinite(t.norm()) && t.norm() <= kLkMaxStepMeters) {
                    const Sophus::SE3f TcurrPrev(Sophus::SO3f(R), t);
                    state.m_lkTwc = state.m_lkTwc * StabilizeLkCameraDelta(TcurrPrev.inverse(), horizontalLateralFlow);
                }
            }
        }

        out.matchesInliers = inlierCount;
        out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
        out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
        const bool hardRecoveryRefill =
            trackedTracks.size() < static_cast<size_t>(kLkHardRecoveryMinTracks) || inlierCount < kLkHardRecoveryMinInliers;
        state.m_lkTracks = hardRecoveryRefill ? std::vector<LkStereoTrack>{}
                                        : SelectLkTracksGridBalanced(trackedTracks, leftRect.size());
        if (hardRecoveryRefill) {
            // A wide-baseline jump can invalidate LK-forwarded tracks. Seed directly from the current stereo frame.
            extractCurrentGfttSeedsIfNeeded(true);
        }
        extractCurrentGfttSeedsIfNeeded(false);
        state.m_lkTracks = SelectLkTracksGridBalanced(state.m_lkTracks, leftRect.size());
        if (extractFeatures) {
            out.leftFeatures.reserve(state.m_lkTracks.size());
            out.rightFeatures.reserve(state.m_lkTracks.size());
            for (const LkStereoTrack &track : state.m_lkTracks) {
                out.leftFeatures.push_back(track.left);
                out.rightFeatures.push_back(track.right);
            }
        }
        state.m_lkPrevLeft = leftRect.clone();
        state.m_lkPrevRight = rightRect.clone();
        ++state.m_lkFrameCount;
    }

    const Sophus::SE3f outputTwc = KltModeStrategy::ApplyLoopClosure(engine, leftRect, input.frameId, state.m_lkTwc);
    const Eigen::Vector3f t = outputTwc.translation();
    out.usedSuperPointFrontend = out.usedSuperPointFrontend || state.m_lastSuperPointInjectedLeftCount > 0;
    state.CopyExternalFeatureStatsToOutput(out);
    const Eigen::Quaternionf q(outputTwc.so3().unit_quaternion());
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();
    return out;
}



Sophus::SE3f KltModeStrategy::ApplyLoopClosure(SlamEngineAdapter &engine, const cv::Mat &leftRect,
                                               uint64_t frameId, const Sophus::SE3f &rawTwc)
{
    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    if (!state.m_lkLoopClosureEnabled || leftRect.empty()) {
        return rawTwc;
    }

    const cv::Mat descriptor = BuildLkLoopImageDescriptor(leftRect);
    if (descriptor.empty()) {
        return state.m_lkLoopCorrection * rawTwc;
    }

    const Sophus::SE3f scaledRaw(rawTwc.so3(), rawTwc.translation() * state.m_lkLoopScale);
    Sophus::SE3f corrected = state.m_lkLoopCorrection * scaledRaw;
    int bestIndex = -1;
    double bestSimilarity = -1.0;
    for (size_t i = 0; i < state.m_lkLoopKeyframes.size(); ++i) {
        const LkLoopKeyframe &keyframe = state.m_lkLoopKeyframes[i];
        if (frameId <= keyframe.frameId + kLkLoopMinAgeFrames) {
            continue;
        }
        const double similarity = LkLoopDescriptorSimilarity(descriptor, keyframe.descriptor);
        if (similarity > bestSimilarity) {
            bestSimilarity = similarity;
            bestIndex = static_cast<int>(i);
        }
    }

    if (bestIndex >= 0 && bestSimilarity >= kLkLoopMinSimilarity &&
        frameId >= state.m_lkLastLoopClosureFrameId + kLkLoopCooldownFrames) {
        const LkLoopKeyframe &loop = state.m_lkLoopKeyframes[static_cast<size_t>(bestIndex)];
        const Eigen::Vector3f residual = loop.correctedTwc.translation() - corrected.translation();
        state.m_lkLoopCorrection =
            Sophus::SE3f(state.m_lkLoopCorrection.so3(), state.m_lkLoopCorrection.translation() + residual * state.m_lkLoopRelaxation);
        corrected = state.m_lkLoopCorrection * scaledRaw;
        state.m_lkLastLoopClosureFrameId = frameId;
        std::cerr << "[lk_loop] visual loop frame=" << frameId << " match_frame=" << loop.frameId
                  << " similarity=" << bestSimilarity << " residual_m=" << residual.norm()
                  << " scale=" << state.m_lkLoopScale << " relaxation=" << state.m_lkLoopRelaxation << "\n";
    }

    const bool shouldAddKeyframe =
        state.m_lkLoopKeyframes.empty() || frameId >= state.m_lkLoopKeyframes.back().frameId + kLkLoopKeyframeIntervalFrames;
    if (shouldAddKeyframe) {
        state.m_lkLoopKeyframes.push_back(LkLoopKeyframe{frameId, rawTwc, corrected, descriptor});
        while (state.m_lkLoopKeyframes.size() > kLkLoopMaxKeyframes) {
            state.m_lkLoopKeyframes.pop_front();
        }
    }

    return corrected;
}



FeatureFrontend KltPerFrameModeStrategy::Frontend() const { return FeatureFrontend::LkGfttPerFrame; }

core::ports::SlamOutput KltPerFrameModeStrategy::Process(SlamEngineAdapter &engine,
                                                        const core::ports::SlamInputBatch &input,
                                                        bool extractFeatures, bool extractPointCloud)
{
    (void)extractFeatures;
    (void)extractPointCloud;
    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);

    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.mapId = 1;
    out.trackingState = ORB_SLAM3::Tracking::OK;
    out.poseValid = true;
    out.pose.valid = true;
    out.usedSuperPointFrontend = false;
    state.ResetExternalFeatureStats();

    const auto prepareStartTp = std::chrono::steady_clock::now();
    cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
    if (leftGray.empty() || rightGray.empty() || !state.m_lkCalibrationLoaded) {
        out.trackingState = ORB_SLAM3::Tracking::LOST;
        out.poseValid = false;
        out.pose.valid = false;
        out.inputPrepareMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - prepareStartTp).count();
        return out;
    }

    const auto rectifyStartTp = std::chrono::steady_clock::now();
    state.EnsureStereoRectifier(leftGray.size());
    const bool requestVpi = state.m_lkPerFrameAcceleration == "auto" || state.m_lkPerFrameAcceleration == "vpi" ||
                            state.m_lkPerFrameAcceleration == "vpi-cuda" ||
                            state.m_lkPerFrameAcceleration == "vpi_cuda" || state.m_lkPerFrameAcceleration == "gpu";
    const bool requestVpiRemap = requestVpi && EnvFlagEnabled("SMART_DRONE_VPI_REMAP", true);
    const bool requestVpiDisparity = requestVpi && EnvFlagEnabled("SMART_DRONE_VPI_DISPARITY", true);
    const bool requestVpiLk = requestVpi && EnvFlagEnabled("SMART_DRONE_VPI_LK", true);
    cv::Mat leftRect = leftGray;
    cv::Mat rightRect = rightGray;
    bool usedVpiRemap = false;
    if (requestVpiRemap && !state.m_lkMap1x.empty() && !state.m_lkMap2x.empty()) {
        usedVpiRemap = VpiRemapCurrentStereo(leftGray, rightGray, leftRect, rightRect, state.m_lkPerFrameVpi, state.m_lkMap1x,
                                             state.m_lkMap1y, state.m_lkMap2x, state.m_lkMap2y, state.m_lkPerFrameAccelLogged);
    }
    if (!usedVpiRemap && !state.m_lkMap1x.empty() && !state.m_lkMap2x.empty()) {
        cv::remap(leftGray, leftRect, state.m_lkMap1x, state.m_lkMap1y, cv::INTER_LINEAR);
        cv::remap(rightGray, rightRect, state.m_lkMap2x, state.m_lkMap2y, cv::INTER_LINEAR);
    }
    const auto rectifyEndTp = std::chrono::steady_clock::now();
    out.lkRectifyMs = std::chrono::duration<double, std::milli>(rectifyEndTp - rectifyStartTp).count();
    out.inputPrepareMs = std::chrono::duration<double, std::milli>(rectifyEndTp - prepareStartTp).count();

    if (!state.m_lkHavePrev) {
        state.m_lkPrevLeft = leftRect.clone();
        state.m_lkPrevRight = rightRect.clone();
#if SMART_DRONE_HAS_VPI
        if (usedVpiRemap) {
            StoreVpiPreviousRectified(state.m_lkPerFrameVpi);
        }
#endif
        state.m_lkTwc = Sophus::SE3f();
        state.m_lkPerFrameReferenceTwc = state.m_lkTwc;
        state.m_lkHavePrev = true;
        state.m_lkFrameCount = 1;
    } else {
        const auto disparityStartTp = std::chrono::steady_clock::now();
        cv::Mat disp;
        bool usedVpi = false;
#if SMART_DRONE_HAS_VPI
        if (usedVpiRemap && requestVpiDisparity && state.m_lkPerFrameVpi && state.m_lkPerFrameVpi->hasPrevRect) {
            usedVpi = ComputeVpiCudaDisparityImages(state.m_lkPrevLeft.size(), state.m_lkPerFrameVpi->prevLeftRect,
                                                    state.m_lkPerFrameVpi->prevRightRect, disp, state.m_lkPerFrameVpi);
        }
#endif
        if (!usedVpi && requestVpiDisparity) {
            usedVpi = ComputeVpiCudaDisparity(state.m_lkPrevLeft, state.m_lkPrevRight, disp, state.m_lkPerFrameVpi,
                                              state.m_lkPerFrameAccelLogged);
        }
        if (!usedVpi) {
            if (!state.m_lkPerFrameAccelLogged && state.m_lkPerFrameAcceleration == "cpu") {
                std::cerr << "[lk_per_frame_accel] backend=cpu_sgbm\n";
                state.m_lkPerFrameAccelLogged = true;
            }
            if (!state.m_lkPerFrameSgbm) {
                const int numDisparities = std::max(16, ((leftRect.cols / 8 + 15) / 16) * 16);
                state.m_lkPerFrameSgbm =
                    cv::StereoSGBM::create(0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 31, 8, 60, 2,
                                           cv::StereoSGBM::MODE_SGBM_3WAY);
            }
            cv::Mat disp16;
            state.m_lkPerFrameSgbm->compute(state.m_lkPrevLeft, state.m_lkPrevRight, disp16);
            disp16.convertTo(disp, CV_32F, 1.0 / 16.0);
        }
        const auto disparityEndTp = std::chrono::steady_clock::now();
        out.lkDisparityMs = std::chrono::duration<double, std::milli>(disparityEndTp - disparityStartTp).count();

        const auto gfttStartTp = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> pts0;
        std::vector<cv::Point2f> rawPts0;
        cv::goodFeaturesToTrack(state.m_lkPrevLeft, rawPts0, kLkGfttPerFrameMaxCorners, kLkGfttQualityLevel,
                                kLkGfttMinDistancePx,
                                cv::Mat(), kLkGfttBlockSize, false, kLkGfttHarrisK);
        pts0 = SelectGfttPointsGridBalanced(rawPts0, state.m_lkPrevLeft.size(), kLkGfttPerFrameMaxCorners,
                                            kLkGfttPerFrameMaxCornersPerCell);
        const auto gfttEndTp = std::chrono::steady_clock::now();
        out.lkGfttMs = std::chrono::duration<double, std::milli>(gfttEndTp - gfttStartTp).count();
        std::vector<cv::Point2f> pts1;
        std::vector<uint8_t> status;
        std::vector<float> err;
        bool usedVpiLk = false;
        const auto flowStartTp = std::chrono::steady_clock::now();
#if SMART_DRONE_HAS_VPI
        if (usedVpiRemap && requestVpiLk && !pts0.empty() && state.m_lkPerFrameVpi && state.m_lkPerFrameVpi->leftRect != nullptr) {
            VPIImage prevLeftImage = state.m_lkPerFrameVpi->hasPrevRect ? state.m_lkPerFrameVpi->prevLeftRect : nullptr;
            usedVpiLk = ComputeVpiCudaPyrLk(state.m_lkPrevLeft, prevLeftImage, state.m_lkPerFrameVpi->leftRect, pts0, pts1,
                                            status, state.m_lkPerFrameVpi);
        }
#endif
        if (!usedVpiLk && !pts0.empty()) {
            cv::calcOpticalFlowPyrLK(state.m_lkPrevLeft, leftRect, pts0, pts1, status, err, cv::Size(21, 21), 3);
        }
        std::vector<cv::Point2f> pts0Back;
        std::vector<uint8_t> statusBack;
        std::vector<float> errBack;
        const bool useForwardBackwardCheck = EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_FB_CHECK", false);
        const bool useDepthBalancedPnP = EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_DEPTH_BALANCE", true);
        const bool useKeyframeReference = EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_KEYFRAME", false);
        const int keyframeInterval =
            std::max(1, EnvIntValue("SMART_DRONE_LK_PER_FRAME_KEYFRAME_INTERVAL", 6));
        if (useForwardBackwardCheck && !pts1.empty()) {
            cv::calcOpticalFlowPyrLK(leftRect, state.m_lkPrevLeft, pts1, pts0Back, statusBack, errBack, cv::Size(21, 21),
                                    3);
        }
        const auto flowEndTp = std::chrono::steady_clock::now();
        out.lkFlowMs = std::chrono::duration<double, std::milli>(flowEndTp - flowStartTp).count();

        struct PerFramePnPCandidate {
            cv::Point3f object;
            cv::Point2f image;
            cv::Point2f prevPoint;
            float depth{0.0f};
            float flow{0.0f};
        };
        std::vector<PerFramePnPCandidate> candidates;
        candidates.reserve(pts0.size());
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        objectPoints.reserve(pts0.size());
        imagePoints.reserve(pts0.size());
        const auto candidateStartTp = std::chrono::steady_clock::now();
        for (size_t i = 0; i < pts0.size() && i < pts1.size(); ++i) {
            if (i >= status.size() || !status[i]) {
                continue;
            }
            const cv::Point2f &p0 = pts0[i];
            const cv::Point2f &p1 = pts1[i];
            if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= disp.cols - 1 || p0.y >= disp.rows - 1 ||
                p1.x < 1.0f || p1.y < 1.0f || p1.x >= leftRect.cols - 1 || p1.y >= leftRect.rows - 1) {
                continue;
            }
            if (cv::norm(p1 - p0) > kLkMaxFlowPx) {
                continue;
            }
            if (useForwardBackwardCheck && (i >= pts0Back.size() || i >= statusBack.size() || !statusBack[i] ||
                                            cv::norm(pts0Back[i] - p0) > kLkPerFrameForwardBackwardMaxErrPx)) {
                continue;
            }
            float d = 0.0f;
            if (!ReadConsistentDisparity(disp, p0, d)) {
                continue;
            }
            const float z = state.m_lkFx * state.m_lkBaseline / d;
            if (!(z >= kLkMinDepthMeters) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
                continue;
            }
            candidates.push_back({cv::Point3f((p0.x - state.m_lkCx) * z / state.m_lkFx, (p0.y - state.m_lkCy) * z / state.m_lkFy, z), p1,
                                  p0, z, static_cast<float>(cv::norm(p1 - p0))});
        }

        if (useDepthBalancedPnP) {
            std::array<int, kLkPerFramePnPSelectGridCols * kLkPerFramePnPSelectGridRows * kLkPerFramePnPDepthBins>
                bucketCounts{};
            for (const PerFramePnPCandidate &candidate : candidates) {
                const int gx = std::clamp(static_cast<int>(candidate.prevPoint.x * kLkPerFramePnPSelectGridCols /
                                                           std::max(1, state.m_lkPrevLeft.cols)),
                                          0, kLkPerFramePnPSelectGridCols - 1);
                const int gy = std::clamp(static_cast<int>(candidate.prevPoint.y * kLkPerFramePnPSelectGridRows /
                                                           std::max(1, state.m_lkPrevLeft.rows)),
                                          0, kLkPerFramePnPSelectGridRows - 1);
                const int dz = LkPerFrameDepthBin(candidate.depth);
                const int bucket = ((gy * kLkPerFramePnPSelectGridCols) + gx) * kLkPerFramePnPDepthBins + dz;
                if (bucketCounts[static_cast<size_t>(bucket)] >= kLkPerFramePnPMaxPerGridDepthBin) {
                    continue;
                }
                ++bucketCounts[static_cast<size_t>(bucket)];
                objectPoints.push_back(candidate.object);
                imagePoints.push_back(candidate.image);
            }
        } else {
            for (const PerFramePnPCandidate &candidate : candidates) {
                objectPoints.push_back(candidate.object);
                imagePoints.push_back(candidate.image);
            }
        }
        const auto candidateEndTp = std::chrono::steady_clock::now();
        out.lkCandidateMs = std::chrono::duration<double, std::milli>(candidateEndTp - candidateStartTp).count();

        int inlierCount = 0;
        bool poseUpdated = false;
        const auto pnpStartTp = std::chrono::steady_clock::now();
        if (objectPoints.size() >= kLkMinPnPPoints) {
            cv::Mat rvec, tvec, inliers;
            const cv::Mat K = MakeCameraMatrix(state.m_lkFx, state.m_lkFy, state.m_lkCx, state.m_lkCy);
            const int pnpIterations =
                std::max(20, EnvIntValue("SMART_DRONE_LK_PER_FRAME_PNP_ITERS", kLkPerFrameDefaultPnPIterations));
            const double pnpConfidence =
                std::clamp(static_cast<double>(EnvFloatValue("SMART_DRONE_LK_PER_FRAME_PNP_CONF",
                                                             static_cast<float>(kLkPerFrameDefaultPnPConfidence))),
                           0.5, 0.9999);
            const double defaultPnpReproj =
                requestVpi ? kLkPerFrameVpiPnPReprojThresholdPx : kLkPerFramePnPReprojThresholdPx;
            const char *pnpReprojOverride = std::getenv("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ");
            const double pnpReproj =
                pnpReprojOverride != nullptr && pnpReprojOverride[0] != '\0'
                    ? std::max(0.5, static_cast<double>(EnvFloatValue("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ",
                                                                      static_cast<float>(defaultPnpReproj))))
                    : defaultPnpReproj;
            bool ok = false;
            try {
                ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, tvec, false, pnpIterations,
                                        pnpReproj, pnpConfidence, inliers, LkPerFramePnPMethod());
                inlierCount = inliers.rows;
            } catch (const cv::Exception &e) {
                std::cerr << "[lk_per_frame_pnp] solvePnPRansac skipped points=" << objectPoints.size()
                          << " error=" << e.what() << "\n";
            }
            if (ok && inlierCount >= kLkMinPnPInliers) {
                std::vector<cv::Point3f> inlierObjectPoints;
                std::vector<cv::Point2f> inlierImagePoints;
                inlierObjectPoints.reserve(static_cast<size_t>(inlierCount));
                inlierImagePoints.reserve(static_cast<size_t>(inlierCount));
                for (int row = 0; row < inliers.rows; ++row) {
                    const int idx = inliers.at<int>(row, 0);
                    if (idx >= 0 && static_cast<size_t>(idx) < objectPoints.size()) {
                        inlierObjectPoints.push_back(objectPoints[static_cast<size_t>(idx)]);
                        inlierImagePoints.push_back(imagePoints[static_cast<size_t>(idx)]);
                    }
                }
                if (inlierObjectPoints.size() >= static_cast<size_t>(kLkMinPnPInliers)) {
                    try {
                        (void)cv::solvePnP(inlierObjectPoints, inlierImagePoints, K, cv::Mat(), rvec, tvec, true,
                                           cv::SOLVEPNP_ITERATIVE);
                    } catch (const cv::Exception &e) {
                        std::cerr << "[lk_per_frame_pnp] iterative refine skipped inliers="
                                  << inlierObjectPoints.size() << " error=" << e.what() << "\n";
                    }
                }
                cv::Mat Rcv;
                cv::Rodrigues(rvec, Rcv);
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f t = Eigen::Vector3f::Zero();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        R(r, c) = static_cast<float>(Rcv.at<double>(r, c));
                    }
                    t(r) = static_cast<float>(tvec.at<double>(r, 0));
                }
                if (std::isfinite(t.norm()) && t.norm() <= kLkMaxStepMeters) {
                    const Sophus::SE3f TcurrPrev(Sophus::SO3f(R), t);
                    const Sophus::SE3f delta = StabilizeLkCameraDelta(TcurrPrev.inverse());
                    if (useKeyframeReference) {
                        state.m_lkTwc = state.m_lkPerFrameReferenceTwc * delta;
                    } else {
                        state.m_lkTwc = state.m_lkTwc * delta;
                    }
                    poseUpdated = true;
                }
            }
        }
        const auto pnpEndTp = std::chrono::steady_clock::now();
        out.lkPnpMs = std::chrono::duration<double, std::milli>(pnpEndTp - pnpStartTp).count();
        out.frontendMs =
            out.lkDisparityMs + out.lkGfttMs + out.lkFlowMs + out.lkCandidateMs + out.lkPnpMs;

        out.matchesInliers = inlierCount;
        out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
        out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
        if (extractFeatures) {
            out.leftFeatures = std::move(pts1);
        }
        const bool refreshReference = !useKeyframeReference || (state.m_lkFrameCount % static_cast<uint32_t>(keyframeInterval)) == 0 ||
                                      inlierCount < std::max(kLkMinPnPInliers * 2, 24);
        const auto updateStartTp = std::chrono::steady_clock::now();
        if (refreshReference) {
            state.m_lkPrevLeft = leftRect.clone();
            state.m_lkPrevRight = rightRect.clone();
            state.m_lkPerFrameReferenceTwc = state.m_lkTwc;
#if SMART_DRONE_HAS_VPI
            if (usedVpiRemap) {
                StoreVpiPreviousRectified(state.m_lkPerFrameVpi);
            }
#endif
        }
        out.lkUpdateMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - updateStartTp)
                             .count();
        if (!poseUpdated) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            out.poseValid = false;
            out.pose.valid = false;
        }
        ++state.m_lkFrameCount;
    }

    const Eigen::Vector3f t = state.m_lkTwc.translation();
    const Eigen::Quaternionf q(state.m_lkTwc.so3().unit_quaternion());
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();
    return out;
}



std::unique_ptr<SlamModeStrategy> CreateKltModeStrategy()
{
    return std::make_unique<KltModeStrategy>();
}

std::unique_ptr<SlamModeStrategy> CreateKltPerFrameModeStrategy()
{
    return std::make_unique<KltPerFrameModeStrategy>();
}

} // namespace smartdrone::adapters::slam
