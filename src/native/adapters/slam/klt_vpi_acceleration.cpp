#include "adapters/slam/klt_vpi_acceleration.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Pyramid.h>
#include <vpi/Stream.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/OpticalFlowPyrLK.h>
#include <vpi/algo/Remap.h>
#include <vpi/algo/StereoDisparity.h>

#include "adapters/slam/klt_mode_utils.h"
#include "adapters/slam/slam_env.h"

namespace SmartDrone::Adapters::Slam {

struct LkPerFrameVpiState {
    ~LkPerFrameVpiState()
    {
        if (prevPts != nullptr) {
            vpiArrayDestroy(prevPts);
        }
        if (curPts != nullptr) {
            vpiArrayDestroy(curPts);
        }
        if (trackStatus != nullptr) {
            vpiArrayDestroy(trackStatus);
        }
        if (prevPyr != nullptr) {
            vpiPyramidDestroy(prevPyr);
        }
        if (curPyr != nullptr) {
            vpiPyramidDestroy(curPyr);
        }
        if (lkPayload != nullptr) {
            vpiPayloadDestroy(lkPayload);
        }
        if (leftRect != nullptr) {
            vpiImageDestroy(leftRect);
        }
        if (rightRect != nullptr) {
            vpiImageDestroy(rightRect);
        }
        if (prevLeftRect != nullptr) {
            vpiImageDestroy(prevLeftRect);
        }
        if (prevRightRect != nullptr) {
            vpiImageDestroy(prevRightRect);
        }
        if (leftRemapPayload != nullptr) {
            vpiPayloadDestroy(leftRemapPayload);
        }
        if (rightRemapPayload != nullptr) {
            vpiPayloadDestroy(rightRemapPayload);
        }
        if (leftWrapper != nullptr) {
            vpiImageDestroy(leftWrapper);
        }
        if (rightWrapper != nullptr) {
            vpiImageDestroy(rightWrapper);
        }
        if (disparity != nullptr) {
            vpiImageDestroy(disparity);
        }
        if (stereoPayload != nullptr) {
            vpiPayloadDestroy(stereoPayload);
        }
        if (stream != nullptr) {
            vpiStreamDestroy(stream);
        }
        vpiWarpMapFreeData(&leftWarp);
        vpiWarpMapFreeData(&rightWarp);
    }

    int width{0};
    int height{0};
    int maxDisparity{0};
    VPIStream stream{nullptr};
    VPIPayload stereoPayload{nullptr};
    VPIPayload leftRemapPayload{nullptr};
    VPIPayload rightRemapPayload{nullptr};
    VPIPayload lkPayload{nullptr};
    VPIImage leftWrapper{nullptr};
    VPIImage rightWrapper{nullptr};
    VPIImage leftRect{nullptr};
    VPIImage rightRect{nullptr};
    VPIImage prevLeftRect{nullptr};
    VPIImage prevRightRect{nullptr};
    VPIImage disparity{nullptr};
    VPIPyramid prevPyr{nullptr};
    VPIPyramid curPyr{nullptr};
    VPIArray prevPts{nullptr};
    VPIArray curPts{nullptr};
    VPIArray trackStatus{nullptr};
    VPIWarpMap leftWarp{};
    VPIWarpMap rightWarp{};
    bool hasPrevRect{false};
};

namespace {

constexpr int kVpiStereoConfidenceThreshold = 32767;
constexpr int kVpiStereoP1 = 20;
constexpr int kVpiStereoP2 = 176;
constexpr float kVpiStereoUniqueness = 0.38f;
constexpr int kVpiStereoIncludeDiagonals = 1;

const char *VpiStatusName(VPIStatus status)
{
    const char *name = vpiStatusGetName(status);
    return name != nullptr ? name : "VPI_ERROR_UNKNOWN";
}

bool FillVpiWarpMapFromOpenCvMaps(const cv::Mat &mapX, const cv::Mat &mapY, VPIWarpMap &warp)
{
    if (mapX.empty() || mapY.empty() || mapX.size() != mapY.size() || mapX.type() != CV_32FC1 ||
        mapY.type() != CV_32FC1) {
        return false;
    }
    vpiWarpMapFreeData(&warp);
    warp = {};
    warp.grid.numHorizRegions = 1;
    warp.grid.numVertRegions = 1;
    warp.grid.regionWidth[0] = static_cast<int16_t>(mapX.cols);
    warp.grid.regionHeight[0] = static_cast<int16_t>(mapX.rows);
    warp.grid.horizInterval[0] = 1;
    warp.grid.vertInterval[0] = 1;
    VPIStatus status = vpiWarpMapAllocData(&warp);
    if (status != VPI_SUCCESS || warp.keypoints == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI warp map allocation failed: " << VpiStatusName(status) << "\n";
        return false;
    }
    for (int y = 0; y < mapX.rows; ++y) {
        auto *row = reinterpret_cast<VPIKeypointF32 *>(reinterpret_cast<uint8_t *>(warp.keypoints) +
                                                       static_cast<size_t>(y) * warp.pitchBytes);
        for (int x = 0; x < mapX.cols; ++x) {
            row[x].x = mapX.at<float>(y, x);
            row[x].y = mapY.at<float>(y, x);
        }
    }
    return true;
}

cv::Mat DownloadVpiU8Image(VPIImage image)
{
    VPIImageData data{};
    VPIStatus status = vpiImageLockData(image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data);
    if (status != VPI_SUCCESS || data.bufferType != VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR ||
        data.buffer.pitch.numPlanes < 1 || data.buffer.pitch.planes[0].data == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI image lock failed: " << VpiStatusName(status) << "\n";
        return {};
    }
    const auto &plane = data.buffer.pitch.planes[0];
    cv::Mat view(plane.height, plane.width, CV_8UC1, plane.data, static_cast<size_t>(plane.pitchBytes));
    cv::Mat out = view.clone();
    vpiImageUnlock(image);
    return out;
}

bool EnsureVpiPerFrameState(std::shared_ptr<LkPerFrameVpiState> &state, const cv::Size &size,
                            const cv::Mat &map1x, const cv::Mat &map1y, const cv::Mat &map2x, const cv::Mat &map2y,
                            int maxDisparity, bool &logged)
{
    const bool recreate = !state || state->width != size.width || state->height != size.height ||
                          state->maxDisparity != maxDisparity;
    if (!recreate) {
        return true;
    }

    state = std::make_shared<LkPerFrameVpiState>();
    state->width = size.width;
    state->height = size.height;
    state->maxDisparity = maxDisparity;

    VPIStatus status = vpiStreamCreate(VPI_BACKEND_CUDA, &state->stream);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI CUDA stream create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    if (!FillVpiWarpMapFromOpenCvMaps(map1x, map1y, state->leftWarp) ||
        !FillVpiWarpMapFromOpenCvMaps(map2x, map2y, state->rightWarp)) {
        std::cerr << "[lk_per_frame_accel] VPI remap warp map build failed; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiCreateRemap(VPI_BACKEND_CUDA, &state->leftWarp, &state->leftRemapPayload);
    if (status == VPI_SUCCESS) {
        status = vpiCreateRemap(VPI_BACKEND_CUDA, &state->rightWarp, &state->rightRemapPayload);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI remap payload create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    const uint64_t imageBackends = VPI_BACKEND_CUDA | VPI_BACKEND_CPU;
    status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->leftRect);
    if (status == VPI_SUCCESS) {
        status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->rightRect);
    }
    if (status == VPI_SUCCESS) {
        status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->prevLeftRect);
    }
    if (status == VPI_SUCCESS) {
        status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->prevRightRect);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI rectified image create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    VPIStereoDisparityEstimatorCreationParams createParams{};
    status = vpiInitStereoDisparityEstimatorCreationParams(&createParams);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo creation params init failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }
    createParams.maxDisparity = maxDisparity;
    createParams.downscaleFactor = 1;
    createParams.includeDiagonals = EnvIntValue("SMART_DRONE_VPI_STEREO_DIAG", kVpiStereoIncludeDiagonals);
    status = vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, size.width, size.height, VPI_IMAGE_FORMAT_U8,
                                               &createParams, &state->stereoPayload);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI CUDA stereo payload create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_S16, VPI_BACKEND_CUDA | VPI_BACKEND_CPU,
                            &state->disparity);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI disparity image create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiPyramidCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, 4, 0.5f,
                              VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->prevPyr);
    if (status == VPI_SUCCESS) {
        status = vpiPyramidCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, 4, 0.5f,
                                  VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->curPyr);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI pyramid create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_lk\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiCreateOpticalFlowPyrLK(VPI_BACKEND_CUDA, size.width, size.height, VPI_IMAGE_FORMAT_U8, 4, 0.5f,
                                       &state->lkPayload);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI PyrLK payload create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_lk\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiArrayCreate(kLkGfttPerFrameMaxCorners, VPI_ARRAY_TYPE_KEYPOINT_F32,
                            VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->prevPts);
    if (status == VPI_SUCCESS) {
        status = vpiArrayCreate(kLkGfttPerFrameMaxCorners, VPI_ARRAY_TYPE_KEYPOINT_F32,
                                VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->curPts);
    }
    if (status == VPI_SUCCESS) {
        status = vpiArrayCreate(kLkGfttPerFrameMaxCorners, VPI_ARRAY_TYPE_U8, VPI_BACKEND_CUDA | VPI_BACKEND_CPU,
                                &state->trackStatus);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI LK array create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_lk\n";
        state.reset();
        logged = true;
        return false;
    }

    std::cerr << "[lk_per_frame_accel] backend=vpi_cuda stages=remap,stereo_disparity,pyr_lk"
              << " pyr_lk=available size=" << size.width
              << "x" << size.height << " max_disparity=" << maxDisparity
              << " conf=" << EnvIntValue("SMART_DRONE_VPI_STEREO_CONF", kVpiStereoConfidenceThreshold)
              << " p1=" << EnvIntValue("SMART_DRONE_VPI_STEREO_P1", kVpiStereoP1)
              << " p2=" << EnvIntValue("SMART_DRONE_VPI_STEREO_P2", kVpiStereoP2)
              << " uniqueness=" << EnvFloatValue("SMART_DRONE_VPI_STEREO_UNIQUENESS", kVpiStereoUniqueness)
              << " diag=" << EnvIntValue("SMART_DRONE_VPI_STEREO_DIAG", kVpiStereoIncludeDiagonals) << "\n";
    logged = true;
    return true;
}

void ConfigureVpiStereoParams(VPIStereoDisparityEstimatorParams &params, int maxDisparity)
{
    params.maxDisparity = maxDisparity;
    params.confidenceThreshold = EnvIntValue("SMART_DRONE_VPI_STEREO_CONF", kVpiStereoConfidenceThreshold);
    params.p1 = EnvIntValue("SMART_DRONE_VPI_STEREO_P1", kVpiStereoP1);
    params.p2 = EnvIntValue("SMART_DRONE_VPI_STEREO_P2", kVpiStereoP2);
    params.uniqueness = EnvFloatValue("SMART_DRONE_VPI_STEREO_UNIQUENESS", kVpiStereoUniqueness);
}

bool DownloadVpiDisparity(const cv::Size &size, VPIImage disparityImage, cv::Mat &disp)
{
    VPIImageData data{};
    VPIStatus status = vpiImageLockData(disparityImage, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data);
    if (status != VPI_SUCCESS || data.bufferType != VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR ||
        data.buffer.pitch.numPlanes < 1 || data.buffer.pitch.planes[0].data == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI disparity lock failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }

    const auto &plane = data.buffer.pitch.planes[0];
    cv::Mat disp16(size.height, size.width, CV_16S, plane.data, static_cast<size_t>(plane.pitchBytes));
    disp16.convertTo(disp, CV_32F, 1.0 / 32.0);
    vpiImageUnlock(disparityImage);
    return true;
}

bool ComputeVpiCudaDisparityImages(const cv::Size &size, VPIImage leftImage, VPIImage rightImage, cv::Mat &disp,
                                   std::shared_ptr<LkPerFrameVpiState> &state)
{
    if (!state || state->stream == nullptr || state->stereoPayload == nullptr || state->disparity == nullptr ||
        leftImage == nullptr || rightImage == nullptr) {
        return false;
    }

    VPIStereoDisparityEstimatorParams params{};
    VPIStatus status = vpiInitStereoDisparityEstimatorParams(&params);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo params init failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }
    ConfigureVpiStereoParams(params, state->maxDisparity);

    status = vpiSubmitStereoDisparityEstimator(state->stream, VPI_BACKEND_CUDA, state->stereoPayload, leftImage,
                                               rightImage, state->disparity, nullptr, &params);
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo submit failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }
    return DownloadVpiDisparity(size, state->disparity, disp);
}

bool ComputeVpiCudaPyrLk(const cv::Mat &prevLeft, VPIImage prevLeftImage, VPIImage curLeftImage,
                         const std::vector<cv::Point2f> &pts0, std::vector<cv::Point2f> &pts1,
                         std::vector<uint8_t> &statusOut, std::shared_ptr<LkPerFrameVpiState> &state)
{
    pts1.clear();
    statusOut.clear();
    if (!state || state->stream == nullptr || state->lkPayload == nullptr || state->prevPyr == nullptr ||
        state->curPyr == nullptr || state->prevPts == nullptr || state->curPts == nullptr ||
        state->trackStatus == nullptr || curLeftImage == nullptr || pts0.empty()) {
        return false;
    }
    if (prevLeftImage == nullptr && prevLeft.empty()) {
        return false;
    }

    const int count = std::min<int>(static_cast<int>(pts0.size()), kLkGfttPerFrameMaxCorners);
    VPIArrayData prevData{};
    VPIArrayData curInitData{};
    VPIStatus vstatus =
        vpiArrayLockData(state->prevPts, VPI_LOCK_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &prevData);
    if (vstatus != VPI_SUCCESS || prevData.bufferType != VPI_ARRAY_BUFFER_HOST_AOS ||
        prevData.buffer.aos.data == nullptr || prevData.buffer.aos.sizePointer == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI prev point upload lock failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        return false;
    }
    vstatus = vpiArrayLockData(state->curPts, VPI_LOCK_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &curInitData);
    if (vstatus != VPI_SUCCESS || curInitData.bufferType != VPI_ARRAY_BUFFER_HOST_AOS ||
        curInitData.buffer.aos.data == nullptr || curInitData.buffer.aos.sizePointer == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI current point init lock failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        vpiArrayUnlock(state->prevPts);
        return false;
    }
    auto *prevKeypoints = static_cast<VPIKeypointF32 *>(prevData.buffer.aos.data);
    auto *curInitKeypoints = static_cast<VPIKeypointF32 *>(curInitData.buffer.aos.data);
    for (int i = 0; i < count; ++i) {
        prevKeypoints[i].x = pts0[static_cast<size_t>(i)].x;
        prevKeypoints[i].y = pts0[static_cast<size_t>(i)].y;
        curInitKeypoints[i].x = pts0[static_cast<size_t>(i)].x;
        curInitKeypoints[i].y = pts0[static_cast<size_t>(i)].y;
    }
    *prevData.buffer.aos.sizePointer = count;
    *curInitData.buffer.aos.sizePointer = count;
    vpiArrayUnlock(state->curPts);
    vpiArrayUnlock(state->prevPts);
    vpiArraySetSize(state->trackStatus, count);

    VPIImage prevWrapper = nullptr;
    VPIImage prevImage = prevLeftImage;
    if (prevImage == nullptr) {
        vstatus = vpiImageCreateWrapperOpenCVMat(prevLeft, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &prevWrapper);
        prevImage = prevWrapper;
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiSubmitGaussianPyramidGenerator(state->stream, VPI_BACKEND_CUDA, prevImage, state->prevPyr,
                                                    VPI_BORDER_CLAMP);
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiSubmitGaussianPyramidGenerator(state->stream, VPI_BACKEND_CUDA, curLeftImage, state->curPyr,
                                                    VPI_BORDER_CLAMP);
    }
    VPIOpticalFlowPyrLKParams lkParams{};
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiInitOpticalFlowPyrLKParams(&lkParams);
        lkParams.windowDimension = 21;
        lkParams.numIterations = 24;
        lkParams.useInitialFlow = 1;
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiSubmitOpticalFlowPyrLK(state->stream, VPI_BACKEND_CUDA, state->lkPayload, state->prevPyr,
                                            state->curPyr, state->prevPts, state->curPts, state->trackStatus,
                                            &lkParams);
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiStreamSync(state->stream);
    }
    if (prevWrapper != nullptr) {
        vpiImageDestroy(prevWrapper);
    }
    if (vstatus != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI PyrLK submit failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        return false;
    }

    VPIArrayData curData{};
    VPIArrayData statusData{};
    vstatus = vpiArrayLockData(state->curPts, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &curData);
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiArrayLockData(state->trackStatus, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &statusData);
    }
    if (vstatus != VPI_SUCCESS || curData.buffer.aos.data == nullptr || statusData.buffer.aos.data == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI LK result lock failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        if (curData.buffer.aos.data != nullptr) {
            vpiArrayUnlock(state->curPts);
        }
        return false;
    }

    auto *curKeypoints = static_cast<VPIKeypointF32 *>(curData.buffer.aos.data);
    auto *trackStatus = static_cast<uint8_t *>(statusData.buffer.aos.data);
    pts1.resize(static_cast<size_t>(count));
    statusOut.resize(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        pts1[static_cast<size_t>(i)] = cv::Point2f(curKeypoints[i].x, curKeypoints[i].y);
        statusOut[static_cast<size_t>(i)] = trackStatus[i] == 0 ? 1 : 0;
    }
    vpiArrayUnlock(state->trackStatus);
    vpiArrayUnlock(state->curPts);
    return true;
}

} // namespace

bool StoreVpiPreviousRectified(std::shared_ptr<LkPerFrameVpiState> &state)
{
    if (!state || state->stream == nullptr || state->leftRect == nullptr || state->rightRect == nullptr ||
        state->prevLeftRect == nullptr || state->prevRightRect == nullptr) {
        return false;
    }
    VPIStatus status = vpiSubmitConvertImageFormat(state->stream, VPI_BACKEND_CUDA, state->leftRect,
                                                   state->prevLeftRect, nullptr);
    if (status == VPI_SUCCESS) {
        status = vpiSubmitConvertImageFormat(state->stream, VPI_BACKEND_CUDA, state->rightRect, state->prevRightRect,
                                             nullptr);
    }
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI previous rect copy failed: " << VpiStatusName(status)
                  << "; fallback=cpu_cache\n";
        state->hasPrevRect = false;
        return false;
    }
    state->hasPrevRect = true;
    return true;
}

bool VpiRemapCurrentStereo(const cv::Mat &leftRaw, const cv::Mat &rightRaw, cv::Mat &leftRect, cv::Mat &rightRect,
                           std::shared_ptr<LkPerFrameVpiState> &state, const cv::Mat &map1x, const cv::Mat &map1y,
                           const cv::Mat &map2x, const cv::Mat &map2y, bool &logged)
{
    if (leftRaw.empty() || rightRaw.empty() || leftRaw.size() != rightRaw.size() || leftRaw.type() != CV_8UC1 ||
        rightRaw.type() != CV_8UC1 || map1x.empty() || map2x.empty()) {
        return false;
    }
    const int maxDisparity = std::clamp(((leftRaw.cols / 8 + 15) / 16) * 16, 16, 256);
    if (!EnsureVpiPerFrameState(state, leftRaw.size(), map1x, map1y, map2x, map2y, maxDisparity, logged)) {
        return false;
    }

    VPIImage leftWrapper = nullptr;
    VPIImage rightWrapper = nullptr;
    VPIStatus status = vpiImageCreateWrapperOpenCVMat(leftRaw, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &leftWrapper);
    if (status == VPI_SUCCESS) {
        status = vpiImageCreateWrapperOpenCVMat(rightRaw, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &rightWrapper);
    }
    if (status == VPI_SUCCESS) {
        status = vpiSubmitRemap(state->stream, VPI_BACKEND_CUDA, state->leftRemapPayload, leftWrapper, state->leftRect,
                                VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
    }
    if (status == VPI_SUCCESS) {
        status = vpiSubmitRemap(state->stream, VPI_BACKEND_CUDA, state->rightRemapPayload, rightWrapper,
                                state->rightRect, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
    }
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (leftWrapper != nullptr) {
        vpiImageDestroy(leftWrapper);
    }
    if (rightWrapper != nullptr) {
        vpiImageDestroy(rightWrapper);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI remap submit failed: " << VpiStatusName(status)
                  << "; fallback=cpu_remap\n";
        state.reset();
        return false;
    }
    leftRect = DownloadVpiU8Image(state->leftRect);
    rightRect = DownloadVpiU8Image(state->rightRect);
    return !leftRect.empty() && !rightRect.empty();
}

bool ComputeVpiCudaDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disp,
                             std::shared_ptr<LkPerFrameVpiState> &state, bool &logged)
{
    if (left.empty() || right.empty() || left.size() != right.size() || left.type() != CV_8UC1 || right.type() != CV_8UC1) {
        return false;
    }

    const int maxDisparity = std::clamp(((left.cols / 8 + 15) / 16) * 16, 16, 256);
    const bool recreate = !state || state->width != left.cols || state->height != left.rows ||
                          state->maxDisparity != maxDisparity;
    if (recreate) {
        state = std::make_shared<LkPerFrameVpiState>();
        state->width = left.cols;
        state->height = left.rows;
        state->maxDisparity = maxDisparity;

        VPIStatus status = vpiStreamCreate(VPI_BACKEND_CUDA, &state->stream);
        if (status != VPI_SUCCESS) {
            if (!logged) {
                std::cerr << "[lk_per_frame_accel] VPI CUDA stream create failed: " << VpiStatusName(status)
                          << "; fallback=cpu_sgbm\n";
                logged = true;
            }
            state.reset();
            return false;
        }

        VPIStereoDisparityEstimatorCreationParams createParams{};
        status = vpiInitStereoDisparityEstimatorCreationParams(&createParams);
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI stereo creation params init failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }
        createParams.maxDisparity = maxDisparity;
        createParams.downscaleFactor = 1;
        createParams.includeDiagonals = EnvIntValue("SMART_DRONE_VPI_STEREO_DIAG", kVpiStereoIncludeDiagonals);

        status = vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, left.cols, left.rows, VPI_IMAGE_FORMAT_U8,
                                                   &createParams, &state->stereoPayload);
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI CUDA stereo payload create failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }

        status = vpiImageCreate(left.cols, left.rows, VPI_IMAGE_FORMAT_S16, VPI_BACKEND_CUDA | VPI_BACKEND_CPU,
                                &state->disparity);
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI disparity image create failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }

        status = vpiImageCreateWrapperOpenCVMat(left, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->leftWrapper);
        if (status == VPI_SUCCESS) {
            status = vpiImageCreateWrapperOpenCVMat(right, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->rightWrapper);
        }
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI OpenCV wrapper create failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }

        std::cerr << "[lk_per_frame_accel] backend=vpi_cuda stage=stereo_disparity size=" << left.cols << "x"
                  << left.rows << " max_disparity=" << maxDisparity << "\n";
        logged = true;
    } else {
        VPIStatus status = VPI_SUCCESS;
        if (state->leftWrapper == nullptr) {
            status = vpiImageCreateWrapperOpenCVMat(left, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->leftWrapper);
        } else {
            status = vpiImageSetWrappedOpenCVMat(state->leftWrapper, left);
        }
        if (status == VPI_SUCCESS) {
            if (state->rightWrapper == nullptr) {
                status =
                    vpiImageCreateWrapperOpenCVMat(right, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->rightWrapper);
            } else {
                status = vpiImageSetWrappedOpenCVMat(state->rightWrapper, right);
            }
        }
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI wrapper update failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            return false;
        }
    }

    VPIStereoDisparityEstimatorParams params{};
    VPIStatus status = vpiInitStereoDisparityEstimatorParams(&params);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo params init failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }
    ConfigureVpiStereoParams(params, maxDisparity);

    status = vpiSubmitStereoDisparityEstimator(state->stream, VPI_BACKEND_CUDA, state->stereoPayload,
                                               state->leftWrapper, state->rightWrapper, state->disparity, nullptr,
                                               &params);
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo submit failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }

    return DownloadVpiDisparity(left.size(), state->disparity, disp);
}

bool HasVpiPreviousRectified(const std::shared_ptr<LkPerFrameVpiState> &state)
{
    return state != nullptr && state->hasPrevRect;
}

bool ComputeVpiCudaPreviousRectifiedDisparity(const cv::Size &size, cv::Mat &disp,
                                              std::shared_ptr<LkPerFrameVpiState> &state)
{
    if (!HasVpiPreviousRectified(state)) {
        return false;
    }
    return ComputeVpiCudaDisparityImages(size, state->prevLeftRect, state->prevRightRect, disp, state);
}

bool ComputeVpiCudaCurrentPyrLk(const cv::Mat &prevLeft, const std::vector<cv::Point2f> &pts0,
                                std::vector<cv::Point2f> &pts1, std::vector<uint8_t> &statusOut,
                                std::shared_ptr<LkPerFrameVpiState> &state)
{
    if (!state || state->leftRect == nullptr) {
        return false;
    }
    VPIImage prevLeftImage = state->hasPrevRect ? state->prevLeftRect : nullptr;
    return ComputeVpiCudaPyrLk(prevLeft, prevLeftImage, state->leftRect, pts0, pts1, statusOut, state);
}

} // namespace SmartDrone::Adapters::Slam
