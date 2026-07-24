#include "adapters/slam/klt/klt_mode_utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/stereo/stereo_geometry.h"

namespace SmartDrone::Adapters::Slam {

namespace {

constexpr float LK_FORWARD_AXIS_GAIN = 1.0f;
constexpr float LK_MAX_FORWARD_STEP_METERS = 0.35f;
constexpr float LK_MAX_LATERAL_STEP_METERS = 0.35f;
constexpr float LK_MAX_VERTICAL_STEP_METERS = 0.35f;
constexpr int LK_RECOVERY_MIN_TRACKS = 24;
constexpr int LK_GRID_COLS = 8;
constexpr int LK_GRID_ROWS = 6;
constexpr int LK_GRID_CELL_COUNT = LK_GRID_COLS * LK_GRID_ROWS;
constexpr int LK_TARGET_TRACKS_PER_CELL = 12;
constexpr int LK_MIN_TRACKS_PER_CELL = 6;
constexpr size_t LK_MAX_TRACKS = 576;
constexpr int LK_GFTT_MAX_CORNERS = 900;
constexpr float LK_MIN_CONSISTENT_DISPARITY_PX = 1.0f;
constexpr float LK_DISPARITY_NEIGHBORHOOD_TOLERANCE_PX = 1.5f;
constexpr int LK_DISPARITY_NEIGHBORHOOD_RADIUS = 1;
constexpr float LK_MIN_SEED_DISTANCE_PX = 3.5f;

int LkGridCellForPoint(const cv::Point2f &pt, const cv::Size &size)
{
    if (size.width <= 0 || size.height <= 0 || pt.x < 0.0f || pt.y < 0.0f ||
        pt.x >= static_cast<float>(size.width) || pt.y >= static_cast<float>(size.height)) {
        return -1;
    }
    const int cellWidth = std::max(1, (size.width + LK_GRID_COLS - 1) / LK_GRID_COLS);
    const int cellHeight = std::max(1, (size.height + LK_GRID_ROWS - 1) / LK_GRID_ROWS);
    const int col = std::clamp(static_cast<int>(pt.x) / cellWidth, 0, LK_GRID_COLS - 1);
    const int row = std::clamp(static_cast<int>(pt.y) / cellHeight, 0, LK_GRID_ROWS - 1);
    return row * LK_GRID_COLS + col;
}

std::array<int, LK_GRID_CELL_COUNT> CountLkTracksByCell(const std::vector<LkStereoTrack> &tracks, const cv::Size &size)
{
    std::array<int, LK_GRID_CELL_COUNT> counts{};
    for (const LkStereoTrack &track : tracks) {
        const int cell = LkGridCellForPoint(track.left, size);
        if (cell >= 0) {
            ++counts[static_cast<size_t>(cell)];
        }
    }
    return counts;
}

bool LkTrackNearExisting(const cv::Point2f &left, const cv::Point2f &right, const std::vector<LkStereoTrack> &tracks)
{
    const float minDistSq = LK_MIN_SEED_DISTANCE_PX * LK_MIN_SEED_DISTANCE_PX;
    for (const LkStereoTrack &track : tracks) {
        const cv::Point2f leftDelta = track.left - left;
        const cv::Point2f rightDelta = track.right - right;
        if ((leftDelta.x * leftDelta.x + leftDelta.y * leftDelta.y) <= minDistSq &&
            (rightDelta.x * rightDelta.x + rightDelta.y * rightDelta.y) <= minDistSq) {
            return true;
        }
    }
    return false;
}

} // namespace

cv::Mat BuildLkLoopImageDescriptor(const cv::Mat &gray)
{
    if (gray.empty()) {
        return {};
    }
    cv::Mat small;
    cv::resize(gray, small, cv::Size(48, 32), 0.0, 0.0, cv::INTER_AREA);
    small.convertTo(small, CV_32F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(small, mean, stddev);
    const double sigma = std::max(1.0e-6, stddev[0]);
    small = (small - mean[0]) / sigma;
    return small.reshape(1, 1).clone();
}

double LkLoopDescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs)
{
    if (lhs.empty() || rhs.empty() || lhs.type() != CV_32F || rhs.type() != CV_32F ||
        lhs.total() != rhs.total()) {
        return -1.0;
    }
    return lhs.dot(rhs) / static_cast<double>(lhs.total());
}

std::vector<cv::Point2f> SelectGfttPointsGridBalanced(const std::vector<cv::Point2f> &points, const cv::Size &size,
                                                      int maxTotal, int maxPerCell)
{
    if (points.empty() || size.area() <= 0 || maxTotal <= 0 || maxPerCell <= 0) {
        return {};
    }

    std::array<int, LK_GRID_CELL_COUNT> counts{};
    std::vector<cv::Point2f> selected;
    selected.reserve(std::min(static_cast<size_t>(maxTotal), points.size()));
    for (const cv::Point2f &point : points) {
        if (static_cast<int>(selected.size()) >= maxTotal) {
            break;
        }
        const int cell = LkGridCellForPoint(point, size);
        if (cell < 0) {
            continue;
        }
        int &cellCount = counts[static_cast<size_t>(cell)];
        if (cellCount >= maxPerCell) {
            continue;
        }
        selected.push_back(point);
        ++cellCount;
    }
    return selected;
}

std::vector<LkStereoTrack> SelectLkTracksGridBalanced(const std::vector<LkStereoTrack> &tracks, const cv::Size &size)
{
    if (tracks.empty() || size.area() <= 0) {
        return tracks;
    }

    std::vector<LkStereoTrack> ranked = tracks;
    std::sort(ranked.begin(), ranked.end(), [](const LkStereoTrack &lhs, const LkStereoTrack &rhs) {
        if (lhs.quality != rhs.quality) {
            return lhs.quality > rhs.quality;
        }
        return lhs.age < rhs.age;
    });

    std::array<int, LK_GRID_CELL_COUNT> counts{};
    std::vector<LkStereoTrack> selected;
    selected.reserve(std::min(LK_MAX_TRACKS, ranked.size()));
    for (const LkStereoTrack &track : ranked) {
        if (selected.size() >= LK_MAX_TRACKS) {
            break;
        }
        const int cell = LkGridCellForPoint(track.left, size);
        if (cell < 0) {
            continue;
        }
        int &cellCount = counts[static_cast<size_t>(cell)];
        if (cellCount >= LK_TARGET_TRACKS_PER_CELL) {
            continue;
        }
        selected.push_back(track);
        ++cellCount;
    }
    return selected;
}

bool LkHasDegradedGridCell(const std::vector<LkStereoTrack> &tracks, const cv::Size &size)
{
    if (tracks.size() < static_cast<size_t>(LK_RECOVERY_MIN_TRACKS)) {
        return true;
    }
    const auto counts = CountLkTracksByCell(tracks, size);
    return std::any_of(counts.begin(), counts.end(), [](int count) { return count < LK_MIN_TRACKS_PER_CELL; });
}

bool IsHorizontalLateralFlow(const std::vector<cv::Point2f> &prevPts, const std::vector<cv::Point2f> &currPts,
                             const std::vector<uint8_t> &status, const cv::Size &size)
{
    const size_t countLimit = std::min(prevPts.size(), currPts.size());
    if (countLimit == 0 || size.width <= 0 || size.height <= 0) {
        return false;
    }

    const cv::Point2f center(0.5f * static_cast<float>(size.width), 0.5f * static_cast<float>(size.height));
    float sumDx = 0.0f;
    float sumDy = 0.0f;
    float sumRadial = 0.0f;
    int validCount = 0;
    for (size_t i = 0; i < countLimit; ++i) {
        if (i >= status.size() || !status[i]) {
            continue;
        }
        const cv::Point2f &p0 = prevPts[i];
        const cv::Point2f &p1 = currPts[i];
        if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= size.width - 1 || p0.y >= size.height - 1 ||
            p1.x < 1.0f || p1.y < 1.0f || p1.x >= size.width - 1 || p1.y >= size.height - 1) {
            continue;
        }
        const cv::Point2f flow = p1 - p0;
        if (cv::norm(flow) > LK_MAX_FLOW_PX) {
            continue;
        }
        const cv::Point2f radial = p0 - center;
        const float radius = std::sqrt(radial.x * radial.x + radial.y * radial.y);
        if (radius > 1.0f) {
            sumRadial += (flow.x * radial.x + flow.y * radial.y) / radius;
        }
        sumDx += flow.x;
        sumDy += flow.y;
        ++validCount;
    }

    if (validCount < 30) {
        return false;
    }
    const float invCount = 1.0f / static_cast<float>(validCount);
    const float meanDx = sumDx * invCount;
    const float meanDy = sumDy * invCount;
    const float meanRadial = sumRadial * invCount;
    const float absDx = std::abs(meanDx);
    return absDx >= 1.5f && absDx >= 1.8f * std::max(1.0f, std::abs(meanDy)) &&
           std::abs(meanRadial) <= 0.35f * absDx;
}

Sophus::SE3f StabilizeLkCameraDelta(const Sophus::SE3f &delta, bool horizontalLateralFlow)
{
    Eigen::Vector3f t = delta.translation();
    if (!std::isfinite(t.x()) || !std::isfinite(t.y()) || !std::isfinite(t.z())) {
        return Sophus::SE3f(delta.so3(), Eigen::Vector3f::Zero());
    }

    t.x() = std::clamp(t.x(), -LK_MAX_LATERAL_STEP_METERS, LK_MAX_LATERAL_STEP_METERS);
    t.y() = std::clamp(t.y(), -LK_MAX_VERTICAL_STEP_METERS, LK_MAX_VERTICAL_STEP_METERS);
    if (horizontalLateralFlow) {
        const float maxForwardFromLateral = std::max(0.015f, 0.25f * std::abs(t.x()));
        t.z() = std::clamp(t.z(), -maxForwardFromLateral, maxForwardFromLateral);
    }
    t.z() = std::clamp(t.z() * LK_FORWARD_AXIS_GAIN, -LK_MAX_FORWARD_STEP_METERS, LK_MAX_FORWARD_STEP_METERS);
    return Sophus::SE3f(delta.so3(), t);
}

bool ReadConsistentDisparity(const cv::Mat &disp, const cv::Point2f &pt, float &disparity)
{
    disparity = 0.0f;
    if (disp.empty()) {
        return false;
    }

    const int cx = static_cast<int>(std::lround(pt.x));
    const int cy = static_cast<int>(std::lround(pt.y));
    if (cx < LK_DISPARITY_NEIGHBORHOOD_RADIUS || cy < LK_DISPARITY_NEIGHBORHOOD_RADIUS ||
        cx >= disp.cols - LK_DISPARITY_NEIGHBORHOOD_RADIUS || cy >= disp.rows - LK_DISPARITY_NEIGHBORHOOD_RADIUS) {
        return false;
    }

    const float center = disp.at<float>(cy, cx);
    if (!(center >= LK_MIN_CONSISTENT_DISPARITY_PX) || center > STEREO_MAX_DISPARITY_PX || !std::isfinite(center)) {
        return false;
    }

    std::vector<float> neighborhood;
    neighborhood.reserve(9);
    for (int dy = -LK_DISPARITY_NEIGHBORHOOD_RADIUS; dy <= LK_DISPARITY_NEIGHBORHOOD_RADIUS; ++dy) {
        for (int dx = -LK_DISPARITY_NEIGHBORHOOD_RADIUS; dx <= LK_DISPARITY_NEIGHBORHOOD_RADIUS; ++dx) {
            const float value = disp.at<float>(cy + dy, cx + dx);
            if (value >= LK_MIN_CONSISTENT_DISPARITY_PX && value <= STEREO_MAX_DISPARITY_PX && std::isfinite(value)) {
                neighborhood.push_back(value);
            }
        }
    }
    if (neighborhood.size() < 5) {
        return false;
    }

    const size_t mid = neighborhood.size() / 2;
    std::nth_element(neighborhood.begin(), neighborhood.begin() + static_cast<std::ptrdiff_t>(mid),
                     neighborhood.end());
    const float median = neighborhood[mid];
    if (std::fabs(center - median) > LK_DISPARITY_NEIGHBORHOOD_TOLERANCE_PX) {
        return false;
    }
    disparity = median;
    return true;
}

int LkPerFrameDepthBin(float z)
{
    if (z < 1.5f) {
        return 0;
    }
    if (z < 3.0f) {
        return 1;
    }
    if (z < 6.0f) {
        return 2;
    }
    return 3;
}

int LkPerFramePnPMethod()
{
    const std::string method = EnvStringValue("SMART_DRONE_LK_PER_FRAME_PNP", "epnp");
    if (method == "p3p") {
        return cv::SOLVEPNP_P3P;
    }
    if (method == "ap3p") {
        return cv::SOLVEPNP_AP3P;
    }
    if (method == "iterative") {
        return cv::SOLVEPNP_ITERATIVE;
    }
    if (method == "sqpnp") {
        return cv::SOLVEPNP_SQPNP;
    }
    return cv::SOLVEPNP_EPNP;
}

cv::Mat BuildLkGfttDisparity(const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    cv::Mat disp32f;
    const int numDisparities =
        std::max(16, ((leftGray.cols / 8 + 15) / 16) * 16);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 31, 8, 60, 2,
        cv::StereoSGBM::MODE_SGBM_3WAY);
    cv::Mat disp16;
    sgbm->compute(leftGray, rightGray, disp16);
    disp16.convertTo(disp32f, CV_32F, 1.0 / 16.0);
    return disp32f;
}

std::vector<cv::Point2f> DetectLkGfttSeedPoints(const cv::Mat &leftGray)
{
    std::vector<cv::Point2f> leftPoints;
    cv::goodFeaturesToTrack(
        leftGray, leftPoints, LK_GFTT_MAX_CORNERS, LK_GFTT_QUALITY_LEVEL,
        LK_GFTT_MIN_DISTANCE_PX, cv::Mat(), LK_GFTT_BLOCK_SIZE, false,
        LK_GFTT_HARRIS_K);
    return leftPoints;
}

bool TryBuildLkGfttSeed(const cv::Mat &disp32f, const cv::Mat &left32f,
                        const cv::Mat &right32f, const cv::Point2f &leftPt,
                        LkStereoTrack &seed)
{
    const int ix = static_cast<int>(std::lround(leftPt.x));
    const int iy = static_cast<int>(std::lround(leftPt.y));
    if (ix < 0 || iy < 0 || ix >= disp32f.cols || iy >= disp32f.rows) {
        return false;
    }
    const float expectedDisparity = disp32f.at<float>(iy, ix);
    if (!(expectedDisparity >= StereoMinDisparityPx()) ||
        expectedDisparity > STEREO_MAX_DISPARITY_PX ||
        !std::isfinite(expectedDisparity)) {
        return false;
    }
    cv::Point2f rightPt;
    float zncc = -1.0f;
    if (!FindRightPointByStereoZnccAroundDisparity(
            {left32f, leftPt, right32f, expectedDisparity, rightPt, zncc})) {
        return false;
    }
    seed = LkStereoTrack{
        leftPt, rightPt, std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f), 0};
    return true;
}

std::vector<LkStereoTrack> BuildLkGfttStereoSeeds(const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    std::vector<LkStereoTrack> seeds;
    if (leftGray.empty() || rightGray.empty()) {
        return seeds;
    }

    const cv::Mat disp32f = BuildLkGfttDisparity(leftGray, rightGray);
    const std::vector<cv::Point2f> leftPoints = DetectLkGfttSeedPoints(leftGray);
    cv::Mat left32f;
    cv::Mat right32f;
    leftGray.convertTo(left32f, CV_32F);
    rightGray.convertTo(right32f, CV_32F);
    seeds.reserve(leftPoints.size());
    for (const cv::Point2f &leftPt : leftPoints) {
        LkStereoTrack seed;
        if (TryBuildLkGfttSeed(disp32f, left32f, right32f, leftPt, seed)) {
            seeds.push_back(seed);
        }
    }
    return seeds;
}

void AppendLkSeedsForDegradedCells(const std::vector<LkStereoTrack> &seeds, const cv::Size &size,
                                   std::vector<LkStereoTrack> &tracks)
{
    if (seeds.empty() || size.area() <= 0) {
        return;
    }
    auto counts = CountLkTracksByCell(tracks, size);
    for (const LkStereoTrack &seed : seeds) {
        if (tracks.size() >= LK_MAX_TRACKS) {
            return;
        }
        const int cell = LkGridCellForPoint(seed.left, size);
        if (cell < 0) {
            continue;
        }
        int &cellCount = counts[static_cast<size_t>(cell)];
        if (cellCount >= LK_TARGET_TRACKS_PER_CELL) {
            continue;
        }
        if (LkTrackNearExisting(seed.left, seed.right, tracks)) {
            continue;
        }
        tracks.push_back(seed);
        ++cellCount;
    }
}

} // namespace SmartDrone::Adapters::Slam
