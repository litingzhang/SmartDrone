#include "adapters/slam/klt/klt_pose_estimator.h"
#include "adapters/slam/klt/klt_mode_utils.h"

#include <cstdlib>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

namespace {

using SmartDrone::Adapters::Slam::EstimateKltPnpPoseDelta;
using SmartDrone::Adapters::Slam::KltPnpCameraIntrinsics;
using SmartDrone::Adapters::Slam::KltPnpPoseEstimatorOptions;
using SmartDrone::Adapters::Slam::LkPerFramePnPMethod;
using SmartDrone::Adapters::Slam::MakeKltPerFramePnpPoseEstimatorOptions;
using SmartDrone::Core::Ports::IVisualPnpPoseBackend;
using SmartDrone::Core::Ports::VisualPnpPoseBackendOptions;
using SmartDrone::Core::Ports::VisualPnpPoseBackendResult;

class ScopedEnvironment final {
public:
    ScopedEnvironment(const char *name, const char *value) : m_name(name)
    {
        const char *previous = std::getenv(name);
        if (previous != nullptr) {
            m_previous = previous;
        }
        setenv(name, value, 1);
    }

    ~ScopedEnvironment()
    {
        if (m_previous.has_value()) {
            setenv(m_name.c_str(), m_previous->c_str(), 1);
        } else {
            unsetenv(m_name.c_str());
        }
    }

private:
    std::string m_name;
    std::optional<std::string> m_previous;
};

Sophus::SE3f MakeSignedPose(const Eigen::Vector3f &translation,
                            const Eigen::Vector3f &rotationRadians)
{
    const Eigen::Quaternionf rotation =
        Eigen::AngleAxisf(rotationRadians.z(), Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(rotationRadians.y(), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(rotationRadians.x(), Eigen::Vector3f::UnitX());
    return Sophus::SE3f(Sophus::SO3f(rotation.normalized()), translation);
}

std::vector<cv::Point3f> MakeNonCoplanarPreviousPoints()
{
    return {
        {-0.90f, -0.60f, 3.40f}, {-0.35f, -0.55f, 4.10f},
        {0.30f, -0.65f, 4.80f},  {0.85f, -0.45f, 5.60f},
        {-0.75f, -0.10f, 5.10f}, {-0.20f, -0.05f, 3.70f},
        {0.40f, -0.15f, 6.20f},  {0.95f, 0.00f, 4.40f},
        {-0.85f, 0.35f, 4.60f},  {-0.30f, 0.45f, 5.80f},
        {0.25f, 0.30f, 3.90f},   {0.80f, 0.55f, 5.30f},
        {-0.55f, 0.70f, 6.50f},  {0.05f, 0.65f, 4.30f},
        {0.55f, 0.75f, 5.00f},   {0.10f, -0.35f, 6.80f},
    };
}

std::vector<cv::Point2f> ProjectIntoCurrentCamera(
    const std::vector<cv::Point3f> &previousPoints,
    const Sophus::SE3f &currentFromPrevious,
    const KltPnpCameraIntrinsics &camera)
{
    std::vector<cv::Point2f> imagePoints;
    imagePoints.reserve(previousPoints.size());
    for (const cv::Point3f &point : previousPoints) {
        const Eigen::Vector3f currentPoint =
            currentFromPrevious * Eigen::Vector3f(point.x, point.y, point.z);
        imagePoints.emplace_back(
            camera.fx * currentPoint.x() / currentPoint.z() + camera.cx,
            camera.fy * currentPoint.y() / currentPoint.z() + camera.cy);
    }
    return imagePoints;
}

KltPnpPoseEstimatorOptions MakeEstimatorOptions(
    const KltPnpCameraIntrinsics &camera)
{
    KltPnpPoseEstimatorOptions options;
    options.camera = camera;
    options.minPoints = 8;
    options.minInliers = 8;
    options.iterations = 120;
    options.reprojectionError = 0.25;
    options.confidence = 0.999;
    options.method = cv::SOLVEPNP_ITERATIVE;
    options.refineWithInliers = true;
    return options;
}

void ExpectPoseNear(const Sophus::SE3f &actual,
                    const Sophus::SE3f &expected, float tolerance)
{
    const float rotationError =
        (expected.so3().inverse() * actual.so3()).log().norm();
    EXPECT_LT((actual.translation() - expected.translation()).norm(),
              tolerance);
    EXPECT_LT(rotationError, tolerance);
}

class FixedCurrentFromPreviousBackend final : public IVisualPnpPoseBackend {
  public:
    explicit FixedCurrentFromPreviousBackend(
        const Sophus::SE3f &currentFromPrevious)
        : m_currentFromPrevious(currentFromPrevious)
    {
    }

    bool EstimatePoseRansac(
        const std::vector<cv::Point3f> &objectPoints,
        const std::vector<cv::Point2f> &,
        const VisualPnpPoseBackendOptions &,
        VisualPnpPoseBackendResult &result) const override
    {
        result.poseValid = true;
        result.inlierCount = static_cast<int>(objectPoints.size());
        result.inlierIndices.resize(objectPoints.size());
        std::iota(result.inlierIndices.begin(), result.inlierIndices.end(), 0);
        result.T_camera_object = m_currentFromPrevious;
        return true;
    }

  private:
    Sophus::SE3f m_currentFromPrevious;
};

class SelectedInlierBackend final : public IVisualPnpPoseBackend {
  public:
    explicit SelectedInlierBackend(std::vector<int> inlierIndices)
        : m_inlierIndices(std::move(inlierIndices))
    {
    }

    bool EstimatePoseRansac(
        const std::vector<cv::Point3f> &,
        const std::vector<cv::Point2f> &,
        const VisualPnpPoseBackendOptions &,
        VisualPnpPoseBackendResult &result) const override
    {
        ++m_callCount;
        result.poseValid = true;
        result.inlierIndices = m_inlierIndices;
        result.inlierCount = static_cast<int>(m_inlierIndices.size());
        result.T_camera_object = Sophus::SE3f();
        return true;
    }

    int CallCount() const
    {
        return m_callCount;
    }

  private:
    std::vector<int> m_inlierIndices;
    mutable int m_callCount{0};
};

std::vector<cv::Point3f> MakePlanarPoints(float depth)
{
    std::vector<cv::Point3f> points;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            points.emplace_back(-0.9f + 0.6f * col, -0.6f + 0.4f * row,
                                depth);
        }
    }
    return points;
}

std::vector<cv::Point3f> MakeSlantedPlanarPoints()
{
    std::vector<cv::Point3f> points;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            const float x = -0.9f + 0.6f * col;
            const float y = -0.6f + 0.4f * row;
            points.emplace_back(x, y, 4.5f + 0.7f * x + 0.3f * y);
        }
    }
    return points;
}

TEST(KltPoseEstimatorTest, RecoversPreviousFromCurrentPoseFromSyntheticProjection)
{
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};
    const Sophus::SE3f previousFromCurrent = MakeSignedPose(
        {0.12f, -0.08f, 0.06f}, {0.025f, -0.035f, 0.045f});
    const Sophus::SE3f currentFromPrevious = previousFromCurrent.inverse();
    const std::vector<cv::Point3f> previousPoints =
        MakeNonCoplanarPreviousPoints();
    const std::vector<cv::Point2f> currentImagePoints =
        ProjectIntoCurrentCamera(previousPoints, currentFromPrevious, camera);

    const auto result = EstimateKltPnpPoseDelta(
        previousPoints, currentImagePoints, MakeEstimatorOptions(camera));

    ASSERT_TRUE(result.poseUpdated);
    EXPECT_EQ(result.inlierCount, static_cast<int>(previousPoints.size()));
    ExpectPoseNear(result.deltaTwc, previousFromCurrent, 1.0e-3f);
}

TEST(KltPoseEstimatorTest, SelectsStablePerFramePnpSolversFromEnvironment)
{
    {
        ScopedEnvironment method("SMART_DRONE_LK_PER_FRAME_PNP", "sqpnp");
        EXPECT_EQ(LkPerFramePnPMethod(), cv::SOLVEPNP_SQPNP);
    }
    {
        ScopedEnvironment method("SMART_DRONE_LK_PER_FRAME_PNP", "iterative");
        EXPECT_EQ(LkPerFramePnPMethod(), cv::SOLVEPNP_ITERATIVE);
    }
    {
        ScopedEnvironment method("SMART_DRONE_LK_PER_FRAME_PNP", "unknown");
        EXPECT_EQ(LkPerFramePnPMethod(), cv::SOLVEPNP_EPNP);
    }
}

TEST(KltPoseEstimatorTest, UsesTightPerFrameReprojectionDefaultForAllBackends)
{
    ScopedEnvironment reprojection("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ", "");
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};

    EXPECT_DOUBLE_EQ(
        MakeKltPerFramePnpPoseEstimatorOptions(camera, false).reprojectionError,
        0.5);
    EXPECT_DOUBLE_EQ(
        MakeKltPerFramePnpPoseEstimatorOptions(camera, true).reprojectionError,
        0.5);
}

TEST(KltPoseEstimatorTest, AllowsPerFrameReprojectionOverride)
{
    ScopedEnvironment reprojection("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ",
                                   "1.25");
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};

    EXPECT_DOUBLE_EQ(
        MakeKltPerFramePnpPoseEstimatorOptions(camera, false).reprojectionError,
        1.25);
    EXPECT_DOUBLE_EQ(
        MakeKltPerFramePnpPoseEstimatorOptions(camera, true).reprojectionError,
        1.25);
}

TEST(KltPoseEstimatorTest, RejectsUnsafePerFrameReprojectionOverrides)
{
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};
    {
        ScopedEnvironment reprojection("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ",
                                       "0.1");
        EXPECT_DOUBLE_EQ(MakeKltPerFramePnpPoseEstimatorOptions(camera, false)
                             .reprojectionError,
                         0.5);
    }
    {
        ScopedEnvironment reprojection("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ",
                                       "invalid");
        EXPECT_DOUBLE_EQ(MakeKltPerFramePnpPoseEstimatorOptions(camera, false)
                             .reprojectionError,
                         0.5);
    }
}

TEST(KltPoseEstimatorTest, InvertsCurrentFromPreviousPoseReturnedByBackend)
{
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};
    const Sophus::SE3f previousFromCurrent = MakeSignedPose(
        {-0.09f, 0.11f, -0.07f}, {-0.04f, 0.03f, -0.05f});
    const Sophus::SE3f currentFromPrevious = previousFromCurrent.inverse();
    FixedCurrentFromPreviousBackend backend(currentFromPrevious);
    const std::vector<cv::Point3f> previousPoints =
        MakeNonCoplanarPreviousPoints();
    const std::vector<cv::Point2f> unusedImagePoints(previousPoints.size());

    const auto result = EstimateKltPnpPoseDelta(
        previousPoints, unusedImagePoints, MakeEstimatorOptions(camera), backend);

    ASSERT_TRUE(result.poseUpdated);
    ExpectPoseNear(result.deltaTwc, previousFromCurrent, 1.0e-6f);
}

TEST(KltPoseEstimatorTest, RejectsPlanarCandidatesBeforeCallingPnp)
{
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};
    const std::vector<cv::Point3f> planarPoints = MakePlanarPoints(5.0f);
    const std::vector<cv::Point2f> imagePoints(planarPoints.size());
    SelectedInlierBackend backend({});

    const auto result = EstimateKltPnpPoseDelta(
        planarPoints, imagePoints, MakeEstimatorOptions(camera), backend);

    EXPECT_FALSE(result.poseUpdated);
    EXPECT_EQ(backend.CallCount(), 0);
}

TEST(KltPoseEstimatorTest, RejectsSlantedPlanarCandidatesWithDepthSpan)
{
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};
    const std::vector<cv::Point3f> planarPoints = MakeSlantedPlanarPoints();
    const std::vector<cv::Point2f> imagePoints(planarPoints.size());
    SelectedInlierBackend backend({});

    const auto result = EstimateKltPnpPoseDelta(
        planarPoints, imagePoints, MakeEstimatorOptions(camera), backend);

    EXPECT_FALSE(result.poseUpdated);
    EXPECT_EQ(backend.CallCount(), 0);
}

TEST(KltPoseEstimatorTest, RejectsPlanarRansacInliersAfterCandidateGate)
{
    const KltPnpCameraIntrinsics camera{420.0f, 415.0f, 320.0f, 240.0f};
    std::vector<cv::Point3f> candidatePoints = MakePlanarPoints(5.0f);
    const std::vector<cv::Point3f> diversePoints = MakeNonCoplanarPreviousPoints();
    candidatePoints.insert(candidatePoints.end(), diversePoints.begin(),
                           diversePoints.end());
    const std::vector<cv::Point2f> imagePoints(candidatePoints.size());
    std::vector<int> planarInliers(16);
    std::iota(planarInliers.begin(), planarInliers.end(), 0);
    SelectedInlierBackend backend(planarInliers);

    const auto result = EstimateKltPnpPoseDelta(
        candidatePoints, imagePoints, MakeEstimatorOptions(camera), backend);

    EXPECT_FALSE(result.poseUpdated);
    EXPECT_EQ(result.inlierCount, 16);
    EXPECT_EQ(backend.CallCount(), 1);
}

} // namespace
