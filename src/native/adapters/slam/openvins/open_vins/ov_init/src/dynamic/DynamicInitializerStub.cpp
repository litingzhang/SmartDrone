#include "dynamic/DynamicInitializer.h"

namespace ov_init {

bool DynamicInitializer::initialize(
    double &timestamp, Eigen::MatrixXd &covariance,
    std::vector<std::shared_ptr<ov_type::Type>> &order,
    std::shared_ptr<ov_type::IMU> &_imu,
    std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>>
        &_features_SLAM)
{
    (void)timestamp;
    (void)covariance;
    (void)order;
    (void)_imu;
    (void)_clones_IMU;
    (void)_features_SLAM;
    return false;
}

} // namespace ov_init
