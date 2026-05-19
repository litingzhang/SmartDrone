#include "core/application/session/imu_window_filter.h"

#include <algorithm>
#include <cmath>

namespace smartdrone::core::application {
namespace {

using ImuReading = smartdrone::core::ports::ImuReading;

constexpr float kMaxAccelNormMps2 = 200.0f;
constexpr float kMaxGyroNormRadps = 40.0f;
constexpr double kMinSampleDtSec = 1e-6;

double ImuSampleTimeSec(const ImuReading &sample)
{
    return static_cast<double>(sample.timestampNs) * 1e-9;
}

bool ImuReadingInPhysicalRange(const ImuReading &sample)
{
    const float accelNorm = std::sqrt(
        sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
    const float gyroNorm = std::sqrt(
        sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz);
    return accelNorm <= kMaxAccelNormMps2 && gyroNorm <= kMaxGyroNormRadps;
}

bool AcceptImuSampleTime(double sampleTime, ImuWindowValidation &stats,
                         double &lastTime, bool &haveLastTime)
{
    if (haveLastTime) {
        const double dt = sampleTime - lastTime;
        if (!(dt > kMinSampleDtSec)) {
            ++stats.droppedNonMonotonic;
            return false;
        }
        stats.largestGapSec = std::max(stats.largestGapSec, dt);
    }
    lastTime = sampleTime;
    haveLastTime = true;
    return true;
}

bool AcceptImuSample(const ImuReading &sample, ImuWindowValidation &stats,
                     double &lastTime, bool &haveLastTime)
{
    if (!IsFiniteImuReading(sample)) {
        ++stats.droppedNonFinite;
        return false;
    }
    if (!ImuReadingInPhysicalRange(sample)) {
        ++stats.droppedOutOfRange;
        return false;
    }
    return AcceptImuSampleTime(ImuSampleTimeSec(sample), stats, lastTime,
                               haveLastTime);
}

std::vector<ImuReading> FilterImuWindow(const std::vector<ImuReading> &input,
                                        ImuWindowValidation &stats)
{
    std::vector<ImuReading> filtered;
    filtered.reserve(input.size());
    double lastTime = 0.0;
    bool haveLastTime = false;
    for (const auto &sample : input) {
        if (AcceptImuSample(sample, stats, lastTime, haveLastTime)) {
            filtered.push_back(sample);
        }
    }
    return filtered;
}

void UpdateImuWindowCoverage(const std::vector<ImuReading> &vImu,
                             double prevFrameTime, double frameTime,
                             ImuWindowValidation &stats)
{
    stats.firstLeadSec = ImuSampleTimeSec(vImu.front()) - prevFrameTime;
    stats.tailLagSec = frameTime - ImuSampleTimeSec(vImu.back());
}

bool ValidateImuWindowCoverage(double expectedImuDtSec,
                               ImuWindowValidation &stats)
{
    const double boundarySlackSec = std::max(6.0 * expectedImuDtSec, 0.010);
    const double maxGapSec = std::max(12.0 * expectedImuDtSec, 0.030);
    if (stats.firstLeadSec > boundarySlackSec) {
        stats.failureReason = "missing_leading_coverage";
        return false;
    }
    if (stats.tailLagSec > boundarySlackSec) {
        stats.failureReason = "missing_trailing_coverage";
        return false;
    }
    if (stats.largestGapSec > maxGapSec) {
        stats.failureReason = "large_internal_gap";
        return false;
    }
    return true;
}

} // namespace

bool IsFiniteImuReading(const smartdrone::core::ports::ImuReading &reading)
{
    return std::isfinite(reading.ax) && std::isfinite(reading.ay) &&
           std::isfinite(reading.az) && std::isfinite(reading.gx) &&
           std::isfinite(reading.gy) && std::isfinite(reading.gz);
}

bool SanitizeImuWindow(std::vector<smartdrone::core::ports::ImuReading> &vImu,
                       double prevFrameTime, double frameTime,
                       double expectedImuDtSec, ImuWindowValidation &stats)
{
    stats = ImuWindowValidation{};
    stats.inputCount = vImu.size();
    auto filtered = FilterImuWindow(vImu, stats);
    vImu.swap(filtered);
    stats.outputCount = vImu.size();
    if (vImu.size() < 2) {
        stats.failureReason = "too_few_samples";
        return false;
    }
    UpdateImuWindowCoverage(vImu, prevFrameTime, frameTime, stats);
    return ValidateImuWindowCoverage(expectedImuDtSec, stats);
}

} // namespace smartdrone::core::application
