#include "adapters/camera/gazebo_image_processor.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace SmartDrone::Adapters::Camera {
namespace {

int PixelChannels(GazeboImagePixelFormat format)
{
    return format == GazeboImagePixelFormat::Mono8 ? 1 : 3;
}

int ConversionCode(GazeboImagePixelFormat format)
{
    return format == GazeboImagePixelFormat::Rgb8 ? cv::COLOR_RGB2GRAY
                                                   : cv::COLOR_BGR2GRAY;
}

std::uint64_t DropBucket(std::uint64_t measurementTimestampNs)
{
    const std::uint64_t frameKey = measurementTimestampNs / 1000000ULL;
    return (frameKey * 2654435761ULL) % 100ULL;
}

} // namespace

GazeboImageProcessor::GazeboImageProcessor(GazeboImageFaultConfig config)
    : m_config(std::move(config))
{
}

bool GazeboImageProcessor::ShouldDrop(
    std::uint64_t pairMeasurementTimestampNs) const
{
    return m_config.dropRate > 0.0 &&
           static_cast<double>(DropBucket(pairMeasurementTimestampNs)) /
                   100.0 <
               m_config.dropRate;
}

void GazeboImageProcessor::UpdateConfig(GazeboImageFaultConfig config)
{
    if (config.generation == m_config.generation) {
        return;
    }
    m_config = std::move(config);
    m_firstMeasurementTimestampNs = 0;
}

std::uint64_t GazeboImageProcessor::Generation() const
{
    return m_config.generation;
}

bool GazeboImageProcessor::BuildGray(const GazeboRawImage &image,
                                     cv::Mat &gray) const
{
    if (!image.payload || image.payload->empty()) {
        return false;
    }
    const int channels = PixelChannels(image.pixelFormat);
    cv::Mat source(image.height, image.width,
                   channels == 1 ? CV_8UC1 : CV_8UC3,
                   const_cast<std::uint8_t *>(image.payload->data()),
                   image.step);
    if (channels == 1) {
        gray = EffectsEnabled() ? source.clone() : source;
        return true;
    }
    cv::cvtColor(source, gray, ConversionCode(image.pixelFormat));
    return true;
}

bool GazeboImageProcessor::EffectsEnabled() const
{
    return m_config.blurKernel > 1 || m_config.blurSigma > 0.0 ||
           m_config.brightness != 1.0 ||
           m_config.noiseStddev > 0.0 || m_config.blackoutMs > 0;
}

void GazeboImageProcessor::ApplyNoise(cv::Mat &gray,
                                      const GazeboRawImage &image,
                                      GazeboStereoEye eye) const
{
    if (m_config.noiseStddev <= 0.0) {
        return;
    }
    const std::uint64_t eyeOffset =
        eye == GazeboStereoEye::Left ? 0ULL : 1ULL;
    cv::RNG random(static_cast<std::uint64_t>(image.sequence) * 2ULL +
                   eyeOffset + 1ULL);
    cv::Mat noise(gray.size(), CV_16SC1);
    random.fill(noise, cv::RNG::NORMAL, 0.0, m_config.noiseStddev);
    cv::Mat working;
    gray.convertTo(working, CV_16SC1);
    working += noise;
    working.convertTo(gray, CV_8UC1);
}

bool GazeboImageProcessor::BlackoutActive(
    std::uint64_t measurementTimestampNs)
{
    if (m_config.blackoutMs <= 0) {
        return false;
    }
    if (m_firstMeasurementTimestampNs == 0 ||
        measurementTimestampNs < m_firstMeasurementTimestampNs) {
        m_firstMeasurementTimestampNs = measurementTimestampNs;
    }
    const std::uint64_t elapsed =
        measurementTimestampNs - m_firstMeasurementTimestampNs;
    return elapsed < static_cast<std::uint64_t>(m_config.blackoutMs) *
                         1000000ULL;
}

void GazeboImageProcessor::ApplyEffects(cv::Mat &gray,
                                        const GazeboRawImage &image,
                                        GazeboStereoEye eye)
{
    if (BlackoutActive(image.measurementTimestampNs)) {
        gray.setTo(0);
        return;
    }
    if (m_config.brightness != 1.0) {
        gray.convertTo(gray, CV_8UC1, m_config.brightness);
    }
    if (m_config.blurKernel > 1 || m_config.blurSigma > 0.0) {
        const int requested = m_config.blurKernel > 1
                                  ? m_config.blurKernel
                                  : static_cast<int>(std::ceil(
                                        m_config.blurSigma * 6.0));
        const int kernel = std::clamp(requested | 1, 3, 31);
        cv::GaussianBlur(gray, gray, cv::Size(kernel, kernel),
                         m_config.blurSigma);
    }
    ApplyNoise(gray, image, eye);
}

bool GazeboImageProcessor::Convert(
    const GazeboRawImage &image, GazeboStereoEye eye,
    SmartDrone::Core::Ports::ImageFrame &out)
{
    cv::Mat gray;
    if (!BuildGray(image, gray)) {
        return false;
    }
    if (EffectsEnabled()) {
        ApplyEffects(gray, image, eye);
    }
    out.cameraId = eye == GazeboStereoEye::Left ? 0 : 1;
    out.timestampNs = image.measurementTimestampNs;
    out.captureMonotonicNs = image.captureMonotonicNs;
    out.arriveNs = image.arrivalMonotonicNs;
    out.sequence = image.sequence;
    out.gray = gray;
    out.owner = EffectsEnabled() || image.pixelFormat != GazeboImagePixelFormat::Mono8
                    ? std::static_pointer_cast<void>(
                          std::make_shared<cv::Mat>(gray))
                    : std::static_pointer_cast<void>(image.payload);
    return true;
}

} // namespace SmartDrone::Adapters::Camera
