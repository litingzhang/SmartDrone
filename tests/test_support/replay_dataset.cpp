#include "test_support/replay_dataset.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace smartdrone::tests {
namespace {

std::string TrimAsciiWhitespace(std::string text)
{
    const auto isSpace = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
    while (!text.empty() && isSpace(static_cast<unsigned char>(text.front()))) {
        text.erase(text.begin());
    }
    while (!text.empty() && isSpace(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    return text;
}

std::vector<ReplayImageSample> LoadImageIndex(const std::filesystem::path &csvPath,
                                              const std::filesystem::path &imageDir, size_t maxFrames)
{
    std::ifstream input(csvPath);
    if (!input) {
        throw std::runtime_error("failed to open image index: " + csvPath.string());
    }

    std::vector<ReplayImageSample> samples;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream lineStream(line);
        std::string timestampField;
        std::string fileField;
        if (!std::getline(lineStream, timestampField, ',')) {
            continue;
        }
        if (!std::getline(lineStream, fileField)) {
            continue;
        }
        ReplayImageSample sample{};
        sample.timestampNs = static_cast<uint64_t>(std::stoull(TrimAsciiWhitespace(timestampField)));
        sample.path = imageDir / TrimAsciiWhitespace(fileField);
        samples.push_back(std::move(sample));
        if (maxFrames > 0 && samples.size() >= maxFrames) {
            break;
        }
    }
    return samples;
}

std::vector<ReplayImuSample> LoadImuCsv(const std::filesystem::path &csvPath)
{
    std::ifstream input(csvPath);
    if (!input) {
        throw std::runtime_error("failed to open imu csv: " + csvPath.string());
    }

    std::vector<ReplayImuSample> samples;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream lineStream(line);
        std::string field;
        ReplayImuSample sample{};
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.timestampNs = std::stoll(TrimAsciiWhitespace(field));
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.gx = std::stof(TrimAsciiWhitespace(field));
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.gy = std::stof(TrimAsciiWhitespace(field));
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.gz = std::stof(TrimAsciiWhitespace(field));
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.ax = std::stof(TrimAsciiWhitespace(field));
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.ay = std::stof(TrimAsciiWhitespace(field));
        if (!std::getline(lineStream, field, ',')) {
            continue;
        }
        sample.az = std::stof(TrimAsciiWhitespace(field));
        samples.push_back(sample);
    }
    return samples;
}

cv::Mat LoadGrayImage(const std::filesystem::path &path)
{
    return cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
}

} // namespace

ReplayDataset ReplayDataset::Load(const std::filesystem::path &rootDir, size_t maxFrames)
{
    ReplayDataset dataset;
    dataset.m_leftFrames = LoadImageIndex(rootDir / "cam0" / "data.csv", rootDir / "cam0", maxFrames);
    dataset.m_rightFrames = LoadImageIndex(rootDir / "cam1" / "data.csv", rootDir / "cam1", maxFrames);
    dataset.m_imuSamples = LoadImuCsv(rootDir / "imu.csv");
    if (dataset.m_leftFrames.size() != dataset.m_rightFrames.size()) {
        throw std::runtime_error("left/right frame count mismatch");
    }
    return dataset;
}

ReplayCameraProvider::ReplayCameraProvider(const ReplayDataset &dataset) : m_dataset(dataset) {}

bool ReplayCameraProvider::Start()
{
    m_started = true;
    m_nextIndex = 0;
    return true;
}

void ReplayCameraProvider::Stop()
{
    m_started = false;
    m_nextIndex = 0;
}

bool ReplayCameraProvider::GrabStereo(smartdrone::core::ports::StereoFrame &out, int, bool, uint64_t minTimestampNs)
{
    if (!m_started) {
        return false;
    }

    while (m_nextIndex < m_dataset.LeftFrames().size()) {
        const ReplayImageSample &left = m_dataset.LeftFrames()[m_nextIndex];
        const ReplayImageSample &right = m_dataset.RightFrames()[m_nextIndex];
        const uint64_t earlierTimestampNs = std::min(left.timestampNs, right.timestampNs);
        const uint64_t laterTimestampNs = std::max(left.timestampNs, right.timestampNs);
        const uint64_t pairTimestampNs = earlierTimestampNs + ((laterTimestampNs - earlierTimestampNs) / 2ULL);
        ++m_nextIndex;
        if (pairTimestampNs < minTimestampNs) {
            continue;
        }

        out.left.timestampNs = left.timestampNs;
        out.right.timestampNs = right.timestampNs;
        out.left.arriveNs = static_cast<int64_t>(left.timestampNs);
        out.right.arriveNs = static_cast<int64_t>(right.timestampNs);
        out.left.sequence = static_cast<uint32_t>(m_nextIndex);
        out.right.sequence = static_cast<uint32_t>(m_nextIndex);
        out.left.gray = LoadGrayImage(left.path);
        out.right.gray = LoadGrayImage(right.path);
        return !out.left.gray.empty() && !out.right.gray.empty();
    }

    return false;
}

smartdrone::core::ports::CameraHealth ReplayCameraProvider::GetHealth() const
{
    return {true, 0};
}

ReplayImuProvider::ReplayImuProvider(const ReplayDataset &dataset) : m_dataset(dataset) {}

bool ReplayImuProvider::Start()
{
    m_started = true;
    m_cursor = 0;
    return true;
}

void ReplayImuProvider::Stop()
{
    m_started = false;
    m_cursor = 0;
}

bool ReplayImuProvider::Ready() const { return m_started; }

std::vector<smartdrone::core::ports::ImuReading> ReplayImuProvider::PopWindow(int64_t fromNs, int64_t toNs)
{
    std::vector<smartdrone::core::ports::ImuReading> out;
    if (!m_started || fromNs >= toNs) {
        return out;
    }

    while (m_cursor < m_dataset.ImuSamples().size() && m_dataset.ImuSamples()[m_cursor].timestampNs < fromNs) {
        ++m_cursor;
    }
    size_t index = m_cursor;
    while (index < m_dataset.ImuSamples().size()) {
        const ReplayImuSample &sample = m_dataset.ImuSamples()[index];
        if (sample.timestampNs > toNs) {
            break;
        }
        out.push_back({sample.timestampNs, sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz});
        ++index;
    }
    m_cursor = index;
    return out;
}

} // namespace smartdrone::tests
