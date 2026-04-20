#include "adapters/slam/xfeat_frontend_client.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

namespace smartdrone::adapters::slam {

namespace {

constexpr std::array<char, 8> kReadyMagic{'X', 'F', 'W', 'K', 'R', 'D', 'Y', '1'};

struct RequestHeader {
    uint32_t seq{0};
    uint32_t imageCount{0};
};

struct ImageHeader {
    uint32_t rows{0};
    uint32_t cols{0};
    uint32_t bytes{0};
};

struct ResponseHeader {
    uint32_t seq{0};
    uint32_t imageCount{0};
};

struct FeatureHeader {
    uint32_t count{0};
    uint32_t descriptorDim{0};
};

std::string ErrnoMessage(const char *prefix)
{
    return std::string(prefix) + ": " + std::strerror(errno);
}

bool SetCloseOnExec(int fd, std::string *err)
{
    const int flags = fcntl(fd, F_GETFD);
    if (flags < 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("fcntl(F_GETFD) failed");
        }
        return false;
    }
    if (fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("fcntl(F_SETFD) failed");
        }
        return false;
    }
    return true;
}

} // namespace

XFeatFrontendClient::~XFeatFrontendClient() { Stop(); }

bool XFeatFrontendClient::Start(const std::string &pythonBin, const std::string &workerScript,
                                const std::string &repoPath, const std::string &device, int topK, int maxPoints,
                                std::string *err)
{
    Stop();

    int stdinPipe[2]{-1, -1};
    int stdoutPipe[2]{-1, -1};
    if (pipe(stdinPipe) != 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("pipe(stdin) failed");
        }
        return false;
    }
    if (pipe(stdoutPipe) != 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("pipe(stdout) failed");
        }
        close(stdinPipe[0]);
        close(stdinPipe[1]);
        return false;
    }

    std::string pipeErr;
    if (!SetCloseOnExec(stdinPipe[1], &pipeErr) || !SetCloseOnExec(stdoutPipe[0], &pipeErr)) {
        if (err != nullptr) {
            *err = pipeErr;
        }
        close(stdinPipe[0]);
        close(stdinPipe[1]);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        if (err != nullptr) {
            *err = ErrnoMessage("fork failed");
        }
        close(stdinPipe[0]);
        close(stdinPipe[1]);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);
        return false;
    }

    if (pid == 0) {
        dup2(stdinPipe[0], STDIN_FILENO);
        dup2(stdoutPipe[1], STDOUT_FILENO);

        close(stdinPipe[0]);
        close(stdinPipe[1]);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);

        const std::string topKText = std::to_string(std::max(1, topK));
        const std::string maxPointsText = std::to_string(std::max(1, maxPoints));

        std::vector<char *> argv;
        argv.push_back(const_cast<char *>(pythonBin.c_str()));
        argv.push_back(const_cast<char *>(workerScript.c_str()));
        argv.push_back(const_cast<char *>("--repo"));
        argv.push_back(const_cast<char *>(repoPath.c_str()));
        argv.push_back(const_cast<char *>("--device"));
        argv.push_back(const_cast<char *>(device.c_str()));
        argv.push_back(const_cast<char *>("--top-k"));
        argv.push_back(const_cast<char *>(topKText.c_str()));
        argv.push_back(const_cast<char *>("--max-points"));
        argv.push_back(const_cast<char *>(maxPointsText.c_str()));
        argv.push_back(nullptr);

        execvp(pythonBin.c_str(), argv.data());
        _exit(127);
    }

    close(stdinPipe[0]);
    close(stdoutPipe[1]);

    m_stdinFd = stdinPipe[1];
    m_stdoutFd = stdoutPipe[0];
    m_pid = pid;
    m_requestSeq = 0;
    std::array<char, kReadyMagic.size()> ready{};
    if (!ReadExact(ready.data(), ready.size(), err)) {
        Stop();
        return false;
    }
    if (ready != kReadyMagic) {
        if (err != nullptr) {
            *err = "xfeat worker handshake mismatch";
        }
        Stop();
        return false;
    }
    return true;
}

void XFeatFrontendClient::Stop()
{
    if (m_stdinFd >= 0) {
        close(m_stdinFd);
        m_stdinFd = -1;
    }
    if (m_stdoutFd >= 0) {
        close(m_stdoutFd);
        m_stdoutFd = -1;
    }
    if (m_pid > 0) {
        kill(m_pid, SIGTERM);
        int status = 0;
        waitpid(m_pid, &status, 0);
        m_pid = -1;
    }
    m_requestSeq = 0;
}

bool XFeatFrontendClient::Running() const { return m_pid > 0 && m_stdinFd >= 0 && m_stdoutFd >= 0; }

bool XFeatFrontendClient::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
{
    XFeatFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool XFeatFrontendClient::DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err)
{
    outFeatures.keypoints.clear();
    outFeatures.descriptors.release();
    if (!Running()) {
        if (err != nullptr) {
            *err = "xfeat worker not running";
        }
        return false;
    }
    if (gray.empty()) {
        if (err != nullptr) {
            *err = "xfeat input frame is empty";
        }
        return false;
    }

    cv::Mat gray8;
    if (!PrepareGrayImage(gray, gray8, err)) {
        return false;
    }

    const uint32_t seq = ++m_requestSeq;
    const RequestHeader header{seq, 1};
    const ImageHeader imageHeader{static_cast<uint32_t>(gray8.rows), static_cast<uint32_t>(gray8.cols),
                                  static_cast<uint32_t>(gray8.rows * gray8.cols)};
    if (!WriteExact(&header, sizeof(header), err)) {
        Stop();
        return false;
    }
    if (!WriteExact(&imageHeader, sizeof(imageHeader), err)) {
        Stop();
        return false;
    }
    if (!WriteExact(gray8.data, static_cast<size_t>(imageHeader.bytes), err)) {
        Stop();
        return false;
    }

    ResponseHeader response{};
    if (!ReadExact(&response, sizeof(response), err)) {
        Stop();
        return false;
    }
    if (response.seq != seq) {
        if (err != nullptr) {
            *err = "xfeat worker response sequence mismatch";
        }
        Stop();
        return false;
    }
    if (response.imageCount != 1) {
        if (err != nullptr) {
            *err = "xfeat worker response image count mismatch";
        }
        Stop();
        return false;
    }

    if (!ReadFeatureSet(outFeatures, err)) {
        Stop();
        return false;
    }
    return true;
}

bool XFeatFrontendClient::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                 XFeatFeatureSet &leftFeatures, XFeatFeatureSet &rightFeatures,
                                                 std::string *err)
{
    leftFeatures.keypoints.clear();
    leftFeatures.descriptors.release();
    rightFeatures.keypoints.clear();
    rightFeatures.descriptors.release();
    if (!Running()) {
        if (err != nullptr) {
            *err = "xfeat worker not running";
        }
        return false;
    }
    if (leftGray.empty() || rightGray.empty()) {
        if (err != nullptr) {
            *err = "xfeat stereo input frame is empty";
        }
        return false;
    }

    cv::Mat leftGray8;
    cv::Mat rightGray8;
    if (!PrepareGrayImage(leftGray, leftGray8, err) || !PrepareGrayImage(rightGray, rightGray8, err)) {
        return false;
    }

    const uint32_t seq = ++m_requestSeq;
    const RequestHeader header{seq, 2};
    const ImageHeader leftHeader{static_cast<uint32_t>(leftGray8.rows), static_cast<uint32_t>(leftGray8.cols),
                                 static_cast<uint32_t>(leftGray8.rows * leftGray8.cols)};
    const ImageHeader rightHeader{static_cast<uint32_t>(rightGray8.rows), static_cast<uint32_t>(rightGray8.cols),
                                  static_cast<uint32_t>(rightGray8.rows * rightGray8.cols)};
    if (!WriteExact(&header, sizeof(header), err) || !WriteExact(&leftHeader, sizeof(leftHeader), err) ||
        !WriteExact(leftGray8.data, static_cast<size_t>(leftHeader.bytes), err) ||
        !WriteExact(&rightHeader, sizeof(rightHeader), err) ||
        !WriteExact(rightGray8.data, static_cast<size_t>(rightHeader.bytes), err)) {
        Stop();
        return false;
    }

    ResponseHeader response{};
    if (!ReadExact(&response, sizeof(response), err)) {
        Stop();
        return false;
    }
    if (response.seq != seq) {
        if (err != nullptr) {
            *err = "xfeat worker response sequence mismatch";
        }
        Stop();
        return false;
    }
    if (response.imageCount != 2) {
        if (err != nullptr) {
            *err = "xfeat worker stereo response image count mismatch";
        }
        Stop();
        return false;
    }
    if (!ReadFeatureSet(leftFeatures, err) || !ReadFeatureSet(rightFeatures, err)) {
        Stop();
        return false;
    }
    return true;
}

bool XFeatFrontendClient::PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err)
{
    if (gray.type() == CV_8UC1 && gray.isContinuous()) {
        gray8 = gray;
        return true;
    }
    if (gray.channels() == 1) {
        gray.convertTo(gray8, CV_8UC1);
    } else {
        cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
    }
    if (!gray8.isContinuous()) {
        gray8 = gray8.clone();
    }
    if (gray8.empty()) {
        if (err != nullptr) {
            *err = "xfeat gray image preparation failed";
        }
        return false;
    }
    return true;
}

bool XFeatFrontendClient::ReadFeatureSet(XFeatFeatureSet &outFeatures, std::string *err)
{
    FeatureHeader featureHeader{};
    if (!ReadExact(&featureHeader, sizeof(featureHeader), err)) {
        return false;
    }

    const size_t xyCount = static_cast<size_t>(featureHeader.count) * 2u;
    std::vector<float> xy(xyCount, 0.0f);
    if (!xy.empty() && !ReadExact(xy.data(), xy.size() * sizeof(float), err)) {
        return false;
    }
    const size_t descriptorValueCount =
        static_cast<size_t>(featureHeader.count) * static_cast<size_t>(featureHeader.descriptorDim);
    std::vector<float> descriptors(descriptorValueCount, 0.0f);
    if (!descriptors.empty() && !ReadExact(descriptors.data(), descriptors.size() * sizeof(float), err)) {
        return false;
    }

    outFeatures.keypoints.reserve(featureHeader.count);
    for (uint32_t i = 0; i < featureHeader.count; ++i) {
        outFeatures.keypoints.emplace_back(xy[static_cast<size_t>(i) * 2u], xy[static_cast<size_t>(i) * 2u + 1u]);
    }
    if (featureHeader.count > 0 && featureHeader.descriptorDim > 0) {
        cv::Mat desc(static_cast<int>(featureHeader.count), static_cast<int>(featureHeader.descriptorDim), CV_32F);
        std::memcpy(desc.data, descriptors.data(), descriptors.size() * sizeof(float));
        outFeatures.descriptors = std::move(desc);
    }
    return true;
}

bool XFeatFrontendClient::WriteExact(const void *data, size_t size, std::string *err)
{
    const uint8_t *cursor = static_cast<const uint8_t *>(data);
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t written = write(m_stdinFd, cursor, remaining);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (err != nullptr) {
                *err = ErrnoMessage("xfeat worker write failed");
            }
            return false;
        }
        if (written == 0) {
            if (err != nullptr) {
                *err = "xfeat worker write returned 0";
            }
            return false;
        }
        cursor += static_cast<size_t>(written);
        remaining -= static_cast<size_t>(written);
    }
    return true;
}

bool XFeatFrontendClient::ReadExact(void *data, size_t size, std::string *err)
{
    uint8_t *cursor = static_cast<uint8_t *>(data);
    size_t remaining = size;
    while (remaining > 0) {
        const ssize_t readBytes = read(m_stdoutFd, cursor, remaining);
        if (readBytes < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (err != nullptr) {
                *err = ErrnoMessage("xfeat worker read failed");
            }
            return false;
        }
        if (readBytes == 0) {
            if (err != nullptr) {
                *err = "xfeat worker closed stdout";
            }
            return false;
        }
        cursor += static_cast<size_t>(readBytes);
        remaining -= static_cast<size_t>(readBytes);
    }
    return true;
}

} // namespace smartdrone::adapters::slam
