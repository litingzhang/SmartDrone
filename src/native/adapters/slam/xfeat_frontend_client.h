#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sys/types.h>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

struct XFeatFeatureSet {
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;
};

class XFeatFrontendClient {
  public:
    XFeatFrontendClient() = default;
    ~XFeatFrontendClient();

    XFeatFrontendClient(const XFeatFrontendClient &) = delete;
    XFeatFrontendClient &operator=(const XFeatFrontendClient &) = delete;

    bool Start(const std::string &pythonBin, const std::string &workerScript, const std::string &repoPath, int topK,
               int maxPoints, std::string *err);
    void Stop();
    bool Running() const;
    bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err);
    bool DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err);

  private:
    int m_stdinFd{-1};
    int m_stdoutFd{-1};
    pid_t m_pid{-1};
    uint32_t m_requestSeq{0};

    bool WriteExact(const void *data, size_t size, std::string *err);
    bool ReadExact(void *data, size_t size, std::string *err);
};

} // namespace smartdrone::adapters::slam
