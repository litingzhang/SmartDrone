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
    struct Stats {
        double prepareMs{0.0};
        double writeMs{0.0};
        double readMs{0.0};
        double totalMs{0.0};
        uint32_t imageCount{0};
        uint32_t payloadBytes{0};
    };

    XFeatFrontendClient() = default;
    ~XFeatFrontendClient();

    XFeatFrontendClient(const XFeatFrontendClient &) = delete;
    XFeatFrontendClient &operator=(const XFeatFrontendClient &) = delete;

    bool Start(const std::string &pythonBin, const std::string &workerScript, const std::string &repoPath,
               const std::string &device, int topK, int maxPoints, std::string *err);
    void Stop();
    bool Running() const;
    bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err);
    bool DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err);
    bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray, XFeatFeatureSet &leftFeatures,
                                XFeatFeatureSet &rightFeatures, std::string *err);
    Stats LastStats() const;

  private:
    int m_stdinFd{-1};
    int m_stdoutFd{-1};
    pid_t m_pid{-1};
    uint32_t m_requestSeq{0};
    Stats m_lastStats{};

    static bool PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err);
    bool ReadFeatureSet(XFeatFeatureSet &outFeatures, std::string *err);
    bool WriteExact(const void *data, size_t size, std::string *err);
    bool ReadExact(void *data, size_t size, std::string *err);
};

} // namespace smartdrone::adapters::slam
