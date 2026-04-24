#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <sys/types.h>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

class XFeatNativeExtractor;

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

    XFeatFrontendClient();
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
    enum class BackendMode : uint8_t {
        None = 0,
        Worker,
        Native,
    };

    int m_stdinFd{-1};
    int m_stdoutFd{-1};
    pid_t m_pid{-1};
    uint32_t m_requestSeq{0};
    Stats m_lastStats{};
    BackendMode m_backendMode{BackendMode::None};
    std::unique_ptr<XFeatNativeExtractor> m_nativeExtractor;
    XFeatFeatureSet m_prevStereoLeftFeatures;
    cv::Mat m_prevStereoLeftGray;
    bool m_havePrevStereoLeftFeatures{false};

    static bool PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err);
    static std::vector<int> ComputeTemporalStableIndices(const XFeatFeatureSet &previous, const cv::Mat &previousGray,
                                                         const XFeatFeatureSet &current, const cv::Mat &currentGray);
    static void ReorderFeaturesByIndices(const XFeatFeatureSet &source, const std::vector<int> &indices,
                                         XFeatFeatureSet &dest);
    bool ReadFeatureSet(XFeatFeatureSet &outFeatures, std::string *err);
    bool WriteExact(const void *data, size_t size, std::string *err);
    bool ReadExact(void *data, size_t size, std::string *err);
};

} // namespace smartdrone::adapters::slam
