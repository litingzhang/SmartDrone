#pragma once

#include <netinet/in.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

class UdpImageSender {
  public:
    static constexpr uint8_t FLAG_FEATURE_POINTS = 0x01;
    static constexpr double MAX_IMAGE_FPS = 12.0;
    static constexpr double MIN_FRAME_INTERVAL_SEC = 1.0 / MAX_IMAGE_FPS;

    struct Slot {
        int camIndex;
        uint64_t frameId{0};
        uint32_t seq;
        double frameTime;
        int width{0};
        int height{0};
        bool sendImage{true};
        bool sendFeature{false};
        cv::Mat preview;
        std::vector<cv::Point2f> trackedPoints;
        std::vector<uint8_t> featureBuf;
    };

    bool Open(const std::string &ip, int port, int jpegQuality = 80, int maxPayload = 1200, int maxQueue = 4);
    void Close();
    void Enqueue(int camIndex, uint64_t frameId, uint32_t seq, double frameTime, const cv::Mat &gray,
                 const std::vector<cv::Point2f> &trackedPoints = {}, bool sendImage = true, bool sendFeature = true);

  private:
#pragma pack(push, 1)
    struct PacketHeader {
        uint32_t magic;
        uint16_t version;
        uint8_t camIndex;
        uint8_t flags;
        uint32_t seq;
        double frameTime;
        uint32_t frameId;
        uint16_t chunkIdx;
        uint16_t chunkCnt;
        uint32_t totalSize;
        uint32_t chunkSize;
    };
#pragma pack(pop)

    void Loop(int camIndex);
    void SendFeaturePacket(Slot &slot, uint32_t frameId, int width, int height,
                           const std::vector<cv::Point2f> &trackedPoints);
    static bool FillPreview(int camIndex, uint32_t seq, const cv::Mat &gray, cv::Mat &preview);
    static void WriteU16Le(uint8_t *out, size_t offset, uint16_t value);

    int m_sock{-1};
    sockaddr_in m_dst{};

    int m_jpegQuality{80};
    int m_maxPayload{1200};
    int m_maxQueue{4};

    std::atomic<bool> m_running{false};
    std::thread m_th[2];
    std::mutex m_mu[2];
    std::condition_variable m_cv[2];
    std::array<std::vector<Slot>, 2> m_slots;
    std::deque<size_t> m_ready[2];
    std::deque<size_t> m_free[2];
    double m_lastAcceptedFrameTime[2]{-1.0, -1.0};
};
