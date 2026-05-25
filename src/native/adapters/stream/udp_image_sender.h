#pragma once

#include <netinet/in.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

class UdpImageSender {
  public:
    using DestinationResolver = std::function<bool(sockaddr_in &)>;

    static constexpr uint8_t FLAG_FEATURE_POINTS = 0x01;
    static constexpr double MAX_IMAGE_FPS = 15.0;
    static constexpr double MIN_FRAME_INTERVAL_SEC = 1.0 / MAX_IMAGE_FPS;
    static constexpr std::size_t MAX_PENDING_SLOTS = 16;

    struct OpenConfig {
        std::string ip;
        int port{0};
        int jpegQuality{80};
        int maxPayload{1200};
        int maxQueue{4};
        DestinationResolver destinationResolver{};
    };

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

    struct EnqueueRequest {
        int camIndex;
        uint64_t frameId;
        uint32_t seq;
        double frameTime;
        const cv::Mat &gray;
        const std::vector<cv::Point2f> &trackedPoints;
        bool sendImage;
        bool sendFeature;
    };

    bool Open(OpenConfig config);
    void Close();
    void Enqueue(const EnqueueRequest &request);
    void StepOnce();
    void StepAll();

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

    struct DestinationState {
        sockaddr_in dst{};
        DestinationResolver destinationResolver{};
        int port{0};
    };

    struct PendingSlot {
        std::size_t queueSeq{0};
        std::shared_ptr<const Slot> slot;
    };

    void StepCamera(int camIndex);
    bool StepCameraOnce(int camIndex);
    bool PopReadySlot(int camIndex, Slot &slot);
    std::shared_ptr<const Slot> BuildSlot(const EnqueueRequest &request);
    bool AcceptFrameTime(int camIndex, double frameTime);
    void StorePendingSlot(int camIndex, std::shared_ptr<const Slot> slot);
    void ResetCameraQueue(int camIndex);
    std::size_t PendingQueueDepth() const;
    void PopulateSlot(Slot &slot, const EnqueueRequest &request);
    void SendSlot(Slot &slot);
    bool EncodePreviewJpeg(const Slot &slot, std::vector<uchar> &jpeg) const;
    void SendImagePackets(const Slot &slot, const std::vector<uchar> &jpeg);
    void SendPacket(const PacketHeader &header, const uint8_t *payload, size_t payloadSize);
    void SendFeaturePacket(Slot &slot, uint32_t frameId, int width, int height,
                           const std::vector<cv::Point2f> &trackedPoints);
    bool BuildFeaturePayload(std::vector<uint8_t> &payloadBuf, int width, int height,
                             const std::vector<cv::Point2f> &trackedPoints) const;
    int SocketFd() const;
    std::shared_ptr<const DestinationState> LoadDestinationState() const;
    sockaddr_in ResolveDestination();
    static bool FillPreview(int camIndex, uint32_t seq, const cv::Mat &gray, cv::Mat &preview);
    static void WriteU16Le(uint8_t *out, size_t offset, uint16_t value);

    std::atomic<int> m_sock{-1};
    std::shared_ptr<const DestinationState> m_destinationState;
    std::atomic<uint32_t> m_lastResolvedIp{0};

    std::atomic<int> m_jpegQuality{80};
    std::atomic<int> m_maxPayload{1200};
    std::atomic<int> m_queueDepth{4};

    std::array<std::array<std::shared_ptr<const PendingSlot>,
                          MAX_PENDING_SLOTS>,
               2>
        m_pendingSlots{};
    std::atomic<std::size_t> m_writeSeq[2]{{0}, {0}};
    std::atomic<std::size_t> m_readSeq[2]{{0}, {0}};
    std::atomic<double> m_lastAcceptedFrameTime[2]{{-1.0}, {-1.0}};
};
