#pragma once

#include <arpa/inet.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cerrno>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/uio.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <array>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
// ---------------- UDP image sender ----------------
class UdpImageSender {
  public:
    static constexpr uint8_t FLAG_FEATURE_POINTS = 0x01;
    static constexpr double MAX_IMAGE_FPS = 12.0;
    static constexpr double MIN_FRAME_INTERVAL_SEC = 1.0 / MAX_IMAGE_FPS;

    struct Slot {
        int camIndex;  // 0=L, 1=R
        uint32_t seq;   // sequence for debug
        double frameTime; // seconds
        cv::Mat preview; // reusable preview storage
        std::vector<cv::Point2f> trackedPoints;
        std::vector<uint8_t> featureBuf;
    };

    bool Open(const std::string &ip, int port, int jpegQuality = 80,
              int maxPayload = 1200, // safe under typical MTU (1500)
              int maxQueue = 4)
    {
        m_jpegQuality = std::max(10, std::min(95, jpegQuality));
        m_maxPayload = std::max(400, maxPayload);
        m_maxQueue = std::max(1, maxQueue);

        m_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock < 0) {
            std::cerr << "[udp] socket() failed: " << strerror(errno) << "\n";
            return false;
        }

        memset(&m_dst, 0, sizeof(m_dst));
        m_dst.sin_family = AF_INET;
        m_dst.sin_port = htons((uint16_t)port);
        if (::inet_pton(AF_INET, ip.c_str(), &m_dst.sin_addr) != 1) {
            std::cerr << "[udp] inet_pton failed for " << ip << "\n";
            ::close(m_sock);
            m_sock = -1;
            return false;
        }
        std::cerr << "[udp] sending to " << ip << ":" << port << "\n";

        for (int cam = 0; cam < 2; ++cam) {
            std::lock_guard<std::mutex> lk(m_mu[cam]);
            m_slots[cam].assign(static_cast<size_t>(m_maxQueue + 2), Slot{});
            m_ready[cam].clear();
            m_free[cam].clear();
            for (size_t i = 0; i < m_slots[cam].size(); ++i) {
                m_slots[cam][i].camIndex = cam;
                m_free[cam].push_back(i);
            }
        }

        m_running.store(true);
        for (int cam = 0; cam < 2; ++cam) {
            m_th[cam] = std::thread([this, cam] { Loop(cam); });
        }
        return true;
    }

    void Close()
    {
        m_running.store(false);
        for (int cam = 0; cam < 2; ++cam) {
            m_cv[cam].notify_all();
        }
        for (int cam = 0; cam < 2; ++cam) {
            if (m_th[cam].joinable())
                m_th[cam].join();
        }
        if (m_sock >= 0) {
            ::close(m_sock);
            m_sock = -1;
        }
        {
            for (int cam = 0; cam < 2; ++cam) {
                std::lock_guard<std::mutex> lk(m_mu[cam]);
                m_ready[cam].clear();
                m_free[cam].clear();
                m_slots[cam].clear();
            }
        }
    }

    // called from SLAM thread (non-blocking-ish)
    void Enqueue(int camIndex,
                 uint32_t seq,
                 double frameTime,
                 const cv::Mat &gray,
                 const std::vector<cv::Point2f> &trackedPoints = {})
    {
        if (m_sock < 0 || camIndex < 0 || camIndex > 1)
            return;

        {
            std::lock_guard<std::mutex> lk(m_mu[camIndex]);
            const double lastFrameTime = m_lastAcceptedFrameTime[camIndex];
            if (lastFrameTime >= 0.0) {
                const double dt = frameTime - lastFrameTime;
                if (dt >= 0.0 && dt < MIN_FRAME_INTERVAL_SEC) {
                    return;
                }
            }
            m_lastAcceptedFrameTime[camIndex] = frameTime;

            if (m_free[camIndex].empty()) {
                if (m_ready[camIndex].empty()) {
                    return;
                }
                const size_t staleSlot = m_ready[camIndex].front();
                m_ready[camIndex].pop_front();
                m_free[camIndex].push_back(staleSlot);
            }

            const size_t slotIndex = m_free[camIndex].front();
            m_free[camIndex].pop_front();
            Slot& slot = m_slots[camIndex][slotIndex];
            slot.seq = seq;
            slot.frameTime = frameTime;
            if (!FillPreview(gray, slot.preview)) {
                m_free[camIndex].push_back(slotIndex);
                return;
            }
            slot.trackedPoints = trackedPoints;
            m_ready[camIndex].push_back(slotIndex);
            while ((int)m_ready[camIndex].size() > m_maxQueue) {
                const size_t staleSlot = m_ready[camIndex].front();
                m_ready[camIndex].pop_front();
                m_free[camIndex].push_back(staleSlot);
            }
        }
        m_cv[camIndex].notify_one();
    }

  private:
#pragma pack(push, 1)
    struct PacketHeader {
        uint32_t magic;      // 'VSIM' 0x5643494D (or any)
        uint16_t version;    // 1
        uint8_t camIndex;   // 0/1
        uint8_t flags;       // reserved
        uint32_t seq;        // camera sequence
        double frameTime;      // seconds
        uint32_t frameId;   // incremental id for this sender
        uint16_t chunkIdx;  // 0..chunkCnt-1
        uint16_t chunkCnt;  // total chunks
        uint32_t totalSize; // jpeg bytes total
        uint32_t chunkSize; // bytes in this packet payload
    };
#pragma pack(pop)

    void Loop(int camIndex)
    {
        std::vector<uchar> jpeg;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, m_jpegQuality};
        auto lastLog = std::chrono::steady_clock::now();
        int encodedFrames = 0;
        int sentPackets = 0;
        int featurePackets = 0;

        while (m_running.load()) {
            size_t slotIndex = 0;
            {
                std::unique_lock<std::mutex> lk(m_mu[camIndex]);
                m_cv[camIndex].wait(lk, [&] { return !m_running.load() || !m_ready[camIndex].empty(); });
                if (!m_running.load())
                    break;
                slotIndex = m_ready[camIndex].front();
                m_ready[camIndex].pop_front();
            }
            Slot& slot = m_slots[camIndex][slotIndex];

            // encode
            jpeg.clear();
            if (slot.preview.empty()) {
                std::lock_guard<std::mutex> lk(m_mu[camIndex]);
                m_free[camIndex].push_back(slotIndex);
                continue;
            }
            try {
                cv::imencode(".jpg", slot.preview, jpeg, params);
            } catch (const std::exception &e) {
                std::cerr << "[udp] imencode exception: " << e.what() << "\n";
                std::lock_guard<std::mutex> lk(m_mu[camIndex]);
                m_free[camIndex].push_back(slotIndex);
                continue;
            }
            if (jpeg.empty()) {
                std::cerr << "[udp] empty jpeg cam=" << camIndex
                          << " type=" << slot.preview.type()
                          << " rows=" << slot.preview.rows
                          << " cols=" << slot.preview.cols
                          << "\n";
                std::lock_guard<std::mutex> lk(m_mu[camIndex]);
                m_free[camIndex].push_back(slotIndex);
                continue;
            }

            const uint32_t total = (uint32_t)jpeg.size();
            const uint16_t chunks = (uint16_t)((total + m_maxPayload - 1) / m_maxPayload);
            const uint32_t fid = m_frameId.fetch_add(1, std::memory_order_relaxed);
            encodedFrames++;

            for (uint16_t ci = 0; ci < chunks; ++ci) {
                const uint32_t off = (uint32_t)ci * (uint32_t)m_maxPayload;
                const uint32_t left = total - off;
                const uint32_t pay =
                    (left > (uint32_t)m_maxPayload) ? (uint32_t)m_maxPayload : left;

                PacketHeader h{};
                h.magic = 0x5643494D; // 'VCIM' just a tag
                h.version = 1;
                h.camIndex = (uint8_t)slot.camIndex;
                h.flags = 0;
                h.seq = slot.seq;
                h.frameTime = slot.frameTime;
                h.frameId = fid;
                h.chunkIdx = ci;
                h.chunkCnt = chunks;
                h.totalSize = total;
                h.chunkSize = pay;

                iovec iov[2]{};
                iov[0].iov_base = &h;
                iov[0].iov_len = sizeof(h);
                iov[1].iov_base = jpeg.data() + off;
                iov[1].iov_len = pay;

                msghdr msg{};
                msg.msg_name = &m_dst;
                msg.msg_namelen = sizeof(m_dst);
                msg.msg_iov = iov;
                msg.msg_iovlen = 2;

                ssize_t sent = ::sendmsg(m_sock, &msg, 0);
                (void)sent; // ignore drop; UDP is best-effort
                sentPackets++;
            }

            if (!slot.trackedPoints.empty()) {
                SendFeaturePacket(slot, fid, slot.preview.cols, slot.preview.rows, slot.trackedPoints);
                featurePackets++;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastLog >= std::chrono::seconds(1)) {
                std::cerr << "[udp] cam=" << camIndex
                          << " encoded=" << encodedFrames
                          << " packets=" << sentPackets
                          << " featurePkt=" << featurePackets
                          << " last_jpeg=" << total
                          << " preview=" << slot.preview.cols << "x" << slot.preview.rows
                          << " previewType=" << slot.preview.type()
                          << "\n";
                encodedFrames = 0;
                sentPackets = 0;
                featurePackets = 0;
                lastLog = now;
            }

            std::lock_guard<std::mutex> lk(m_mu[camIndex]);
            m_free[camIndex].push_back(slotIndex);
        }
    }

    void SendFeaturePacket(Slot &slot,
                           uint32_t frameId,
                           int width,
                           int height,
                           const std::vector<cv::Point2f> &trackedPoints)
    {
        if (m_sock < 0 || width <= 0 || height <= 0 || trackedPoints.empty()) {
            return;
        }

        const size_t sendCount = std::min<size_t>(trackedPoints.size(), 160);
        const size_t payloadSize = 6 + sendCount * 4;
        slot.featureBuf.resize(payloadSize);
        uint8_t* payload = slot.featureBuf.data();

        WriteU16Le(payload, 0, static_cast<uint16_t>(std::min(width, 0xFFFF)));
        WriteU16Le(payload, 2, static_cast<uint16_t>(std::min(height, 0xFFFF)));
        WriteU16Le(payload, 4, static_cast<uint16_t>(sendCount));
        for (size_t i = 0; i < sendCount; ++i) {
            const cv::Point2f &pt = trackedPoints[i];
            const int xi = std::clamp<int>(static_cast<int>(std::lround(pt.x)), 0, width - 1);
            const int yi = std::clamp<int>(static_cast<int>(std::lround(pt.y)), 0, height - 1);
            const size_t pointOffset = 6 + i * 4;
            WriteU16Le(payload, pointOffset, static_cast<uint16_t>(xi));
            WriteU16Le(payload, pointOffset + 2, static_cast<uint16_t>(yi));
        }

        PacketHeader h{};
        h.magic = 0x5643494D;
        h.version = 1;
        h.camIndex = static_cast<uint8_t>(slot.camIndex);
        h.flags = FLAG_FEATURE_POINTS;
        h.seq = slot.seq;
        h.frameTime = slot.frameTime;
        h.frameId = frameId;
        h.chunkIdx = 0;
        h.chunkCnt = 1;
        h.totalSize = static_cast<uint32_t>(payloadSize);
        h.chunkSize = static_cast<uint32_t>(payloadSize);

        iovec iov[2]{};
        iov[0].iov_base = &h;
        iov[0].iov_len = sizeof(h);
        iov[1].iov_base = slot.featureBuf.data();
        iov[1].iov_len = slot.featureBuf.size();

        msghdr msg{};
        msg.msg_name = &m_dst;
        msg.msg_namelen = sizeof(m_dst);
        msg.msg_iov = iov;
        msg.msg_iovlen = 2;
        ::sendmsg(m_sock, &msg, 0);
    }

    static bool FillPreview(const cv::Mat& gray, cv::Mat& preview)
    {
        if (gray.empty()) {
            preview.release();
            return false;
        }

        if (gray.type() == CV_8UC1) {
            preview.create(gray.rows, gray.cols, CV_8UC1);
            gray.copyTo(preview);
        } else if (gray.type() == CV_16UC1) {
            preview.create(gray.rows, gray.cols, CV_8UC1);
            gray.convertTo(preview, CV_8U, 1.0 / 256.0);
        } else {
            preview.create(gray.rows, gray.cols, CV_8UC1);
            gray.convertTo(preview, CV_8U);
        }
        return true;
    }

    static void WriteU16Le(uint8_t* out, size_t offset, uint16_t value)
    {
        out[offset] = static_cast<uint8_t>(value & 0xFF);
        out[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

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

    std::atomic<uint32_t> m_frameId{1};
};
