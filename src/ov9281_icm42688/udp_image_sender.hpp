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
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
// ---------------- UDP image sender ----------------
class UdpImageSender {
  public:
    static constexpr uint8_t FLAG_FEATURE_POINTS = 0x01;

    struct Job {
        int camIndex;  // 0=L, 1=R
        uint32_t seq;   // sequence for debug
        double frameTime; // seconds
        cv::Mat gray;   // preview source, may be CV_8UC1 or CV_16UC1
        std::vector<cv::Point2f> trackedPoints;
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
                m_q[cam].clear();
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
        Job job;
        job.camIndex = camIndex;
        job.seq = seq;
        job.frameTime = frameTime;
        job.gray = gray.clone(); // IMPORTANT: own data (libcamera buffer will be reused)
        job.trackedPoints = trackedPoints;

        {
            std::lock_guard<std::mutex> lk(m_mu[camIndex]);
            m_q[camIndex].push_back(std::move(job));
            while ((int)m_q[camIndex].size() > m_maxQueue)
                m_q[camIndex].pop_front(); // drop old to keep real-time
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
        cv::Mat preview8;
        auto lastLog = std::chrono::steady_clock::now();
        int encodedFrames = 0;
        int sentPackets = 0;
        int featurePackets = 0;

        while (m_running.load()) {
            Job job;
            {
                std::unique_lock<std::mutex> lk(m_mu[camIndex]);
                m_cv[camIndex].wait(lk, [&] { return !m_running.load() || !m_q[camIndex].empty(); });
                if (!m_running.load())
                    break;
                job = std::move(m_q[camIndex].front());
                m_q[camIndex].pop_front();
            }

            // encode
            jpeg.clear();
            preview8.release();
            if (job.gray.empty()) {
                continue;
            }
            if (job.gray.type() == CV_8UC1) {
                preview8 = job.gray;
            } else if (job.gray.type() == CV_16UC1) {
                double minVal = 0.0;
                double maxVal = 0.0;
                cv::minMaxLoc(job.gray, &minVal, &maxVal);
                const double scale = (maxVal > minVal) ? (255.0 / (maxVal - minVal)) : (1.0 / 256.0);
                const double shift = (maxVal > minVal) ? (-minVal * scale) : 0.0;
                job.gray.convertTo(preview8, CV_8U, scale, shift);
            } else {
                job.gray.convertTo(preview8, CV_8U);
            }
            try {
                cv::imencode(".jpg", preview8, jpeg, params);
            } catch (const std::exception &e) {
                std::cerr << "[udp] imencode exception: " << e.what() << "\n";
                continue;
            }
            if (jpeg.empty()) {
                std::cerr << "[udp] empty jpeg cam=" << camIndex
                          << " type=" << job.gray.type()
                          << " rows=" << job.gray.rows
                          << " cols=" << job.gray.cols
                          << "\n";
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
                h.camIndex = (uint8_t)job.camIndex;
                h.flags = 0;
                h.seq = job.seq;
                h.frameTime = job.frameTime;
                h.frameId = fid;
                h.chunkIdx = ci;
                h.chunkCnt = chunks;
                h.totalSize = total;
                h.chunkSize = pay;

                // build packet buffer: header + payload
                std::vector<uint8_t> pkt(sizeof(PacketHeader) + pay);
                memcpy(pkt.data(), &h, sizeof(PacketHeader));
                memcpy(pkt.data() + sizeof(PacketHeader), jpeg.data() + off, pay);

                ssize_t sent =
                    ::sendto(m_sock, pkt.data(), pkt.size(), 0, (sockaddr *)&m_dst, sizeof(m_dst));
                (void)sent; // ignore drop; UDP is best-effort
                sentPackets++;
            }

            if (!job.trackedPoints.empty()) {
                SendFeaturePacket(job, fid, preview8.cols, preview8.rows, job.trackedPoints);
                featurePackets++;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastLog >= std::chrono::seconds(1)) {
                std::cerr << "[udp] cam=" << camIndex
                          << " encoded=" << encodedFrames
                          << " packets=" << sentPackets
                          << " featurePkt=" << featurePackets
                          << " last_jpeg=" << total
                          << " srcType=" << job.gray.type()
                          << " previewType=" << preview8.type()
                          << "\n";
                encodedFrames = 0;
                sentPackets = 0;
                featurePackets = 0;
                lastLog = now;
            }
        }
    }

    void SendFeaturePacket(const Job &job,
                           uint32_t frameId,
                           int width,
                           int height,
                           const std::vector<cv::Point2f> &trackedPoints)
    {
        if (m_sock < 0 || width <= 0 || height <= 0 || trackedPoints.empty()) {
            return;
        }

        const size_t sendCount = std::min<size_t>(trackedPoints.size(), 160);
        std::vector<uint8_t> payload;
        payload.reserve(6 + sendCount * 4);
        WriteU16Le(payload, static_cast<uint16_t>(std::min(width, 0xFFFF)));
        WriteU16Le(payload, static_cast<uint16_t>(std::min(height, 0xFFFF)));
        WriteU16Le(payload, static_cast<uint16_t>(sendCount));
        for (size_t i = 0; i < sendCount; ++i) {
            const cv::Point2f &pt = trackedPoints[i];
            const int xi = std::clamp<int>(static_cast<int>(std::lround(pt.x)), 0, width - 1);
            const int yi = std::clamp<int>(static_cast<int>(std::lround(pt.y)), 0, height - 1);
            WriteU16Le(payload, static_cast<uint16_t>(xi));
            WriteU16Le(payload, static_cast<uint16_t>(yi));
        }

        PacketHeader h{};
        h.magic = 0x5643494D;
        h.version = 1;
        h.camIndex = static_cast<uint8_t>(job.camIndex);
        h.flags = FLAG_FEATURE_POINTS;
        h.seq = job.seq;
        h.frameTime = job.frameTime;
        h.frameId = frameId;
        h.chunkIdx = 0;
        h.chunkCnt = 1;
        h.totalSize = static_cast<uint32_t>(payload.size());
        h.chunkSize = static_cast<uint32_t>(payload.size());

        std::vector<uint8_t> pkt(sizeof(PacketHeader) + payload.size());
        std::memcpy(pkt.data(), &h, sizeof(PacketHeader));
        std::memcpy(pkt.data() + sizeof(PacketHeader), payload.data(), payload.size());
        ::sendto(m_sock, pkt.data(), pkt.size(), 0, reinterpret_cast<sockaddr *>(&m_dst), sizeof(m_dst));
    }

    static void WriteU16Le(std::vector<uint8_t> &out, uint16_t value)
    {
        out.push_back(static_cast<uint8_t>(value & 0xFF));
        out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
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
    std::deque<Job> m_q[2];

    std::atomic<uint32_t> m_frameId{1};
};
