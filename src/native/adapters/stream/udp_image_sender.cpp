#include "adapters/stream/udp_image_sender.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>

#include "common/thread_launch.h"

bool UdpImageSender::Open(const std::string &ip, int port, int jpegQuality, int maxPayload, int maxQueue)
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
    m_dst.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &m_dst.sin_addr) != 1) {
        std::cerr << "[udp] inet_pton failed for " << ip << "\n";
        ::close(m_sock);
        m_sock = -1;
        return false;
    }
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
        const auto role =
            cam == 0 ? smartdrone::common::ThreadRole::UdpImageCam0 : smartdrone::common::ThreadRole::UdpImageCam1;
        m_th[cam] = SMARTDRONE_START_THREAD(role, "UdpImageSender", [this, cam] { Loop(cam); });
    }
    return true;
}

void UdpImageSender::Close()
{
    m_running.store(false);
    for (int cam = 0; cam < 2; ++cam) {
        m_cv[cam].notify_all();
    }
    for (int cam = 0; cam < 2; ++cam) {
        if (m_th[cam].joinable()) {
            m_th[cam].join();
        }
    }
    if (m_sock >= 0) {
        ::close(m_sock);
        m_sock = -1;
    }
    for (int cam = 0; cam < 2; ++cam) {
        std::lock_guard<std::mutex> lk(m_mu[cam]);
        m_ready[cam].clear();
        m_free[cam].clear();
        m_slots[cam].clear();
    }
}

void UdpImageSender::Enqueue(int camIndex, uint64_t frameId, uint32_t seq, double frameTime, const cv::Mat &gray,
                             const std::vector<cv::Point2f> &trackedPoints, bool sendImage, bool sendFeature)
{
    if (m_sock < 0 || camIndex < 0 || camIndex > 1) {
        return;
    }
    if (!sendImage && !sendFeature) {
        return;
    }

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
        Slot &slot = m_slots[camIndex][slotIndex];
        slot.frameId = frameId;
        slot.seq = seq;
        slot.frameTime = frameTime;
        slot.width = gray.cols;
        slot.height = gray.rows;
        slot.sendImage = sendImage;
        slot.sendFeature = sendFeature;
        if (sendImage && !FillPreview(gray, slot.preview)) {
            m_free[camIndex].push_back(slotIndex);
            return;
        }
        if (!sendImage) {
            slot.preview.release();
        }
        slot.trackedPoints = sendFeature ? trackedPoints : std::vector<cv::Point2f>{};
        m_ready[camIndex].push_back(slotIndex);
        while (static_cast<int>(m_ready[camIndex].size()) > m_maxQueue) {
            const size_t staleSlot = m_ready[camIndex].front();
            m_ready[camIndex].pop_front();
            m_free[camIndex].push_back(staleSlot);
        }
    }
    m_cv[camIndex].notify_one();
}

void UdpImageSender::Loop(int camIndex)
{
    std::vector<uchar> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, m_jpegQuality};
    while (m_running.load()) {
        size_t slotIndex = 0;
        {
            std::unique_lock<std::mutex> lk(m_mu[camIndex]);
            m_cv[camIndex].wait(lk, [&] { return !m_running.load() || !m_ready[camIndex].empty(); });
            if (!m_running.load()) {
                break;
            }
            slotIndex = m_ready[camIndex].front();
            m_ready[camIndex].pop_front();
        }
        Slot &slot = m_slots[camIndex][slotIndex];

        jpeg.clear();
        uint32_t total = 0;
        if (slot.sendImage) {
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
                std::cerr << "[udp] empty jpeg cam=" << camIndex << " type=" << slot.preview.type()
                          << " rows=" << slot.preview.rows << " cols=" << slot.preview.cols << "\n";
                std::lock_guard<std::mutex> lk(m_mu[camIndex]);
                m_free[camIndex].push_back(slotIndex);
                continue;
            }

            total = static_cast<uint32_t>(jpeg.size());
            const uint16_t chunks = static_cast<uint16_t>((total + m_maxPayload - 1) / m_maxPayload);
            for (uint16_t ci = 0; ci < chunks; ++ci) {
                const uint32_t off = static_cast<uint32_t>(ci) * static_cast<uint32_t>(m_maxPayload);
                const uint32_t left = total - off;
                const uint32_t pay =
                    (left > static_cast<uint32_t>(m_maxPayload)) ? static_cast<uint32_t>(m_maxPayload) : left;

                PacketHeader h{};
                h.magic = 0x5643494D;
                h.version = 1;
                h.camIndex = static_cast<uint8_t>(slot.camIndex);
                h.flags = 0;
                h.seq = slot.seq;
                h.frameTime = slot.frameTime;
                h.frameId = static_cast<uint32_t>(slot.frameId & 0xFFFFFFFFu);
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

                const ssize_t sent = ::sendmsg(m_sock, &msg, 0);
                (void)sent;
            }
        }

        if (slot.sendFeature && !slot.trackedPoints.empty()) {
            SendFeaturePacket(slot, static_cast<uint32_t>(slot.frameId & 0xFFFFFFFFu), slot.width, slot.height,
                              slot.trackedPoints);
        }

        std::lock_guard<std::mutex> lk(m_mu[camIndex]);
        m_free[camIndex].push_back(slotIndex);
    }
}

void UdpImageSender::SendFeaturePacket(Slot &slot, uint32_t frameId, int width, int height,
                                       const std::vector<cv::Point2f> &trackedPoints)
{
    if (m_sock < 0 || width <= 0 || height <= 0 || trackedPoints.empty()) {
        return;
    }

    const size_t sendCount = std::min<size_t>(trackedPoints.size(), 160);
    const size_t payloadSize = 6 + sendCount * 4;
    slot.featureBuf.resize(payloadSize);
    uint8_t *payload = slot.featureBuf.data();

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

bool UdpImageSender::FillPreview(const cv::Mat &gray, cv::Mat &preview)
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

void UdpImageSender::WriteU16Le(uint8_t *out, size_t offset, uint16_t value)
{
    out[offset] = static_cast<uint8_t>(value & 0xFF);
    out[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}
