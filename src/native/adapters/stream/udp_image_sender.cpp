#include "adapters/stream/udp_image_sender.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>

namespace {

struct PreviewCompressionState {
    bool initialized{false};
    double lo{0.0};
    double hi{4095.0};
    std::array<uint8_t, 65536> lut{};
};

struct PreviewRange {
    double lo{0.0};
    double hi{4095.0};
};

struct HistogramSample {
    std::array<uint32_t, 4096> hist{};
    uint32_t sampleCount{0};
};

std::vector<cv::Point2f> SelectGridSampledFeatures(const std::vector<cv::Point2f> &points, int width, int height,
                                                   size_t maxCount)
{
    if (points.empty() || width <= 0 || height <= 0 || maxCount == 0) {
        return {};
    }

    constexpr int kGridCols = 8;
    constexpr int kGridRows = 6;
    constexpr size_t kPerCellCap = 4;
    constexpr size_t kCellCount = static_cast<size_t>(kGridCols * kGridRows);

    std::array<std::vector<size_t>, kCellCount> cells{};
    for (size_t i = 0; i < points.size(); ++i) {
        const int xi = std::clamp<int>(static_cast<int>(std::lround(points[i].x)), 0, width - 1);
        const int yi = std::clamp<int>(static_cast<int>(std::lround(points[i].y)), 0, height - 1);
        const int gx = std::min(kGridCols - 1, (xi * kGridCols) / width);
        const int gy = std::min(kGridRows - 1, (yi * kGridRows) / height);
        const size_t cellIdx = static_cast<size_t>(gy * kGridCols + gx);
        cells[cellIdx].push_back(i);
    }

    std::array<size_t, kCellCount> cursor{};
    std::array<size_t, kCellCount> used{};
    std::vector<cv::Point2f> selected;
    selected.reserve(std::min(maxCount, points.size()));

    bool madeProgress = true;
    while (selected.size() < maxCount && madeProgress) {
        madeProgress = false;
        for (size_t cellIdx = 0; cellIdx < kCellCount && selected.size() < maxCount; ++cellIdx) {
            auto &bucket = cells[cellIdx];
            if (bucket.empty() || used[cellIdx] >= kPerCellCap || cursor[cellIdx] >= bucket.size()) {
                continue;
            }
            selected.push_back(points[bucket[cursor[cellIdx]++]]);
            used[cellIdx]++;
            madeProgress = true;
        }
    }

    return selected;
}

HistogramSample BuildPreviewHistogram(const cv::Mat &src16)
{
    HistogramSample sample{};
    for (int y = 0; y < src16.rows; y += 2) {
        const uint16_t *row = src16.ptr<uint16_t>(y);
        for (int x = 0; x < src16.cols; x += 2) {
            const uint16_t bin = static_cast<uint16_t>(row[x] >> 4);
            sample.hist[std::min<size_t>(bin, sample.hist.size() - 1)]++;
            ++sample.sampleCount;
        }
    }
    return sample;
}

PreviewRange EstimatePreviewRange(const HistogramSample &sample)
{
    const uint32_t lowTarget = static_cast<uint32_t>(sample.sampleCount * 0.01);
    const uint32_t highTarget = static_cast<uint32_t>(sample.sampleCount * 0.99);
    uint32_t cumulative = 0;
    int lowBin = 0;
    int highBin = static_cast<int>(sample.hist.size() - 1);
    bool lowFound = false;
    for (size_t i = 0; i < sample.hist.size(); ++i) {
        cumulative += sample.hist[i];
        if (!lowFound && cumulative >= lowTarget) {
            lowBin = static_cast<int>(i);
            lowFound = true;
        }
        if (cumulative >= highTarget) {
            highBin = static_cast<int>(i);
            break;
        }
    }

    PreviewRange range{static_cast<double>(lowBin << 4), static_cast<double>(highBin << 4)};
    if (range.hi <= range.lo + 64.0) {
        range.hi = range.lo + 64.0;
    }
    return range;
}

void SmoothPreviewRange(PreviewCompressionState &state, const PreviewRange &target)
{
    if (!state.initialized) {
        state.lo = target.lo;
        state.hi = target.hi;
        state.initialized = true;
    } else {
        state.lo = state.lo * 0.85 + target.lo * 0.15;
        state.hi = state.hi * 0.85 + target.hi * 0.15;
    }
    if (state.hi <= state.lo + 64.0) {
        state.hi = state.lo + 64.0;
    }
}

void RebuildPreviewLut(PreviewCompressionState &state)
{
    const double invRange = 255.0 / (state.hi - state.lo);
    for (int i = 0; i < 65536; ++i) {
        const double v = (static_cast<double>(i) - state.lo) * invRange;
        const int out = (v <= 0.0) ? 0 : (v >= 255.0) ? 255
                                                      : static_cast<int>(v + 0.5);
        state.lut[static_cast<size_t>(i)] = static_cast<uint8_t>(out);
    }
}

void ApplyPreviewLut(const cv::Mat &src16, const PreviewCompressionState &state, cv::Mat &dst8)
{
    dst8.create(src16.rows, src16.cols, CV_8UC1);
    for (int y = 0; y < src16.rows; ++y) {
        const uint16_t *srcRow = src16.ptr<uint16_t>(y);
        uint8_t *dstRow = dst8.ptr<uint8_t>(y);
        for (int x = 0; x < src16.cols; ++x) {
            dstRow[x] = state.lut[srcRow[x]];
        }
    }
}

void Compress16To8Adaptive(const cv::Mat &src16, cv::Mat &dst8, int camIndex, uint32_t seq)
{
    static std::array<PreviewCompressionState, 2> states{};
    const size_t idx = static_cast<size_t>((camIndex == 1) ? 1 : 0);
    PreviewCompressionState &state = states[idx];

    const HistogramSample sample = BuildPreviewHistogram(src16);
    if (sample.sampleCount == 0) {
        dst8.create(src16.rows, src16.cols, CV_8UC1);
        dst8.setTo(0);
        return;
    }

    SmoothPreviewRange(state, EstimatePreviewRange(sample));
    RebuildPreviewLut(state);
    ApplyPreviewLut(src16, state, dst8);

    if ((seq % 120u) == 0u) {
        std::cerr << "[udp_preview_r16] cam=" << camIndex << " seq=" << seq << " lo=" << state.lo
                  << " hi=" << state.hi << " samples=" << sample.sampleCount << "\n";
    }
}

} // namespace

bool UdpImageSender::Open(const std::string &ip, int port, int jpegQuality, int maxPayload, int maxQueue,
                          DestinationResolver destinationResolver)
{
    return Open(OpenConfig{ip, port, jpegQuality, maxPayload, maxQueue, std::move(destinationResolver)});
}

bool UdpImageSender::Open(OpenConfig config)
{
    m_jpegQuality = std::max(10, std::min(95, config.jpegQuality));
    m_maxPayload = std::max(400, config.maxPayload);
    m_maxQueue = std::max(1, config.maxQueue);
    m_port = config.port;

    m_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sock < 0) {
        std::cerr << "[udp] socket() failed: " << strerror(errno) << "\n";
        return false;
    }

    memset(&m_dst, 0, sizeof(m_dst));
    m_dst.sin_family = AF_INET;
    m_dst.sin_port = htons(static_cast<uint16_t>(config.port));
    if (::inet_pton(AF_INET, config.ip.c_str(), &m_dst.sin_addr) != 1) {
        std::cerr << "[udp] inet_pton failed for " << config.ip << "\n";
        ::close(m_sock);
        m_sock = -1;
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(m_dstMu);
        m_destinationResolver = std::move(config.destinationResolver);
        m_lastResolvedIp = 0;
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
        m_lastAcceptedFrameTime[cam] = -1.0;
    }

    return true;
}

sockaddr_in UdpImageSender::ResolveDestination() const
{
    sockaddr_in dst{};
    {
        std::lock_guard<std::mutex> lk(m_dstMu);
        dst = m_dst;
    }

    sockaddr_in dynamicDst{};
    DestinationResolver resolver;
    {
        std::lock_guard<std::mutex> lk(m_dstMu);
        resolver = m_destinationResolver;
    }
    if (!resolver || !resolver(dynamicDst)) {
        return dst;
    }

    dynamicDst.sin_family = AF_INET;
    dynamicDst.sin_port = htons(static_cast<uint16_t>(m_port));
    const uint32_t resolvedIp = dynamicDst.sin_addr.s_addr;
    uint32_t previousIp = 0;
    {
        std::lock_guard<std::mutex> lk(m_dstMu);
        previousIp = m_lastResolvedIp;
        m_lastResolvedIp = resolvedIp;
    }
    if (resolvedIp != 0 && resolvedIp != previousIp) {
        char ipBuf[INET_ADDRSTRLEN]{};
        const char *resolvedText = ::inet_ntop(AF_INET, &dynamicDst.sin_addr, ipBuf, sizeof(ipBuf));
        if (resolvedText != nullptr) {
            std::cerr << "[udp] destination peer -> " << resolvedText << ":" << m_port << "\n";
        }
    }
    return dynamicDst;
}

void UdpImageSender::Close()
{
    if (m_sock >= 0) {
        ::close(m_sock);
        m_sock = -1;
    }
    for (int cam = 0; cam < 2; ++cam) {
        std::lock_guard<std::mutex> lk(m_mu[cam]);
        m_ready[cam].clear();
        m_free[cam].clear();
        m_slots[cam].clear();
        m_lastAcceptedFrameTime[cam] = -1.0;
    }
}

void UdpImageSender::Enqueue(int camIndex, uint64_t frameId, uint32_t seq, double frameTime, const cv::Mat &gray,
                             const std::vector<cv::Point2f> &trackedPoints, bool sendImage, bool sendFeature)
{
    Enqueue(EnqueueRequest{camIndex, frameId, seq, frameTime, gray, trackedPoints, sendImage, sendFeature});
}

void UdpImageSender::Enqueue(const EnqueueRequest &request)
{
    if (m_sock < 0 || request.camIndex < 0 || request.camIndex > 1) {
        return;
    }
    if (!request.sendImage && !request.sendFeature) {
        return;
    }

    std::lock_guard<std::mutex> lk(m_mu[request.camIndex]);
    if (!AcceptFrameTime(request.camIndex, request.frameTime) || !PrepareFreeSlot(request.camIndex)) {
        return;
    }

    const size_t slotIndex = m_free[request.camIndex].front();
    m_free[request.camIndex].pop_front();
    Slot &slot = m_slots[request.camIndex][slotIndex];
    PopulateSlot(slot, request);
    if (request.sendImage && !FillPreview(request.camIndex, request.seq, request.gray, slot.preview)) {
        m_free[request.camIndex].push_back(slotIndex);
        return;
    }
    if (!request.sendImage) {
        slot.preview.release();
    }
    m_ready[request.camIndex].push_back(slotIndex);
    TrimReadyQueue(request.camIndex);
}

bool UdpImageSender::AcceptFrameTime(int camIndex, double frameTime)
{
    const double lastFrameTime = m_lastAcceptedFrameTime[camIndex];
    if (lastFrameTime >= 0.0) {
        const double dt = frameTime - lastFrameTime;
        if (dt >= 0.0 && dt < MIN_FRAME_INTERVAL_SEC) {
            return false;
        }
    }
    m_lastAcceptedFrameTime[camIndex] = frameTime;
    return true;
}

bool UdpImageSender::PrepareFreeSlot(int camIndex)
{
    if (!m_free[camIndex].empty()) {
        return true;
    }
    if (m_ready[camIndex].empty()) {
        return false;
    }
    const size_t staleSlot = m_ready[camIndex].front();
    m_ready[camIndex].pop_front();
    m_free[camIndex].push_back(staleSlot);
    return true;
}

void UdpImageSender::PopulateSlot(Slot &slot, const EnqueueRequest &request)
{
    slot.frameId = request.frameId;
    slot.seq = request.seq;
    slot.frameTime = request.frameTime;
    slot.width = request.gray.cols;
    slot.height = request.gray.rows;
    slot.sendImage = request.sendImage;
    slot.sendFeature = request.sendFeature;
    slot.trackedPoints = request.sendFeature ? request.trackedPoints : std::vector<cv::Point2f>{};
}

void UdpImageSender::TrimReadyQueue(int camIndex)
{
    while (static_cast<int>(m_ready[camIndex].size()) > m_maxQueue) {
        const size_t staleSlot = m_ready[camIndex].front();
        m_ready[camIndex].pop_front();
        m_free[camIndex].push_back(staleSlot);
    }
}

void UdpImageSender::StepAll()
{
    StepCamera(0);
    StepCamera(1);
}

void UdpImageSender::StepOnce()
{
    StepCameraOnce(0);
    StepCameraOnce(1);
}

void UdpImageSender::StepCamera(int camIndex)
{
    if (m_sock < 0 || camIndex < 0 || camIndex > 1) {
        return;
    }
    while (StepCameraOnce(camIndex)) {
    }
}

bool UdpImageSender::StepCameraOnce(int camIndex)
{
    if (m_sock < 0 || camIndex < 0 || camIndex > 1) {
        return false;
    }
    Slot slot{};
    if (!PopReadySlot(camIndex, slot)) {
        return false;
    }
    SendSlot(slot);
    return true;
}

bool UdpImageSender::PopReadySlot(int camIndex, Slot &slot)
{
    std::lock_guard<std::mutex> lk(m_mu[camIndex]);
    if (m_ready[camIndex].empty()) {
        return false;
    }

    const size_t slotIndex = m_ready[camIndex].front();
    m_ready[camIndex].pop_front();
    slot = m_slots[camIndex][slotIndex];
    m_slots[camIndex][slotIndex].preview.release();
    m_slots[camIndex][slotIndex].trackedPoints.clear();
    m_slots[camIndex][slotIndex].featureBuf.clear();
    m_free[camIndex].push_back(slotIndex);
    return true;
}

void UdpImageSender::SendSlot(Slot &slot)
{
    if (slot.sendImage) {
        std::vector<uchar> jpeg;
        if (!EncodePreviewJpeg(slot, jpeg)) {
            return;
        }
        SendImagePackets(slot, jpeg);
    }

    if (slot.sendFeature && !slot.trackedPoints.empty()) {
        SendFeaturePacket(slot, static_cast<uint32_t>(slot.frameId & 0xFFFFFFFFu), slot.width, slot.height,
                          slot.trackedPoints);
    }
}

bool UdpImageSender::EncodePreviewJpeg(const Slot &slot, std::vector<uchar> &jpeg) const
{
    if (slot.preview.empty()) {
        return false;
    }

    try {
        const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, m_jpegQuality};
        cv::imencode(".jpg", slot.preview, jpeg, params);
    } catch (const std::exception &e) {
        std::cerr << "[udp] imencode exception: " << e.what() << "\n";
        return false;
    }
    if (!jpeg.empty()) {
        return true;
    }

    std::cerr << "[udp] empty jpeg cam=" << slot.camIndex << " type=" << slot.preview.type()
              << " rows=" << slot.preview.rows << " cols=" << slot.preview.cols << "\n";
    return false;
}

void UdpImageSender::SendImagePackets(const Slot &slot, const std::vector<uchar> &jpeg)
{
    const uint32_t total = static_cast<uint32_t>(jpeg.size());
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

        SendPacket(h, jpeg.data() + off, pay);
    }
}

void UdpImageSender::SendPacket(const PacketHeader &header, const uint8_t *payload, size_t payloadSize)
{
    PacketHeader packetHeader = header;
    iovec iov[2]{};
    iov[0].iov_base = &packetHeader;
    iov[0].iov_len = sizeof(packetHeader);
    iov[1].iov_base = const_cast<uint8_t *>(payload);
    iov[1].iov_len = payloadSize;

    const sockaddr_in dst = ResolveDestination();
    msghdr msg{};
    msg.msg_name = const_cast<sockaddr_in *>(&dst);
    msg.msg_namelen = sizeof(dst);
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;
    const ssize_t sent = ::sendmsg(m_sock, &msg, 0);
    (void)sent;
}

void UdpImageSender::SendFeaturePacket(Slot &slot, uint32_t frameId, int width, int height,
                                       const std::vector<cv::Point2f> &trackedPoints)
{
    if (m_sock < 0 || width <= 0 || height <= 0 || trackedPoints.empty()) {
        return;
    }
    if (!BuildFeaturePayload(slot.featureBuf, width, height, trackedPoints)) {
        return;
    }

    const size_t payloadSize = slot.featureBuf.size();
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
    SendPacket(h, slot.featureBuf.data(), payloadSize);
}

bool UdpImageSender::BuildFeaturePayload(std::vector<uint8_t> &payloadBuf, int width, int height,
                                         const std::vector<cv::Point2f> &trackedPoints) const
{
    const std::vector<cv::Point2f> sampled = SelectGridSampledFeatures(trackedPoints, width, height, 160);
    const size_t sendCount = sampled.size();
    if (sendCount == 0) {
        return false;
    }
    const size_t payloadSize = 6 + sendCount * 4;
    payloadBuf.resize(payloadSize);
    uint8_t *payload = payloadBuf.data();

    WriteU16Le(payload, 0, static_cast<uint16_t>(std::min(width, 0xFFFF)));
    WriteU16Le(payload, 2, static_cast<uint16_t>(std::min(height, 0xFFFF)));
    WriteU16Le(payload, 4, static_cast<uint16_t>(sendCount));
    for (size_t i = 0; i < sendCount; ++i) {
        const cv::Point2f &pt = sampled[i];
        const int xi = std::clamp<int>(static_cast<int>(std::lround(pt.x)), 0, width - 1);
        const int yi = std::clamp<int>(static_cast<int>(std::lround(pt.y)), 0, height - 1);
        const size_t pointOffset = 6 + i * 4;
        WriteU16Le(payload, pointOffset, static_cast<uint16_t>(xi));
        WriteU16Le(payload, pointOffset + 2, static_cast<uint16_t>(yi));
    }
    return true;
}

bool UdpImageSender::FillPreview(int camIndex, uint32_t seq, const cv::Mat &gray, cv::Mat &preview)
{
    if (gray.empty()) {
        preview.release();
        return false;
    }

    if (gray.type() == CV_8UC1) {
        preview.create(gray.rows, gray.cols, CV_8UC1);
        gray.copyTo(preview);
    } else if (gray.type() == CV_16UC1) {
        Compress16To8Adaptive(gray, preview, camIndex, seq);
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
