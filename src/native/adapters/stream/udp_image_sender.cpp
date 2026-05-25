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

    constexpr int gridCols = 8;
    constexpr int gridRows = 6;
    constexpr size_t perCellCap = 4;
    constexpr size_t cellCount = static_cast<size_t>(gridCols * gridRows);

    std::array<std::vector<size_t>, cellCount> cells{};
    for (size_t i = 0; i < points.size(); ++i) {
        const int xi = std::clamp<int>(static_cast<int>(std::lround(points[i].x)), 0, width - 1);
        const int yi = std::clamp<int>(static_cast<int>(std::lround(points[i].y)), 0, height - 1);
        const int gx = std::min(gridCols - 1, (xi * gridCols) / width);
        const int gy = std::min(gridRows - 1, (yi * gridRows) / height);
        const size_t cellIdx = static_cast<size_t>(gy * gridCols + gx);
        cells[cellIdx].push_back(i);
    }

    std::array<size_t, cellCount> cursor{};
    std::array<size_t, cellCount> used{};
    std::vector<cv::Point2f> selected;
    selected.reserve(std::min(maxCount, points.size()));

    bool madeProgress = true;
    while (selected.size() < maxCount && madeProgress) {
        madeProgress = false;
        for (size_t cellIdx = 0; cellIdx < cellCount && selected.size() < maxCount; ++cellIdx) {
            auto &bucket = cells[cellIdx];
            if (bucket.empty() || used[cellIdx] >= perCellCap || cursor[cellIdx] >= bucket.size()) {
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

bool UdpImageSender::Open(OpenConfig config)
{
    m_jpegQuality.store(std::max(10, std::min(95, config.jpegQuality)),
                        std::memory_order_relaxed);
    m_maxPayload.store(std::max(400, config.maxPayload),
                       std::memory_order_relaxed);
    const int queueDepth =
        std::clamp(config.maxQueue, 1, static_cast<int>(MAX_PENDING_SLOTS));
    m_queueDepth.store(queueDepth, std::memory_order_relaxed);

    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "[udp] socket() failed: " << strerror(errno) << "\n";
        return false;
    }

    DestinationState destination{};
    destination.dst.sin_family = AF_INET;
    destination.dst.sin_port = htons(static_cast<uint16_t>(config.port));
    destination.port = config.port;
    destination.destinationResolver = std::move(config.destinationResolver);
    if (::inet_pton(AF_INET, config.ip.c_str(), &destination.dst.sin_addr) !=
        1) {
        std::cerr << "[udp] inet_pton failed for " << config.ip << "\n";
        ::close(sock);
        return false;
    }
    auto destinationState =
        std::make_shared<const DestinationState>(std::move(destination));
    std::atomic_store_explicit(&m_destinationState,
                               std::move(destinationState),
                               std::memory_order_release);
    m_lastResolvedIp.store(0, std::memory_order_relaxed);
    for (int cam = 0; cam < 2; ++cam) {
        ResetCameraQueue(cam);
    }

    const int previousSock = m_sock.exchange(sock, std::memory_order_acq_rel);
    if (previousSock >= 0) {
        ::close(previousSock);
    }
    return true;
}

sockaddr_in UdpImageSender::ResolveDestination()
{
    const std::shared_ptr<const DestinationState> state =
        LoadDestinationState();
    if (!state) {
        return {};
    }

    sockaddr_in dynamicDst{};
    if (!state->destinationResolver ||
        !state->destinationResolver(dynamicDst)) {
        return state->dst;
    }

    dynamicDst.sin_family = AF_INET;
    dynamicDst.sin_port = htons(static_cast<uint16_t>(state->port));
    const uint32_t resolvedIp = dynamicDst.sin_addr.s_addr;
    const uint32_t previousIp =
        m_lastResolvedIp.exchange(resolvedIp, std::memory_order_acq_rel);
    if (resolvedIp != 0 && resolvedIp != previousIp) {
        char ipBuf[INET_ADDRSTRLEN]{};
        const char *resolvedText = ::inet_ntop(AF_INET, &dynamicDst.sin_addr, ipBuf, sizeof(ipBuf));
        if (resolvedText != nullptr) {
            std::cerr << "[udp] destination peer -> " << resolvedText << ":"
                      << state->port << "\n";
        }
    }
    return dynamicDst;
}

void UdpImageSender::Close()
{
    const int sock = m_sock.exchange(-1, std::memory_order_acq_rel);
    if (sock >= 0) {
        ::close(sock);
    }
    for (int cam = 0; cam < 2; ++cam) {
        ResetCameraQueue(cam);
    }
}

void UdpImageSender::Enqueue(const EnqueueRequest &request)
{
    if (SocketFd() < 0 || request.camIndex < 0 || request.camIndex > 1) {
        return;
    }
    if (!request.sendImage && !request.sendFeature) {
        return;
    }

    if (!AcceptFrameTime(request.camIndex, request.frameTime)) {
        return;
    }
    StorePendingSlot(request.camIndex, BuildSlot(request));
}

std::shared_ptr<const UdpImageSender::Slot> UdpImageSender::BuildSlot(
    const EnqueueRequest &request)
{
    auto slot = std::make_shared<Slot>();
    PopulateSlot(*slot, request);
    if (request.sendImage &&
        !FillPreview(request.camIndex, request.seq, request.gray,
                     slot->preview)) {
        return nullptr;
    }
    return slot;
}

bool UdpImageSender::AcceptFrameTime(int camIndex, double frameTime)
{
    double lastFrameTime =
        m_lastAcceptedFrameTime[camIndex].load(std::memory_order_acquire);
    while (true) {
        const double dt = frameTime - lastFrameTime;
        if (lastFrameTime >= 0.0 && dt >= 0.0 &&
            dt < MIN_FRAME_INTERVAL_SEC) {
            return false;
        }
        if (m_lastAcceptedFrameTime[camIndex].compare_exchange_weak(
                lastFrameTime, frameTime, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            return true;
        }
    }
}

void UdpImageSender::StorePendingSlot(
    int camIndex, std::shared_ptr<const Slot> slot)
{
    if (!slot) {
        return;
    }
    const std::size_t queueSeq =
        m_writeSeq[camIndex].fetch_add(1, std::memory_order_acq_rel);
    const std::size_t depth = PendingQueueDepth();
    const std::size_t minReadSeq = queueSeq >= depth
                                       ? queueSeq + 1 - depth
                                       : 0;
    std::size_t readSeq = m_readSeq[camIndex].load(std::memory_order_acquire);
    while (readSeq < minReadSeq &&
           !m_readSeq[camIndex].compare_exchange_weak(
               readSeq, minReadSeq, std::memory_order_acq_rel,
               std::memory_order_acquire)) {
    }
    auto pending = std::make_shared<const PendingSlot>(PendingSlot{queueSeq,
                                                                   slot});
    const std::size_t slotIndex = queueSeq % MAX_PENDING_SLOTS;
    std::atomic_store_explicit(&m_pendingSlots[camIndex][slotIndex],
                               std::move(pending), std::memory_order_release);
}

void UdpImageSender::ResetCameraQueue(int camIndex)
{
    const std::size_t writeSeq =
        m_writeSeq[camIndex].load(std::memory_order_acquire);
    m_readSeq[camIndex].store(writeSeq, std::memory_order_release);
    m_lastAcceptedFrameTime[camIndex].store(-1.0, std::memory_order_release);
    for (std::shared_ptr<const PendingSlot> &slot : m_pendingSlots[camIndex]) {
        std::atomic_store_explicit(&slot,
                                   std::shared_ptr<const PendingSlot>{},
                                   std::memory_order_release);
    }
}

std::size_t UdpImageSender::PendingQueueDepth() const
{
    const int queueDepth = m_queueDepth.load(std::memory_order_relaxed);
    return static_cast<std::size_t>(
        std::clamp(queueDepth, 1, static_cast<int>(MAX_PENDING_SLOTS)));
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
    slot.preview.release();
    slot.trackedPoints = request.sendFeature ? request.trackedPoints : std::vector<cv::Point2f>{};
    slot.featureBuf.clear();
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
    if (SocketFd() < 0 || camIndex < 0 || camIndex > 1) {
        return;
    }
    while (StepCameraOnce(camIndex)) {
    }
}

bool UdpImageSender::StepCameraOnce(int camIndex)
{
    if (SocketFd() < 0 || camIndex < 0 || camIndex > 1) {
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
    std::size_t readSeq =
        m_readSeq[camIndex].load(std::memory_order_acquire);
    while (true) {
        if (readSeq >=
            m_writeSeq[camIndex].load(std::memory_order_acquire)) {
            return false;
        }
        const std::size_t slotIndex = readSeq % MAX_PENDING_SLOTS;
        std::shared_ptr<const PendingSlot> pending =
            std::atomic_load_explicit(&m_pendingSlots[camIndex][slotIndex],
                                      std::memory_order_acquire);
        if (!pending) {
            return false;
        }
        if (pending->queueSeq > readSeq) {
            m_readSeq[camIndex].compare_exchange_weak(
                readSeq, pending->queueSeq, std::memory_order_acq_rel,
                std::memory_order_acquire);
            continue;
        }
        if (pending->queueSeq < readSeq) {
            return false;
        }
        const std::size_t nextReadSeq = readSeq + 1;
        if (!m_readSeq[camIndex].compare_exchange_weak(
                readSeq, nextReadSeq, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            continue;
        }
        if (!pending->slot) {
            return false;
        }
        slot = *pending->slot;
        return true;
    }
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
        const int jpegQuality = m_jpegQuality.load(std::memory_order_relaxed);
        const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,
                                         jpegQuality};
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
    const int maxPayload = m_maxPayload.load(std::memory_order_relaxed);
    const uint16_t chunks =
        static_cast<uint16_t>((total + maxPayload - 1) / maxPayload);
    for (uint16_t ci = 0; ci < chunks; ++ci) {
        const uint32_t off =
            static_cast<uint32_t>(ci) * static_cast<uint32_t>(maxPayload);
        const uint32_t left = total - off;
        const uint32_t pay =
            (left > static_cast<uint32_t>(maxPayload))
                ? static_cast<uint32_t>(maxPayload)
                : left;

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
    const int sock = SocketFd();
    if (sock < 0) {
        return;
    }

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
    const ssize_t sent = ::sendmsg(sock, &msg, 0);
    (void)sent;
}

void UdpImageSender::SendFeaturePacket(Slot &slot, uint32_t frameId, int width, int height,
                                       const std::vector<cv::Point2f> &trackedPoints)
{
    if (SocketFd() < 0 || width <= 0 || height <= 0 || trackedPoints.empty()) {
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

int UdpImageSender::SocketFd() const
{
    return m_sock.load(std::memory_order_acquire);
}

std::shared_ptr<const UdpImageSender::DestinationState>
UdpImageSender::LoadDestinationState() const
{
    return std::atomic_load_explicit(&m_destinationState,
                                     std::memory_order_acquire);
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
