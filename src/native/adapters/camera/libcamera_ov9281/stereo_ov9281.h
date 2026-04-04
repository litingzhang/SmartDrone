#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <sys/types.h>

#include <opencv2/opencv.hpp>

#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>

extern std::atomic<bool> g_runningFlag;
void SigIntHandler(int);

struct FrameItem {
    int camIndex{-1};
    uint64_t tsNs{0};
    uint32_t seq{0};
    int64_t arriveNs{0};
    cv::Mat gray;
    std::shared_ptr<void> owner;
};

struct PlaneMap {
    void* addr{nullptr};
    size_t len{0};
    off_t off{0};
};

class LibcameraMonoCam {
public:
    bool Open(
        std::shared_ptr<libcamera::Camera> cam,
        int camIndex,
        int w,
        int h,
        int fps,
        bool aeDisable,
        int exposureUs,
        float gain,
        bool requestY8);
    bool Start();
    void Stop();
    void Close();
    void SetSink(std::function<void(FrameItem&&)> sink);
    libcamera::PixelFormat PixelFmt() const;
    libcamera::Size SizeWH() const;
    int Stride() const;
    void SetR16Normalize(bool on);
    bool Healthy() const;

private:
    struct CallbackScope {
        LibcameraMonoCam* self{nullptr};
        bool armed{false};

        explicit CallbackScope(LibcameraMonoCam* owner);
        ~CallbackScope();

        bool Active() const;
    };

    struct FrameSlot {
        cv::Mat gray;
    };

    struct FramePoolState {
        std::atomic<bool> active{false};
        std::vector<std::unique_ptr<FrameSlot>> slots;
        std::deque<FrameSlot*> freeSlots;
        std::mutex mu;
    };

    void WaitForCallbacks();
    std::function<void(FrameItem&&)> CopySink() const;
    bool RequeueRequest(libcamera::Request* req);
    void OnRequestComplete(libcamera::Request* req);
    std::shared_ptr<FrameSlot> AcquireFrameSlot();

    std::shared_ptr<libcamera::Camera> m_cam;
    int m_camIndex{-1};
    std::unique_ptr<libcamera::CameraConfiguration> m_config;
    libcamera::Stream* m_stream{nullptr};
    libcamera::ControlList m_controls{libcamera::controls::controls};
    std::unique_ptr<libcamera::FrameBufferAllocator> m_allocator;
    std::vector<std::unique_ptr<libcamera::Request>> m_requests;
    std::map<libcamera::FrameBuffer*, std::vector<PlaneMap>> m_bufferMaps;
    std::function<void(FrameItem&&)> m_sink;
    mutable std::mutex m_sinkMu;
    bool m_r16Normalize{false};
    std::atomic<bool> m_active{false};
    std::atomic<bool> m_streamFault{false};
    std::mutex m_framePoolMu;
    std::shared_ptr<FramePoolState> m_framePool;
    std::mutex m_callbackMu;
    std::condition_variable m_callbackCv;
    size_t m_callbacksInFlight{0};
    bool m_closing{false};
};

class LibcameraStereoOV9281_TsPair {
public:
    static constexpr size_t kPairLookahead = 3;

    bool Open(
        int w,
        int h,
        int fps,
        bool aeDisable,
        int exposureUs,
        float gain,
        bool requestY8,
        int64_t pairThreshNs,
        int64_t keepWindowNs,
        int maxPairQueue = 8,
        bool r16Normalize = false,
        int leftCamIndex = 0,
        int rightCamIndex = 1);
    void Close();
    bool GrabPair(
        FrameItem& L,
        FrameItem& R,
        int timeoutMs = 1000,
        bool preferLatest = false,
        uint64_t minTimestampNs = 0);

    int64_t LastDtMs() const;
    uint32_t LastSeq() const;
    uint32_t LastRawSeqL() const;
    uint32_t LastRawSeqR() const;
    uint64_t RawCountL() const;
    uint64_t RawCountR() const;
    int64_t LastRejectDtUs() const;
    uint64_t DroppedUnpairedL() const;
    uint64_t DroppedUnpairedR() const;
    size_t PendL() const;
    size_t PendR() const;
    int64_t PairTolNs() const;
    uint64_t DroppedPaired() const;
    bool Healthy() const;

private:
    uint64_t PairTimestampNs(const std::pair<FrameItem, FrameItem>& pair) const;
    bool HasEligiblePairLocked(uint64_t minTimestampNs) const;
    void PushFrame(FrameItem&& fi);
    bool TryPairLocked();
    size_t FindBestMatchIndex(const std::deque<FrameItem>& q, uint64_t targetTs) const;
    void PurgeOldLocked();
    void OnFrameLocked(FrameItem&& fi);

    int m_w{640};
    int m_h{400};
    int m_fps{60};
    int m_maxPairQueue{8};
    int64_t m_pairThreshNs{2'000'000};
    int64_t m_keepWindowNs{120'000'000};

    std::unique_ptr<libcamera::CameraManager> m_cm;
    std::shared_ptr<libcamera::Camera> m_camL;
    std::shared_ptr<libcamera::Camera> m_camR;
    LibcameraMonoCam m_left;
    LibcameraMonoCam m_right;

    mutable std::mutex m_muPair;
    std::condition_variable m_cvPair;
    std::deque<FrameItem> m_qL;
    std::deque<FrameItem> m_qR;
    std::deque<std::pair<FrameItem, FrameItem>> m_paired;
    std::atomic<bool> m_acceptFrames{false};

    std::atomic<int64_t> m_lastDtMs{0};
    std::atomic<uint32_t> m_lastSeq{0};
    std::atomic<uint64_t> m_droppedPaired{0};
    std::atomic<uint32_t> m_lastRawSeq[2]{{0}, {0}};
    std::atomic<uint64_t> m_rawFrameCount[2]{{0}, {0}};
    std::atomic<int64_t> m_lastRejectDtNs{0};
    std::atomic<uint64_t> m_droppedUnpaired[2]{{0}, {0}};
};
