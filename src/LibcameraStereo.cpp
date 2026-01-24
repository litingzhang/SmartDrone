#include "LibcameraStereo.h"
#include "FrameTypes.h"
#include <cerrno>
#include <cstring>
#include <iostream>
#include <libcamera/formats.h>
#include <sys/mman.h>
#include <unistd.h>

class FrameBlob {
  public:
    static bool CopyFrameBufferPlanes(libcamera::FrameBuffer *fb, std::shared_ptr<std::vector<uint8_t>> &outBlob)
    {
        if (!fb)
            return false;

        const auto &planes = fb->planes();
        if (planes.empty())
            return false;

        size_t total = 0;
        for (const auto &p : planes)
            total += p.length;

        auto blob = std::make_shared<std::vector<uint8_t>>();
        blob->resize(total);

        const long page = sysconf(_SC_PAGESIZE);
        const size_t pageMask = static_cast<size_t>(page) - 1;

        size_t outOff = 0;

        for (size_t i = 0; i < planes.size(); i++) {
            const auto &p = planes[i];

            int fd = p.fd.get();
            size_t len = p.length;
            size_t off = p.offset;

            size_t offAligned = off & ~pageMask;
            size_t delta = off - offAligned;
            size_t mapLen = len + delta;

            void *addr = mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fd, offAligned);
            if (addr == MAP_FAILED) {
                std::cerr << "mmap failed: plane=" << i << " fd=" << fd << " off=" << off
                          << " offAligned=" << offAligned << " len=" << len << " mapLen=" << mapLen
                          << " errno=" << errno << " (" << strerror(errno) << ")\n";
                return false;
            }

            std::memcpy(blob->data() + outOff, reinterpret_cast<uint8_t *>(addr) + delta, len);
            outOff += len;

            munmap(addr, mapLen);
        }

        outBlob = std::move(blob);
        return true;
    }
};

LibcameraStereo::CamCtx::CamCtx()
    : m_index(-1), m_cam(nullptr), m_config(nullptr), m_alloc(nullptr), m_stream(nullptr),
      m_requests(), m_width(0), m_height(0), m_stride(0), m_fmt(), m_owner(nullptr)
{
}

void LibcameraStereo::CamCtx::OnRequestComplete(libcamera::Request *req)
{
    if (!req || req->status() == libcamera::Request::RequestCancelled)
        return;

    auto &bufMap = req->buffers();
    auto it = bufMap.find(m_stream);
    if (it == bufMap.end())
        return;

    libcamera::FrameBuffer *fb = it->second;
    const libcamera::FrameMetadata &md = fb->metadata();

    std::shared_ptr<std::vector<uint8_t>> blob;
    if (!FrameBlob::CopyFrameBufferPlanes(fb, blob)) {
        std::cerr << "cam" << m_index << " CopyFrameBufferPlanes FAILED\n";
        req->reuse(libcamera::Request::ReuseBuffers);
        m_cam->queueRequest(req);
        return;
    }

    FrameItem item;
    item.m_camIndex = m_index;
    item.m_tsNs = md.timestamp;
    item.m_stride = m_stride;
    item.m_fmt = m_fmt;
    item.m_width = m_width;
    item.m_height = m_height;
    item.m_data = blob;

    m_owner->m_queue.PushFrame(item);

    req->reuse(libcamera::Request::ReuseBuffers);
    m_cam->queueRequest(req);
}

LibcameraStereo::LibcameraStereo(FrameSyncQueue &queue)
    : m_queue(queue), m_running(false), m_left(), m_right()
{
    m_left.m_index = 0;
    m_right.m_index = 1;
    m_left.m_owner = this;
    m_right.m_owner = this;
}

LibcameraStereo::~LibcameraStereo() { Stop(); }

bool LibcameraStereo::Start(const std::shared_ptr<libcamera::Camera> &leftCam,
                            const std::shared_ptr<libcamera::Camera> &rightCam, unsigned int width,
                            unsigned int height)
{
    if (m_running)
        return true;

    if (!SetupCamera(m_left, leftCam, width, height)) {
        StopCamera(m_left);
        return false;
    }

    if (!SetupCamera(m_right, rightCam, width, height)) {
        StopCamera(m_left);
        StopCamera(m_right);
        return false;
    }

    m_running = true;
    return true;
}

void LibcameraStereo::Stop()
{
    if (!m_running) {
        StopCamera(m_left);
        StopCamera(m_right);
        return;
    }

    StopCamera(m_left);
    StopCamera(m_right);
    m_running = false;
}

bool LibcameraStereo::SetupCamera(CamCtx &ctx, const std::shared_ptr<libcamera::Camera> &cam,
                                  unsigned int width, unsigned int height)
{
    ctx.m_cam = cam;

    if (ctx.m_cam->acquire()) {
        std::cerr << "Failed to acquire camera " << ctx.m_index << "\n";
        return false;
    }

    ctx.m_config = ctx.m_cam->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!ctx.m_config) {
        std::cerr << "Failed to generate configuration (cam " << ctx.m_index << ")\n";
        return false;
    }

    libcamera::StreamConfiguration &sc = ctx.m_config->at(0);
    sc.size.width = width;
    sc.size.height = height;
    sc.pixelFormat = libcamera::formats::YUV420;

    ctx.m_config->validate();
    if (ctx.m_cam->configure(ctx.m_config.get()) < 0) {
        std::cerr << "Failed to configure camera " << ctx.m_index << "\n";
        return false;
    }

    ctx.m_stream = sc.stream();
    ctx.m_width = sc.size.width;
    ctx.m_height = sc.size.height;
    ctx.m_stride = sc.stride;
    ctx.m_fmt = sc.pixelFormat;

    std::cerr << "cam" << ctx.m_index << " configured: " << sc.size.toString()
              << " stride=" << sc.stride << " pixelFormat=" << sc.pixelFormat.toString() << "\n";

    ctx.m_alloc = std::make_unique<libcamera::FrameBufferAllocator>(ctx.m_cam);
    if (ctx.m_alloc->allocate(ctx.m_stream) < 0) {
        std::cerr << "Failed to allocate buffers (cam " << ctx.m_index << ")\n";
        return false;
    }

    ctx.m_cam->requestCompleted.connect(&ctx, &LibcameraStereo::CamCtx::OnRequestComplete);

    const auto &buffers = ctx.m_alloc->buffers(ctx.m_stream);
    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : buffers) {
        std::unique_ptr<libcamera::Request> req = ctx.m_cam->createRequest();
        if (!req) {
            std::cerr << "Failed to create request (cam " << ctx.m_index << ")\n";
            return false;
        }

        if (req->addBuffer(ctx.m_stream, buffer.get()) < 0) {
            std::cerr << "Failed to add buffer to request (cam " << ctx.m_index << ")\n";
            return false;
        }

        ctx.m_requests.push_back(std::move(req));
    }

    if (ctx.m_cam->start() < 0) {
        std::cerr << "Failed to start camera " << ctx.m_index << "\n";
        return false;
    }

    for (auto &req : ctx.m_requests) {
        if (ctx.m_cam->queueRequest(req.get()) < 0) {
            std::cerr << "Failed to queue request (cam " << ctx.m_index << ")\n";
            return false;
        }
    }

    return true;
}

void LibcameraStereo::StopCamera(CamCtx &ctx)
{
    if (!ctx.m_cam)
        return;

    ctx.m_cam->stop();
    ctx.m_cam->release();

    ctx.m_requests.clear();
    ctx.m_alloc.reset();
    ctx.m_config.reset();
    ctx.m_stream = nullptr;
    ctx.m_cam.reset();
}
