#pragma once

#include <memory>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>

#include "FrameSyncQueue.h"

class LibcameraStereo {
  public:
    explicit LibcameraStereo(FrameSyncQueue &queue);
    ~LibcameraStereo();

    bool Start(const std::shared_ptr<libcamera::Camera> &leftCam,
               const std::shared_ptr<libcamera::Camera> &rightCam, unsigned int width,
               unsigned int height);

    void Stop();

  private:
    struct CamCtx {
        int m_index;
        std::shared_ptr<libcamera::Camera> m_cam;
        std::unique_ptr<libcamera::CameraConfiguration> m_config;
        std::unique_ptr<libcamera::FrameBufferAllocator> m_alloc;
        libcamera::Stream *m_stream;
        std::vector<std::unique_ptr<libcamera::Request>> m_requests;

        unsigned int m_width;
        unsigned int m_height;
        unsigned int m_stride;
        libcamera::PixelFormat m_fmt;

        LibcameraStereo *m_owner;

        CamCtx();
        void OnRequestComplete(libcamera::Request *req);
    };

    bool SetupCamera(CamCtx &ctx, const std::shared_ptr<libcamera::Camera> &cam, unsigned int width,
                     unsigned int height);

    void StopCamera(CamCtx &ctx);

  private:
    FrameSyncQueue &m_queue;
    bool m_running;

    CamCtx m_left;
    CamCtx m_right;
};
