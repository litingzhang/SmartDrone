#pragma once

#include <opencv2/core.hpp>
#include "FrameTypes.h"

class CvUtils {
  public:
    static cv::Mat YPlaneToGrayMat(const FrameItem &item)
    {
        cv::Mat gray(static_cast<int>(item.m_height), static_cast<int>(item.m_width), CV_8UC1);

        const uint8_t *y = item.m_data->data();

        for (unsigned int r = 0; r < item.m_height; r++) {
            std::memcpy(gray.ptr(r), y + r * item.m_stride, item.m_width);
        }

        return gray;
    }
};

class CamOffsetLearner {
  public:
    CamOffsetLearner() : m_haveOffset(false), m_count(0), m_offsetNs(0.0) {}

    bool UpdateAndCheckReady(uint64_t leftTsNs, uint64_t rightTsNs)
    {
        if (m_haveOffset)
            return true;

        double cur = static_cast<double>(rightTsNs) - static_cast<double>(leftTsNs);
        m_offsetNs = (m_offsetNs * m_count + cur) / static_cast<double>(m_count + 1);
        m_count++;

        if (m_count % 10 == 0) {
            std::cerr << "[CAM] learning offset: " << m_offsetNs * 1e-6 << " ms\n";
        }

        if (m_count >= 60) {
            m_haveOffset = true;
            std::cerr << "[CAM] final offset: " << m_offsetNs * 1e-6 << " ms\n";
        }
        return m_haveOffset;
    }

    bool HaveOffset() const { return m_haveOffset; }

    double GetOffsetNs() const { return m_offsetNs; }

  private:
    bool m_haveOffset;
    int m_count;
    double m_offsetNs; // cam1 - cam0
};