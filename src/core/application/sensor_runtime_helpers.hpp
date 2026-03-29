#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <thread>

#include "ImuTypes.h"
#include "System.h"
#include "adapters/camera/libcamera_ov9281/stereo_ov9281.hpp"
#include "adapters/imu/icm42688/icm42688_imu.hpp"
#include "common/time_utils.hpp"
#include "core/application/runtime_session_common.hpp"
#include "platform/linux/gpio/drdy_gpio.hpp"

namespace smartdrone::core::application {

inline std::thread StartImuThread(const MainRuntimeAliases& a,
                                  ImuThreadState& s,
                                  std::atomic<bool>& stop,
                                  std::atomic<bool>& runningFlag)
{
    ImuScale init{};
    s.accelLsbPerG.store(init.accelLsbPerG);
    s.gyroLsbPerDps.store(init.gyroLsbPerDps);
    return std::thread([&a, &s, &stop, &runningFlag]() {
        if (a.rtImu) SetThreadRealtime(a.rtPrio);
        SpiDev spi(a.spiDev);
        if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits)) return;
        ImuScale scale{};
        if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale)) return;
        s.accelLsbPerG.store(scale.accelLsbPerG);
        s.gyroLsbPerDps.store(scale.gyroLsbPerDps);
        DrdyGpio drdy;
        if (!drdy.Open(a.gpiochip, a.drdyLine)) return;
        s.imuOk.store(true);
        uint8_t raw12[12]{};
        uint8_t st = 0;
        spi.ReadReg(REG_INT_STATUS, st);
        int64_t lastAcceptedTsNs = 0;
        uint64_t lastNonMonotonicLogUs = 0;
        while (runningFlag.load() && !stop.load()) {
            int64_t tNs = 0;
            if (!drdy.WaitTs(1000, tNs)) continue;
            if (lastAcceptedTsNs != 0 && tNs <= lastAcceptedTsNs) {
                s.imuDrop.fetch_add(1, std::memory_order_relaxed);
                const uint64_t nowUs = MonoTimeUs();
                if ((lastNonMonotonicLogUs == 0) || (nowUs - lastNonMonotonicLogUs >= 1000000ULL)) {
                    std::cerr << "[imu] dropped non-monotonic DRDY timestamp"
                              << " prev_ns=" << lastAcceptedTsNs
                              << " cur_ns=" << tNs
                              << "\n";
                    lastNonMonotonicLogUs = nowUs;
                }
                continue;
            }
            ImuSample sample{};
            sample.tNs = tNs;
            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12))) {
                s.imuDrop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            ImuScale cur{};
            cur.accelLsbPerG = s.accelLsbPerG.load();
            cur.gyroLsbPerDps = s.gyroLsbPerDps.load();
            ConvertRaw12AccelGyroToSi(raw12, cur, sample);
            s.imuBuffer.Push(sample);
            lastAcceptedTsNs = tNs;
            s.imuCnt.fetch_add(1, std::memory_order_relaxed);
        }
    });
}

inline std::thread StartCalibImuWriterThread(const MainRuntimeAliases& a,
                                             FILE* fImu,
                                             std::atomic<bool>& imuOk,
                                             std::atomic<bool>& stop,
                                             std::atomic<bool>& runningFlag)
{
    return std::thread([&a, fImu, &imuOk, &stop, &runningFlag]() {
        SpiDev spi(a.spiDev);
        if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits)) return;
        ImuScale scale{};
        if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale)) return;
        DrdyGpio drdy;
        if (!drdy.Open(a.gpiochip, a.drdyLine)) return;
        imuOk.store(true);
        uint8_t raw12[12]{};
        uint8_t st = 0;
        spi.ReadReg(REG_INT_STATUS, st);
        int lines = 0;
        while (runningFlag.load() && !stop.load()) {
            int64_t tNs = 0;
            if (!drdy.WaitTs(1000, tNs)) continue;
            ImuSample sample{};
            sample.tNs = tNs;
            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12))) continue;
            ConvertRaw12AccelGyroToSi(raw12, scale, sample);
            std::fprintf(fImu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                         static_cast<long long>(sample.tNs), static_cast<double>(sample.gx),
                         static_cast<double>(sample.gy), static_cast<double>(sample.gz),
                         static_cast<double>(sample.ax), static_cast<double>(sample.ay),
                         static_cast<double>(sample.az));
            if ((++lines % 800) == 0) std::fflush(fImu);
        }
        std::fflush(fImu);
    });
}

inline bool OpenCamera(LibcameraStereoOV9281_TsPair& cam, const MainRuntimeAliases& a)
{
    return cam.Open(a.width, a.height, a.fps, a.aeDisable, a.exposureUs, a.gain, a.requestY8,
                    static_cast<int64_t>(a.pairMs) * 1000000LL, static_cast<int64_t>(a.keepMs) * 1000000LL,
                    a.pairQueue, a.r16Norm, a.leftCamIndex, a.rightCamIndex);
}

}  // namespace smartdrone::core::application
