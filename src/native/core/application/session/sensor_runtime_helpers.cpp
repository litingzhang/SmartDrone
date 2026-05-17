#include "core/application/session/sensor_runtime_helpers.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <thread>

#include "adapters/imu/icm42688/icm42688_imu.h"
#include "common/thread_launch.h"
#include "common/time_utils.h"
#include "platform/linux/gpio/drdy_gpio.h"

namespace smartdrone::core::application {

std::thread StartImuThread(const MainRuntimeAliases &a, ImuThreadState &s, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
{
    ImuScale init{};
    s.accelLsbPerG.store(init.accelLsbPerG);
    s.gyroLsbPerDps.store(init.gyroLsbPerDps);
    return smartdrone::common::StartThread(
        smartdrone::common::MakeThreadLaunchInfo(smartdrone::common::ThreadRole::Imu, "SensorRuntime"),
        [&a, &s, &stop, &runningFlag]() {
            if (a.rtImu)
                SetThreadRealtime(a.rtPrio);
            SpiDev spi(a.spiDev);
            if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits))
                return;
            ImuScale scale{};
            if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale))
                return;
            s.accelLsbPerG.store(scale.accelLsbPerG);
            s.gyroLsbPerDps.store(scale.gyroLsbPerDps);
            DrdyGpio drdy;
            if (!drdy.Open(a.gpiochip, a.drdyLine))
                return;
            s.imuOk.store(true);
            uint8_t raw12[12]{};
            uint8_t st = 0;
            spi.ReadReg(REG_INT_STATUS, st);
            int64_t lastAcceptedTsNs = 0;
            uint64_t lastNonMonotonicLogUs = 0;
            while (runningFlag.load() && !stop.load()) {
                int64_t tNs = 0;
                if (!drdy.WaitTs(1000, tNs))
                    continue;
                if (lastAcceptedTsNs != 0 && tNs <= lastAcceptedTsNs) {
                    s.imuDrop.fetch_add(1, std::memory_order_relaxed);
                    const uint64_t nowUs = MonoTimeUs();
                    if ((lastNonMonotonicLogUs == 0) || (nowUs - lastNonMonotonicLogUs >= 1000000ULL)) {
                        std::cerr << "[imu] dropped non-monotonic DRDY timestamp"
                                  << " prev_ns=" << lastAcceptedTsNs << " cur_ns=" << tNs << "\n";
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

std::thread StartCalibImuWriterThread(const MainRuntimeAliases &a, FILE *fImu, std::atomic<bool> &imuOk,
                                      std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
{
    return smartdrone::common::StartThread(
        smartdrone::common::MakeThreadLaunchInfo(smartdrone::common::ThreadRole::CalibImuWriter, "SensorRuntime"),
        [&a, fImu, &imuOk, &stop, &runningFlag]() {
            SpiDev spi(a.spiDev);
            if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits))
                return;
            ImuScale scale{};
            if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale))
                return;
            DrdyGpio drdy;
            if (!drdy.Open(a.gpiochip, a.drdyLine))
                return;
            imuOk.store(true);
            uint8_t raw12[12]{};
            uint8_t st = 0;
            spi.ReadReg(REG_INT_STATUS, st);
            int lines = 0;
            while (runningFlag.load() && !stop.load()) {
                int64_t tNs = 0;
                if (!drdy.WaitTs(1000, tNs))
                    continue;
                ImuSample sample{};
                sample.tNs = tNs;
                spi.ReadReg(REG_INT_STATUS, st);
                if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12)))
                    continue;
                ConvertRaw12AccelGyroToSi(raw12, scale, sample);
                std::fprintf(fImu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", static_cast<long long>(sample.tNs),
                             static_cast<double>(sample.gx), static_cast<double>(sample.gy),
                             static_cast<double>(sample.gz), static_cast<double>(sample.ax),
                             static_cast<double>(sample.ay), static_cast<double>(sample.az));
                if ((++lines % 800) == 0)
                    std::fflush(fImu);
            }
            std::fflush(fImu);
        });
}

} // namespace smartdrone::core::application
