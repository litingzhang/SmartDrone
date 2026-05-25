#include "core/application/state/imu_buffer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <limits>

namespace {

constexpr std::size_t IMU_BUFFER_SLOT_COUNT = 16384;
constexpr std::int64_t IMU_BUFFER_KEEP_NS = 5000000000LL;
constexpr std::int64_t IMU_BUFFER_PURGE_MARGIN_NS = 20000000LL;
constexpr auto IMU_BUFFER_WRITE_ORDER = std::memory_order_release;
constexpr auto IMU_BUFFER_READ_ORDER = std::memory_order_acquire;
constexpr auto IMU_BUFFER_RELAXED_ORDER = std::memory_order_relaxed;

struct TimestampedImuSample {
    std::uint64_t sequence{0};
    ImuSample sample{};
};

struct ImuBufferSlot {
    std::atomic<std::uint64_t> sequence{0};
    std::atomic<std::int64_t> timestampNs{0};
    std::atomic<float> ax{0.0F};
    std::atomic<float> ay{0.0F};
    std::atomic<float> az{0.0F};
    std::atomic<float> gx{0.0F};
    std::atomic<float> gy{0.0F};
    std::atomic<float> gz{0.0F};
};

std::int64_t KeepWindowStartNs(std::int64_t latestTimestampNs)
{
    const std::int64_t minTimestamp = std::numeric_limits<std::int64_t>::min();
    if (latestTimestampNs <= minTimestamp + IMU_BUFFER_KEEP_NS) {
        return minTimestamp;
    }
    return latestTimestampNs - IMU_BUFFER_KEEP_NS;
}

void StoreSampleFields(ImuBufferSlot &slot, const ImuSample &sample)
{
    slot.timestampNs.store(sample.tNs, IMU_BUFFER_RELAXED_ORDER);
    slot.ax.store(sample.ax, IMU_BUFFER_RELAXED_ORDER);
    slot.ay.store(sample.ay, IMU_BUFFER_RELAXED_ORDER);
    slot.az.store(sample.az, IMU_BUFFER_RELAXED_ORDER);
    slot.gx.store(sample.gx, IMU_BUFFER_RELAXED_ORDER);
    slot.gy.store(sample.gy, IMU_BUFFER_RELAXED_ORDER);
    slot.gz.store(sample.gz, IMU_BUFFER_RELAXED_ORDER);
}

bool LoadStableSample(const ImuBufferSlot &slot, TimestampedImuSample &out)
{
    const std::uint64_t startSequence =
        slot.sequence.load(IMU_BUFFER_READ_ORDER);
    if (startSequence == 0 || (startSequence & 1U) != 0U) {
        return false;
    }

    ImuSample sample{};
    sample.tNs = slot.timestampNs.load(IMU_BUFFER_RELAXED_ORDER);
    sample.ax = slot.ax.load(IMU_BUFFER_RELAXED_ORDER);
    sample.ay = slot.ay.load(IMU_BUFFER_RELAXED_ORDER);
    sample.az = slot.az.load(IMU_BUFFER_RELAXED_ORDER);
    sample.gx = slot.gx.load(IMU_BUFFER_RELAXED_ORDER);
    sample.gy = slot.gy.load(IMU_BUFFER_RELAXED_ORDER);
    sample.gz = slot.gz.load(IMU_BUFFER_RELAXED_ORDER);

    const std::uint64_t endSequence =
        slot.sequence.load(IMU_BUFFER_READ_ORDER);
    if (startSequence != endSequence || (endSequence & 1U) != 0U) {
        return false;
    }

    out.sequence = endSequence >> 1U;
    out.sample = sample;
    return true;
}

bool SampleSortsBefore(const TimestampedImuSample &left,
                       const TimestampedImuSample &right)
{
    if (left.sample.tNs == right.sample.tNs) {
        return left.sequence < right.sequence;
    }
    return left.sample.tNs < right.sample.tNs;
}

std::vector<ImuSample> ExtractSamples(
    std::vector<TimestampedImuSample> timestamped)
{
    std::sort(timestamped.begin(), timestamped.end(), SampleSortsBefore);
    std::vector<ImuSample> samples;
    samples.reserve(timestamped.size());
    for (const TimestampedImuSample &entry : timestamped) {
        samples.push_back(entry.sample);
    }
    return samples;
}

void UpdateMaxTimestamp(std::atomic<std::int64_t> &target,
                        std::int64_t candidate)
{
    std::int64_t current = target.load(IMU_BUFFER_READ_ORDER);
    while (candidate > current &&
           !target.compare_exchange_weak(current, candidate,
                                         IMU_BUFFER_WRITE_ORDER,
                                         IMU_BUFFER_READ_ORDER)) {
    }
}

} // namespace

struct ImuBuffer::Impl {
    void Push(const ImuSample &sample)
    {
        const std::uint64_t sampleSequence =
            m_nextSequence.fetch_add(1, IMU_BUFFER_RELAXED_ORDER);
        ImuBufferSlot &slot =
            m_slots[(sampleSequence - 1U) % IMU_BUFFER_SLOT_COUNT];
        slot.sequence.store((sampleSequence << 1U) | 1U,
                            IMU_BUFFER_WRITE_ORDER);
        StoreSampleFields(slot, sample);
        slot.sequence.store(sampleSequence << 1U, IMU_BUFFER_WRITE_ORDER);
        UpdateMaxTimestamp(m_latestTimestampNs, sample.tNs);
    }

    std::vector<ImuSample> ReadSamples() const
    {
        const std::int64_t latestTimestampNs =
            m_latestTimestampNs.load(IMU_BUFFER_READ_ORDER);
        const std::int64_t purgeBeforeNs =
            m_purgeBeforeNs.load(IMU_BUFFER_READ_ORDER);
        const std::int64_t keepStartNs = KeepWindowStartNs(latestTimestampNs);
        const std::int64_t startNs = std::max(keepStartNs, purgeBeforeNs);

        std::vector<TimestampedImuSample> samples;
        samples.reserve(IMU_BUFFER_SLOT_COUNT);
        for (const ImuBufferSlot &slot : m_slots) {
            TimestampedImuSample entry{};
            if (LoadStableSample(slot, entry) && entry.sample.tNs >= startNs) {
                samples.push_back(entry);
            }
        }
        return ExtractSamples(std::move(samples));
    }

    void MarkConsumedThrough(std::int64_t rangeStartNs)
    {
        UpdateMaxTimestamp(m_purgeBeforeNs,
                           rangeStartNs - IMU_BUFFER_PURGE_MARGIN_NS);
    }

    std::array<ImuBufferSlot, IMU_BUFFER_SLOT_COUNT> m_slots{};
    std::atomic<std::uint64_t> m_nextSequence{1};
    std::atomic<std::int64_t> m_latestTimestampNs{
        std::numeric_limits<std::int64_t>::min()};
    std::atomic<std::int64_t> m_purgeBeforeNs{
        std::numeric_limits<std::int64_t>::min()};
};

ImuBuffer::ImuBuffer()
    : m_impl(std::make_unique<Impl>())
{
}

ImuBuffer::~ImuBuffer() = default;

void ImuBuffer::Push(const ImuSample &sample)
{
    m_impl->Push(sample);
}

std::vector<SmartDrone::Core::Ports::ImuReading>
ImuBuffer::PopBetweenNs(const ImuTimeRange &range)
{
    std::vector<SmartDrone::Core::Ports::ImuReading> out;
    const std::vector<ImuSample> samples = m_impl->ReadSamples();

    if (samples.empty() || range.endNs < range.startNs) {
        return out;
    }

    const int64_t rangeStartNs = range.startNs - range.slackBeforeNs;
    const int64_t rangeEndNs = range.endNs + range.slackAfterNs;
    const size_t searchBeginIdx =
        FindFirstIndexAtOrAfter(samples, rangeStartNs, 0);
    out.reserve(std::min(samples.size() - searchBeginIdx,
                         static_cast<size_t>(256)));

    size_t startIdx =
        FindFirstIndexAtOrAfter(samples, range.startNs, searchBeginIdx);
    AppendLeadingSample(samples, range, rangeEndNs, startIdx, out);
    const size_t cursorIdx =
        AppendSamplesUntil(samples, range.startNs, range.endNs, startIdx, out);
    AppendTrailingSample(samples, range, rangeEndNs, cursorIdx, out);

    if (!out.empty()) {
        m_impl->MarkConsumedThrough(range.startNs);
    }
    return out;
}

std::vector<SmartDrone::Core::Ports::ImuReading>
ImuBuffer::PopBetweenNs(int64_t t0Ns, int64_t t1Ns,
                        int64_t slackBeforeNs, int64_t slackAfterNs)
{
    return PopBetweenNs(ImuTimeRange{t0Ns, t1Ns, slackBeforeNs, slackAfterNs});
}

size_t ImuBuffer::Size() const
{
    return m_impl->ReadSamples().size();
}

bool ImuBuffer::PeekFirstLast(int64_t &tFirst, int64_t &tLast) const
{
    const std::vector<ImuSample> samples = m_impl->ReadSamples();
    if (samples.empty()) {
        return false;
    }
    tFirst = samples.front().tNs;
    tLast = samples.back().tNs;
    return true;
}

ImuSample ImuBuffer::InterpolateSample(const ImuSample &a, const ImuSample &b, int64_t targetNs)
{
    if (b.tNs <= a.tNs || targetNs <= a.tNs) {
        ImuSample out = a;
        out.tNs = targetNs;
        return out;
    }
    if (targetNs >= b.tNs) {
        ImuSample out = b;
        out.tNs = targetNs;
        return out;
    }

    const float alpha = static_cast<float>(targetNs - a.tNs) /
                        static_cast<float>(b.tNs - a.tNs);
    ImuSample out{};
    out.tNs = targetNs;
    out.ax = a.ax + (b.ax - a.ax) * alpha;
    out.ay = a.ay + (b.ay - a.ay) * alpha;
    out.az = a.az + (b.az - a.az) * alpha;
    out.gx = a.gx + (b.gx - a.gx) * alpha;
    out.gy = a.gy + (b.gy - a.gy) * alpha;
    out.gz = a.gz + (b.gz - a.gz) * alpha;
    return out;
}

SmartDrone::Core::Ports::ImuReading ImuBuffer::ToReading(const ImuSample &sample)
{
    SmartDrone::Core::Ports::ImuReading reading{};
    reading.timestampNs = sample.tNs;
    reading.ax = sample.ax;
    reading.ay = sample.ay;
    reading.az = sample.az;
    reading.gx = sample.gx;
    reading.gy = sample.gy;
    reading.gz = sample.gz;
    return reading;
}

size_t ImuBuffer::FindFirstIndexAtOrAfter(const std::vector<ImuSample> &samples,
                                          int64_t timestampNs,
                                          size_t beginIdx)
{
    size_t index = beginIdx;
    while (index < samples.size() && samples[index].tNs < timestampNs) {
        ++index;
    }
    return index;
}

void ImuBuffer::AppendLeadingSample(
    const std::vector<ImuSample> &samples, const ImuTimeRange &range,
    int64_t rangeEndNs, size_t &startIdx,
    std::vector<SmartDrone::Core::Ports::ImuReading> &out)
{
    if (startIdx < samples.size() && samples[startIdx].tNs == range.startNs) {
        out.push_back(ToReading(samples[startIdx]));
        ++startIdx;
        return;
    }
    if (startIdx > 0 && startIdx < samples.size() &&
        samples[startIdx - 1].tNs < range.startNs &&
        samples[startIdx].tNs > range.startNs &&
        samples[startIdx].tNs <= rangeEndNs) {
        out.push_back(ToReading(InterpolateSample(samples[startIdx - 1],
                                                  samples[startIdx], range.startNs)));
        return;
    }
    const int64_t rangeStartNs = range.startNs - range.slackBeforeNs;
    if (startIdx > 0 && samples[startIdx - 1].tNs >= rangeStartNs) {
        out.push_back(ToReading(samples[startIdx - 1]));
    }
}

size_t ImuBuffer::AppendSamplesUntil(
    const std::vector<ImuSample> &samples, int64_t startNs, int64_t endNs,
    size_t beginIdx, std::vector<SmartDrone::Core::Ports::ImuReading> &out)
{
    int64_t lastAppendedTsNs =
        out.empty() ? std::numeric_limits<int64_t>::min() : out.back().timestampNs;
    size_t index = beginIdx;
    while (index < samples.size() && samples[index].tNs <= endNs) {
        if (samples[index].tNs > startNs &&
            samples[index].tNs != lastAppendedTsNs) {
            out.push_back(ToReading(samples[index]));
            lastAppendedTsNs = samples[index].tNs;
        }
        ++index;
    }
    return index;
}

void ImuBuffer::AppendTrailingSample(
    const std::vector<ImuSample> &samples, const ImuTimeRange &range,
    int64_t rangeEndNs, size_t cursorIdx,
    std::vector<SmartDrone::Core::Ports::ImuReading> &out)
{
    if (!out.empty() && out.back().timestampNs == range.endNs) {
        return;
    }
    if (cursorIdx < samples.size() && samples[cursorIdx].tNs == range.endNs) {
        out.push_back(ToReading(samples[cursorIdx]));
        return;
    }
    if (cursorIdx > 0 && cursorIdx < samples.size() &&
        samples[cursorIdx - 1].tNs < range.endNs &&
        samples[cursorIdx].tNs > range.endNs &&
        samples[cursorIdx].tNs <= rangeEndNs) {
        out.push_back(ToReading(InterpolateSample(samples[cursorIdx - 1],
                                                  samples[cursorIdx], range.endNs)));
        return;
    }
    if (cursorIdx < samples.size() && samples[cursorIdx].tNs <= rangeEndNs) {
        out.push_back(ToReading(samples[cursorIdx]));
    }
}
