bool LibcameraStereoOV9281_TsPair::Open(const StereoCameraOpenParams &params)
{
    Close();
    ResetPairingState();
    ApplyOpenParams(params);
    m_acceptFrames.store(true, std::memory_order_relaxed);

    if (!StartCameraManager() || !SelectCameras(params.leftCamIndex, params.rightCamIndex) ||
        !OpenMonoCameras(params) || !StartMonoCameras()) {
        Close();
        return false;
    }

    LogMonoFormats();
    return true;
}

void LibcameraStereoOV9281_TsPair::ApplyOpenParams(const StereoCameraOpenParams &params)
{
    m_w = params.width;
    m_h = params.height;
    m_fps = params.fps;
    m_maxPairQueue = params.maxPairQueue;
    m_pairThreshNs = params.pairThreshNs;
    m_keepWindowNs = params.keepWindowNs;
}

bool LibcameraStereoOV9281_TsPair::StartCameraManager()
{
    m_cm = std::make_unique<libcamera::CameraManager>();
    if (!m_cm->start()) {
        return true;
    }
    std::cerr << "CameraManager start failed\n";
    return false;
}

bool LibcameraStereoOV9281_TsPair::SelectCameras(int leftCamIndex, int rightCamIndex)
{
    const auto &cams = m_cm->cameras();
    if (cams.size() < 2) {
        std::cerr << "Need 2 cameras, but found " << cams.size() << "\n";
        return false;
    }

    const int camCount = static_cast<int>(cams.size());
    if (leftCamIndex < 0 || leftCamIndex >= camCount || rightCamIndex < 0 || rightCamIndex >= camCount) {
        std::cerr << "[cam] invalid camera index left=" << leftCamIndex << " right=" << rightCamIndex
                  << " available=" << camCount << "\n";
        for (int i = 0; i < camCount; ++i) {
            std::cerr << "[cam] available index=" << i << " id=" << cams[static_cast<size_t>(i)]->id() << "\n";
        }
        return false;
    }
    if (leftCamIndex == rightCamIndex) {
        std::cerr << "[cam] left and right camera index must differ, got " << leftCamIndex << "\n";
        return false;
    }

    m_camL = cams[static_cast<size_t>(leftCamIndex)];
    m_camR = cams[static_cast<size_t>(rightCamIndex)];
    return true;
}

bool LibcameraStereoOV9281_TsPair::OpenMonoCameras(const StereoCameraOpenParams &params)
{
    auto sink = [&](FrameItem &&fi) { PushFrame(std::move(fi)); };
    const MonoCameraOpenParams leftParams{m_camL, 0, params.width, params.height, params.fps, params.aeDisable,
                                          params.exposureUs, params.gain, params.requestY8};
    const MonoCameraOpenParams rightParams{m_camR, 1, params.width, params.height, params.fps, params.aeDisable,
                                           params.exposureUs, params.gain, params.requestY8};
    if (!m_left.Open(leftParams)) {
        return false;
    }
    if (!m_right.Open(rightParams)) {
        return false;
    }

    m_left.SetR16Normalize(params.r16Normalize);
    m_right.SetR16Normalize(params.r16Normalize);
    m_left.SetSink(sink);
    m_right.SetSink(sink);
    return true;
}

bool LibcameraStereoOV9281_TsPair::StartMonoCameras()
{
    if (!m_left.Start()) {
        return false;
    }
    if (!m_right.Start()) {
        return false;
    }
    return true;
}

void LibcameraStereoOV9281_TsPair::LogMonoFormats() const
{
    std::cerr << "Left fmt=" << m_left.PixelFmt().toString() << " size=" << m_left.SizeWH().toString()
              << " stride=" << m_left.Stride() << "\n";
    std::cerr << "Right fmt=" << m_right.PixelFmt().toString() << " size=" << m_right.SizeWH().toString()
              << " stride=" << m_right.Stride() << "\n";
}

void LibcameraStereoOV9281_TsPair::Close()
{
    m_acceptFrames.store(false, std::memory_order_relaxed);
    m_left.SetSink(nullptr);
    m_right.SetSink(nullptr);
    m_left.Stop();
    m_right.Stop();

    {
        std::lock_guard<std::mutex> lk(m_muPair);
        m_qL.clear();
        m_qR.clear();
        m_paired.clear();
    }
    m_cvPair.notify_all();

    m_left.Close();
    m_right.Close();
    m_camL.reset();
    m_camR.reset();
    if (m_cm) {
        m_cm->stop();
    }
    m_cm.reset();
    ResetPairingState();
}

bool LibcameraStereoOV9281_TsPair::GrabPair(FrameItem &L, FrameItem &R, int timeoutMs, bool preferLatest,
                                            uint64_t minTimestampNs)
{
    std::unique_lock<std::mutex> lk(m_muPair);
    if (timeoutMs <= 0) {
        return TryGrabPairLocked(L, R, preferLatest, minTimestampNs);
    }
    if (!m_cvPair.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&] {
            return HasEligiblePairLocked(minTimestampNs) || !SmartDrone::Common::g_runningFlag.load() ||
                   !m_acceptFrames.load(std::memory_order_relaxed);
        })) {
        return false;
    }
    return TryGrabPairLocked(L, R, preferLatest, minTimestampNs);
}

void LibcameraStereoOV9281_TsPair::DropPairsBeforeLocked(size_t selectedIndex)
{
    if (selectedIndex > 0) {
        m_droppedPaired.fetch_add(selectedIndex, std::memory_order_relaxed);
        for (size_t i = 0; i < selectedIndex; ++i) {
            m_paired.pop_front();
        }
    }
}

bool LibcameraStereoOV9281_TsPair::TryGrabPairLocked(FrameItem &L, FrameItem &R, bool preferLatest,
                                                     uint64_t minTimestampNs)
{
    if (!HasEligiblePairLocked(minTimestampNs)) {
        return false;
    }
    const size_t selectedIndex = SelectPairIndexLocked(preferLatest, minTimestampNs);
    if (selectedIndex >= m_paired.size()) {
        return false;
    }
    DropPairsBeforeLocked(selectedIndex);
    auto &p = m_paired.front();
    L = std::move(p.first);
    R = std::move(p.second);
    m_paired.pop_front();
    const int64_t pairMonoNs = std::max(L.arriveNs, R.arriveNs);
    m_lastPairMonoNs.store(pairMonoNs, std::memory_order_relaxed);
    return true;
}

int64_t LibcameraStereoOV9281_TsPair::LastDtMs() const
{
    return m_lastDtMs.load();
}

uint32_t LibcameraStereoOV9281_TsPair::LastSeq() const
{
    return m_lastSeq.load();
}

uint32_t LibcameraStereoOV9281_TsPair::LastRawSeqL() const
{
    return m_lastRawSeq[0].load(std::memory_order_relaxed);
}

uint32_t LibcameraStereoOV9281_TsPair::LastRawSeqR() const
{
    return m_lastRawSeq[1].load(std::memory_order_relaxed);
}

uint64_t LibcameraStereoOV9281_TsPair::RawCountL() const
{
    return m_rawFrameCount[0].load(std::memory_order_relaxed);
}

uint64_t LibcameraStereoOV9281_TsPair::RawCountR() const
{
    return m_rawFrameCount[1].load(std::memory_order_relaxed);
}

int64_t LibcameraStereoOV9281_TsPair::LastRejectDtUs() const
{
    return m_lastRejectDtNs.load(std::memory_order_relaxed) / 1000;
}

uint64_t LibcameraStereoOV9281_TsPair::DroppedUnpairedL() const
{
    return m_droppedUnpaired[0].load(std::memory_order_relaxed);
}

uint64_t LibcameraStereoOV9281_TsPair::DroppedUnpairedR() const
{
    return m_droppedUnpaired[1].load(std::memory_order_relaxed);
}

size_t LibcameraStereoOV9281_TsPair::PendL() const
{
    std::lock_guard<std::mutex> lk(m_muPair);
    return m_qL.size();
}

size_t LibcameraStereoOV9281_TsPair::PendR() const
{
    std::lock_guard<std::mutex> lk(m_muPair);
    return m_qR.size();
}

int64_t LibcameraStereoOV9281_TsPair::PairTolNs() const
{
    return m_pairThreshNs;
}

uint64_t LibcameraStereoOV9281_TsPair::DroppedPaired() const
{
    return m_droppedPaired.load(std::memory_order_relaxed);
}

bool LibcameraStereoOV9281_TsPair::Healthy() const
{
    return m_left.Healthy() && m_right.Healthy();
}

LibcameraStereoOV9281_TsPair::PairingDiagnostics LibcameraStereoOV9281_TsPair::GetDiagnostics() const
{
    PairingDiagnostics out{};
    out.healthy = Healthy();
    out.acceptFrames = m_acceptFrames.load(std::memory_order_relaxed);
    out.lastRawSeqL = m_lastRawSeq[0].load(std::memory_order_relaxed);
    out.lastRawSeqR = m_lastRawSeq[1].load(std::memory_order_relaxed);
    out.rawCountL = m_rawFrameCount[0].load(std::memory_order_relaxed);
    out.rawCountR = m_rawFrameCount[1].load(std::memory_order_relaxed);
    out.droppedPaired = m_droppedPaired.load(std::memory_order_relaxed);
    out.droppedUnpairedL = m_droppedUnpaired[0].load(std::memory_order_relaxed);
    out.droppedUnpairedR = m_droppedUnpaired[1].load(std::memory_order_relaxed);
    out.pairTolNs = m_pairThreshNs;
    out.lastPairDtMs = m_lastDtMs.load(std::memory_order_relaxed);
    out.lastRejectDtUs = LastRejectDtUs();

    const int64_t nowNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    const int64_t lastArriveNsL = m_lastArriveNs[0].load(std::memory_order_relaxed);
    const int64_t lastArriveNsR = m_lastArriveNs[1].load(std::memory_order_relaxed);
    const int64_t lastPairMonoNs = m_lastPairMonoNs.load(std::memory_order_relaxed);
    out.lastFrameAgeMsL = lastArriveNsL > 0 ? (nowNs - lastArriveNsL) / 1000000LL : -1;
    out.lastFrameAgeMsR = lastArriveNsR > 0 ? (nowNs - lastArriveNsR) / 1000000LL : -1;
    out.lastPairAgeMs = lastPairMonoNs > 0 ? (nowNs - lastPairMonoNs) / 1000000LL : -1;

    std::lock_guard<std::mutex> lk(m_muPair);
    out.pendingL = m_qL.size();
    out.pendingR = m_qR.size();
    out.pairedQueue = m_paired.size();
    return out;
}

uint64_t LibcameraStereoOV9281_TsPair::PairTimestampNs(const std::pair<FrameItem, FrameItem> &pair) const
{
    return (pair.first.tsNs + pair.second.tsNs) / 2ULL;
}

bool LibcameraStereoOV9281_TsPair::HasEligiblePairLocked(uint64_t minTimestampNs) const
{
    if (m_paired.empty()) {
        return false;
    }
    if (minTimestampNs == 0) {
        return true;
    }
    for (const auto &pair : m_paired) {
        if (PairTimestampNs(pair) >= minTimestampNs) {
            return true;
        }
    }
    return false;
}

size_t LibcameraStereoOV9281_TsPair::SelectPairIndexLocked(bool preferLatest, uint64_t minTimestampNs) const
{
    if (minTimestampNs == 0) {
        return preferLatest && m_paired.size() > 1 ? m_paired.size() - 1 : 0;
    }
    size_t selectedIndex = m_paired.size();
    for (size_t i = 0; i < m_paired.size(); ++i) {
        if (PairTimestampNs(m_paired[i]) < minTimestampNs) {
            continue;
        }
        selectedIndex = i;
        if (!preferLatest) {
            break;
        }
    }
    return selectedIndex;
}

void LibcameraStereoOV9281_TsPair::PushFrame(FrameItem &&fi)
{
    if (!m_acceptFrames.load(std::memory_order_relaxed)) {
        return;
    }
    std::lock_guard<std::mutex> lk(m_muPair);
    if (!m_acceptFrames.load(std::memory_order_relaxed)) {
        return;
    }
    OnFrameLocked(std::move(fi));
}

bool LibcameraStereoOV9281_TsPair::TryPairLocked()
{
    if (m_qL.empty() || m_qR.empty()) {
        return false;
    }

    const PairMatchSelection selection = SelectPairMatchLocked();
    if (!selection.valid) {
        return false;
    }
    if (selection.bestDtNs > m_pairThreshNs) {
        return DropOldestUnpairedLocked(selection.bestDtNs);
    }

    CommitPairLocked(selection);
    return true;
}

LibcameraStereoOV9281_TsPair::PairMatchSelection LibcameraStereoOV9281_TsPair::SelectPairMatchLocked() const
{
    const size_t bestRightForLeft = FindBestMatchIndex(m_qR, m_qL.front().tsNs);
    const int64_t bestLeftDt =
        (bestRightForLeft < m_qR.size())
            ? Abs64(static_cast<int64_t>(m_qR[bestRightForLeft].tsNs) - static_cast<int64_t>(m_qL.front().tsNs))
            : INT64_MAX;
    const size_t bestLeftForRight = FindBestMatchIndex(m_qL, m_qR.front().tsNs);
    const int64_t bestRightDt =
        (bestLeftForRight < m_qL.size())
            ? Abs64(static_cast<int64_t>(m_qL[bestLeftForRight].tsNs) - static_cast<int64_t>(m_qR.front().tsNs))
            : INT64_MAX;

    const bool pairFromLeft = bestLeftDt <= bestRightDt;
    PairMatchSelection selection;
    selection.valid = true;
    selection.pairFromLeft = pairFromLeft;
    selection.leftIndex = pairFromLeft ? 0 : bestLeftForRight;
    selection.rightIndex = pairFromLeft ? bestRightForLeft : 0;
    selection.bestDtNs = pairFromLeft ? bestLeftDt : bestRightDt;
    return selection;
}

bool LibcameraStereoOV9281_TsPair::DropOldestUnpairedLocked(int64_t rejectDtNs)
{
    m_lastRejectDtNs.store(rejectDtNs == INT64_MAX ? -1 : rejectDtNs, std::memory_order_relaxed);
    if (m_qL.front().tsNs <= m_qR.front().tsNs) {
        m_droppedUnpaired[0].fetch_add(1, std::memory_order_relaxed);
        m_qL.pop_front();
    } else {
        m_droppedUnpaired[1].fetch_add(1, std::memory_order_relaxed);
        m_qR.pop_front();
    }
    return true;
}

void LibcameraStereoOV9281_TsPair::CommitPairLocked(const PairMatchSelection &selection)
{
    FrameItem L;
    FrameItem R;
    if (selection.pairFromLeft) {
        L = std::move(m_qL.front());
        R = std::move(m_qR[selection.rightIndex]);
        m_qL.pop_front();
        m_qR.erase(m_qR.begin() + static_cast<std::ptrdiff_t>(selection.rightIndex));
    } else {
        L = std::move(m_qL[selection.leftIndex]);
        R = std::move(m_qR.front());
        m_qL.erase(m_qL.begin() + static_cast<std::ptrdiff_t>(selection.leftIndex));
        m_qR.pop_front();
    }

    m_lastDtMs.store((static_cast<int64_t>(R.tsNs) - static_cast<int64_t>(L.tsNs)) / 1'000'000);
    m_lastSeq.store(L.seq);

    m_paired.push_back({std::move(L), std::move(R)});
    while (static_cast<int>(m_paired.size()) > m_maxPairQueue) {
        m_paired.pop_front();
        m_droppedPaired.fetch_add(1, std::memory_order_relaxed);
    }
    m_cvPair.notify_one();
}

size_t LibcameraStereoOV9281_TsPair::FindBestMatchIndex(const std::deque<FrameItem> &q, uint64_t targetTs) const
{
    const size_t limit = std::min(kPairLookahead, q.size());
    size_t bestIdx = q.size();
    int64_t bestDt = INT64_MAX;
    for (size_t i = 0; i < limit; ++i) {
        const int64_t dt = Abs64(static_cast<int64_t>(q[i].tsNs) - static_cast<int64_t>(targetTs));
        if (dt < bestDt) {
            bestDt = dt;
            bestIdx = i;
        }
    }
    return bestIdx;
}

void LibcameraStereoOV9281_TsPair::PurgeOldLocked()
{
    uint64_t newest = 0;
    if (!m_qL.empty()) {
        newest = std::max<uint64_t>(newest, m_qL.back().tsNs);
    }
    if (!m_qR.empty()) {
        newest = std::max<uint64_t>(newest, m_qR.back().tsNs);
    }

    auto purge = [&](std::deque<FrameItem> &q) {
        while (!q.empty() && static_cast<int64_t>(newest - q.front().tsNs) > m_keepWindowNs) {
            q.pop_front();
        }
    };
    purge(m_qL);
    purge(m_qR);
}

void LibcameraStereoOV9281_TsPair::OnFrameLocked(FrameItem &&fi)
{
    if (fi.camIndex >= 0 && fi.camIndex < 2) {
        m_lastRawSeq[fi.camIndex].store(fi.seq, std::memory_order_relaxed);
        m_rawFrameCount[fi.camIndex].fetch_add(1, std::memory_order_relaxed);
        m_lastArriveNs[fi.camIndex].store(fi.arriveNs, std::memory_order_relaxed);
    }
    if (fi.camIndex == 0) {
        m_qL.push_back(std::move(fi));
    } else {
        m_qR.push_back(std::move(fi));
    }

    PurgeOldLocked();
    while (TryPairLocked()) {
    }
}

void LibcameraMonoCam::ResetOpenState()
{
    Close();
}

void LibcameraStereoOV9281_TsPair::ResetPairingState()
{
    std::lock_guard<std::mutex> lk(m_muPair);
    m_qL.clear();
    m_qR.clear();
    m_paired.clear();
    m_lastDtMs.store(0, std::memory_order_relaxed);
    m_lastSeq.store(0, std::memory_order_relaxed);
    m_droppedPaired.store(0, std::memory_order_relaxed);
    m_lastRawSeq[0].store(0, std::memory_order_relaxed);
    m_lastRawSeq[1].store(0, std::memory_order_relaxed);
    m_rawFrameCount[0].store(0, std::memory_order_relaxed);
    m_rawFrameCount[1].store(0, std::memory_order_relaxed);
    m_lastRejectDtNs.store(0, std::memory_order_relaxed);
    m_droppedUnpaired[0].store(0, std::memory_order_relaxed);
    m_droppedUnpaired[1].store(0, std::memory_order_relaxed);
    m_lastArriveNs[0].store(0, std::memory_order_relaxed);
    m_lastArriveNs[1].store(0, std::memory_order_relaxed);
    m_lastPairMonoNs.store(0, std::memory_order_relaxed);
}
