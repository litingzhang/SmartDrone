struct DpvoPatchState {
    float x{0.0f};
    float y{0.0f};
    float invDepth{1.0f};
    float stereoPriorInvDepth{1.0f};
    bool hasStereoPrior{false};
};

struct DpvoFrameState {
    uint64_t frameId{0};
    int64_t timestampNs{0};
    Sophus::SE3f Tcw;
    std::vector<DpvoPatchState> patches;
    std::vector<float> fmap;
    std::vector<float> fmapLevel4;
    std::vector<float> patchImap;
    std::vector<float> patchGmap;
    int fmapChannels{0};
    int fmapHeight{0};
    int fmapWidth{0};
    int imapChannels{0};
    int imapHeight{0};
    int imapWidth{0};
    int imageWidth{0};
    int imageHeight{0};
};

struct DpvoEdgeState {
    int patchGlobal{0};
    int sourceFrame{0};
    int targetFrame{0};
};

struct DpvoIntrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
};

class DpvoGraphState {
  public:
    void Reset(int patchesPerFrame, int optimizationWindow)
    {
        m_patchesPerFrame = std::clamp(patchesPerFrame, 16, 256);
        m_optimizationWindow = std::clamp(optimizationWindow, 4, 32);
        m_patchLifetime = std::max(8, m_optimizationWindow + 4);
        m_removalWindow = std::max(12, m_optimizationWindow + 9);
        m_frames.clear();
        m_edges.clear();
        m_initialized = false;
        m_counter = 0;
        m_keyframeRemovals = 0;
        const DpvoGraphRuntimeOptions options = LoadDpvoGraphRuntimeOptions();
        m_persistentEdges = options.persistentEdges;
        m_keyframeRemovalEnabled = options.keyframeRemovalEnabled;
        m_capRebuiltEdges = options.capRebuiltEdges;
        m_maxActiveEdges = options.maxActiveEdges;
    }

    void PushFrame(uint64_t frameId, int64_t timestampNs, const cv::Mat &gray,
                   const Sophus::SE3f &initialPose,
                   const DpvoPatchifierRun &patchRun)
    {
        DpvoFrameState frame{};
        frame.frameId = frameId;
        frame.timestampNs = timestampNs;
        frame.Tcw = initialPose;
        frame.imageWidth = gray.cols;
        frame.imageHeight = gray.rows;
        if (patchRun.ok && patchRun.fmapHost != nullptr &&
            patchRun.fmapDims.nbDims == 4) {
            frame.fmapChannels = static_cast<int>(patchRun.fmapDims.d[1]);
            frame.fmapHeight = static_cast<int>(patchRun.fmapDims.d[2]);
            frame.fmapWidth = static_cast<int>(patchRun.fmapDims.d[3]);
            const size_t expected = static_cast<size_t>(frame.fmapChannels) *
                                    static_cast<size_t>(frame.fmapHeight) *
                                    static_cast<size_t>(frame.fmapWidth);
            if (expected > 0 && expected <= patchRun.fmapValueCount) {
                frame.fmap.assign(patchRun.fmapHost, patchRun.fmapHost + expected);
                frame.fmapLevel4 =
                    BuildPooledFmap(frame.fmap, frame.fmapChannels, frame.fmapHeight,
                                    frame.fmapWidth, 4);
            }
        }
        if (patchRun.ok && patchRun.imapHost != nullptr &&
            patchRun.imapDims.nbDims == 4) {
            frame.imapChannels = static_cast<int>(patchRun.imapDims.d[1]);
            frame.imapHeight = static_cast<int>(patchRun.imapDims.d[2]);
            frame.imapWidth = static_cast<int>(patchRun.imapDims.d[3]);
        }
        frame.patches = SelectPatches(frame, gray);
        if (frame.patches.size() < static_cast<size_t>(m_patchesPerFrame)) {
            frame.patches.resize(static_cast<size_t>(m_patchesPerFrame));
        }
        if (patchRun.ok && patchRun.imapHost != nullptr &&
            frame.imapChannels == kDim && frame.imapHeight > 0 &&
            frame.imapWidth > 0 &&
            patchRun.imapValueCount >= static_cast<size_t>(frame.imapChannels) *
                                           static_cast<size_t>(frame.imapHeight) *
                                           static_cast<size_t>(frame.imapWidth)) {
            frame.patchImap.assign(static_cast<size_t>(m_patchesPerFrame) * kDim,
                                   0.0f);
            for (int p = 0; p < m_patchesPerFrame; ++p) {
                SampleFeatureVector(patchRun.imapHost, frame.imapChannels,
                                    frame.imapHeight, frame.imapWidth,
                                    frame.patches[static_cast<size_t>(p)].x,
                                    frame.patches[static_cast<size_t>(p)].y,
                                    &frame.patchImap[static_cast<size_t>(p) * kDim]);
            }
        }
        if (!frame.fmap.empty() && frame.fmapChannels == kFmapChannels) {
            frame.patchGmap.assign(static_cast<size_t>(m_patchesPerFrame) *
                                       kFmapChannels * kPatchArea,
                                   0.0f);
            for (int p = 0; p < m_patchesPerFrame; ++p) {
                SampleFeaturePatch3(frame.fmap.data(), frame.fmapChannels,
                                    frame.fmapHeight, frame.fmapWidth,
                                    frame.patches[static_cast<size_t>(p)].x,
                                    frame.patches[static_cast<size_t>(p)].y,
                                    &frame.patchGmap[static_cast<size_t>(p) *
                                                     kFmapChannels * kPatchArea]);
            }
        }
        m_frames.push_back(std::move(frame));
        ++m_counter;
        if (m_frames.size() >= kInitializationFrames) {
            m_initialized = true;
        }
        if (!m_persistentEdges) {
            m_edges.clear();
        }
        AppendEdgesForNewest();
        PruneOldEdges();
        if (m_persistentEdges || m_capRebuiltEdges) {
            CapActiveEdges();
        }
        PruneFrames();
    }

    bool Initialized() const
    {
        return m_initialized;
    }
    int FrameCount() const
    {
        return static_cast<int>(m_frames.size());
    }
    int EdgeCount() const
    {
        return static_cast<int>(m_edges.size());
    }
    int PatchCount() const
    {
        return FrameCount() * m_patchesPerFrame;
    }
    int PatchesPerFrame() const
    {
        return m_patchesPerFrame;
    }
    int OptimizationWindow() const
    {
        return m_optimizationWindow;
    }
    int KeyframeRemovals() const
    {
        return m_keyframeRemovals;
    }
    const std::vector<DpvoFrameState> &Frames() const
    {
        return m_frames;
    }
    std::vector<DpvoFrameState> &MutableFrames()
    {
        return m_frames;
    }
    const std::vector<DpvoEdgeState> &Edges() const
    {
        return m_edges;
    }
    int LastStereoDepthUpdates() const
    {
        return m_lastStereoDepthUpdates;
    }
    void ApplyStereoDepthFromRightFmap(const DpvoPatchifierRun &rightRun,
                                       float fx, float baseline)
    {
        m_lastStereoDepthUpdates = 0;
        if (m_frames.empty() || !rightRun.ok || rightRun.fmapHost == nullptr ||
            rightRun.fmapDims.nbDims != 4 || !(fx > 0.0f) || !(baseline > 0.0f)) {
            return;
        }
        DpvoFrameState &frame = m_frames.back();
        const int rightChannels = static_cast<int>(rightRun.fmapDims.d[1]);
        const int rightHeight = static_cast<int>(rightRun.fmapDims.d[2]);
        const int rightWidth = static_cast<int>(rightRun.fmapDims.d[3]);
        const size_t expected = static_cast<size_t>(rightChannels) *
                                static_cast<size_t>(rightHeight) *
                                static_cast<size_t>(rightWidth);
        if (rightChannels != kFmapChannels || rightHeight != frame.fmapHeight ||
            rightWidth != frame.fmapWidth || rightRun.fmapValueCount < expected ||
            frame.patchGmap.size() < static_cast<size_t>(frame.patches.size()) *
                                         kFmapChannels * kPatchArea) {
            return;
        }

        int updated = 0;
        const DpvoStereoDepthOptions stereoOptions =
            LoadDpvoStereoDepthOptions(rightWidth);
        const int maxDisp = stereoOptions.maxDisparity;
        const float minScore = stereoOptions.minScore;
        const float minMargin = stereoOptions.minMargin;
        for (int p = 0; p < static_cast<int>(frame.patches.size()); ++p) {
            DpvoPatchState &patch = frame.patches[static_cast<size_t>(p)];
            if (patch.x < 2.0f || patch.y < 2.0f ||
                patch.x >= static_cast<float>(rightWidth - 2) ||
                patch.y >= static_cast<float>(rightHeight - 2)) {
                continue;
            }
            float bestScore = -std::numeric_limits<float>::infinity();
            float secondScore = -std::numeric_limits<float>::infinity();
            int bestDisp = 0;
            std::vector<float> scores(static_cast<size_t>(maxDisp + 1),
                                      -std::numeric_limits<float>::infinity());
            const int maxPatchDisp = std::min(
                maxDisp, std::max(1, static_cast<int>(std::floor(patch.x)) - 1));
            for (int disp = 1; disp <= maxPatchDisp; ++disp) {
                double ab = 0.0;
                double aa = 0.0;
                double bb = 0.0;
                const float x = patch.x - static_cast<float>(disp);
                const float y = patch.y;
                const size_t gmapOffset =
                    static_cast<size_t>(p) * kFmapChannels * kPatchArea;
                for (int c = 0; c < kFmapChannels; ++c) {
                    for (int py = 0; py < kPatchSize; ++py) {
                        for (int px = 0; px < kPatchSize; ++px) {
                            const size_t k = static_cast<size_t>(py * kPatchSize + px);
                            const float a =
                                frame.patchGmap[gmapOffset +
                                                static_cast<size_t>(c) * kPatchArea + k];
                            const float b = SampleFeatureBilinear(
                                rightRun.fmapHost, rightChannels, rightHeight, rightWidth, c,
                                x + static_cast<float>(px - kPatchRadius),
                                y + static_cast<float>(py - kPatchRadius));
                            ab += static_cast<double>(a) * static_cast<double>(b);
                            aa += static_cast<double>(a) * static_cast<double>(a);
                            bb += static_cast<double>(b) * static_cast<double>(b);
                        }
                    }
                }
                const float score = aa > 1e-9 && bb > 1e-9
                                        ? static_cast<float>(ab / std::sqrt(aa * bb))
                                        : -1.0f;
                scores[static_cast<size_t>(disp)] = score;
                if (score > bestScore) {
                    secondScore = bestScore;
                    bestScore = score;
                    bestDisp = disp;
                } else if (score > secondScore) {
                    secondScore = score;
                }
            }
            if (bestDisp > 0 && std::isfinite(bestScore) &&
                (!std::isfinite(secondScore) ||
                 bestScore > secondScore + minMargin) &&
                bestScore > minScore) {
                float refinedDisp = static_cast<float>(bestDisp);
                if (bestDisp > 1 && bestDisp < maxPatchDisp) {
                    const float left = scores[static_cast<size_t>(bestDisp - 1)];
                    const float center = scores[static_cast<size_t>(bestDisp)];
                    const float right = scores[static_cast<size_t>(bestDisp + 1)];
                    const float denom = left - 2.0f * center + right;
                    if (std::isfinite(denom) && std::fabs(denom) > 1e-6f) {
                        refinedDisp +=
                            std::clamp(0.5f * (left - right) / denom, -0.5f, 0.5f);
                    }
                }
                patch.invDepth =
                    std::clamp(refinedDisp / (fx * baseline), 1.0e-3f, 10.0f);
                patch.stereoPriorInvDepth = patch.invDepth;
                patch.hasStereoPrior = true;
                ++updated;
            }
        }
        if (updated == 0) {
            return;
        }
        m_lastStereoDepthUpdates = updated;
    }
    bool FeatureMapsReady() const
    {
        return !m_frames.empty() &&
               std::all_of(m_frames.begin(), m_frames.end(),
                           [](const DpvoFrameState &frame) {
                               return frame.patchImap.size() >=
                                          static_cast<size_t>(frame.patches.size()) *
                                              kDim &&
                                      frame.patchGmap.size() >=
                                          static_cast<size_t>(frame.patches.size()) *
                                              kFmapChannels * kPatchArea &&
                                      !frame.fmap.empty() &&
                                      !frame.fmapLevel4.empty();
                           });
    }
    const DpvoFrameState *PreviousFrame() const
    {
        return m_frames.size() >= 2 ? &m_frames[m_frames.size() - 2] : nullptr;
    }
    const DpvoFrameState *NewestFrame() const
    {
        return m_frames.empty() ? nullptr : &m_frames.back();
    }
    bool MaybeRemoveKeyframe(const DpvoIntrinsics &intrinsics)
    {
        if (!m_keyframeRemovalEnabled || !m_initialized ||
            FrameCount() <= kKeyframeIndex + 1) {
            PruneOldEdges();
            CapActiveEdges();
            return false;
        }
        const int n = FrameCount();
        const int i = n - kKeyframeIndex - 1;
        const int j = n - kKeyframeIndex + 1;
        if (i < 0 || j < 0 || i >= n || j >= n) {
            return false;
        }
        const float m =
            MotionMagnitude(i, j, intrinsics) + MotionMagnitude(j, i, intrinsics);
        if (!(std::isfinite(m) && 0.5f * m < kKeyframeThreshold)) {
            PruneOldEdges();
            CapActiveEdges();
            return false;
        }
        const int k = n - kKeyframeIndex;
        RemoveFrameAt(k);
        ++m_keyframeRemovals;
        PruneOldEdges();
        CapActiveEdges();
        return true;
    }

  private:
    static float FeatureAt(const float *data, int channels, int height, int width,
                           int c, int y, int x)
    {
        if (data == nullptr || c < 0 || c >= channels || y < 0 || y >= height ||
            x < 0 || x >= width) {
            return 0.0f;
        }
        const size_t idx = (static_cast<size_t>(c) * static_cast<size_t>(height) +
                            static_cast<size_t>(y)) *
                               static_cast<size_t>(width) +
                           static_cast<size_t>(x);
        return data[idx];
    }

    static float SampleFeatureBilinear(const float *data, int channels,
                                       int height, int width, int c, float x,
                                       float y)
    {
        if (data == nullptr || channels <= 0 || height <= 0 || width <= 0) {
            return 0.0f;
        }
        const int x0 = static_cast<int>(std::floor(x));
        const int y0 = static_cast<int>(std::floor(y));
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;
        const float dx = x - static_cast<float>(x0);
        const float dy = y - static_cast<float>(y0);
        const float v00 = FeatureAt(data, channels, height, width, c, y0, x0);
        const float v01 = FeatureAt(data, channels, height, width, c, y0, x1);
        const float v10 = FeatureAt(data, channels, height, width, c, y1, x0);
        const float v11 = FeatureAt(data, channels, height, width, c, y1, x1);
        return (1.0f - dy) * ((1.0f - dx) * v00 + dx * v01) +
               dy * ((1.0f - dx) * v10 + dx * v11);
    }

    static void SampleFeatureVector(const float *data, int channels, int height,
                                    int width, float x, float y, float *out)
    {
        if (out == nullptr) {
            return;
        }
        for (int c = 0; c < channels; ++c) {
            out[c] = SampleFeatureBilinear(data, channels, height, width, c, x, y);
        }
    }

    static void SampleFeaturePatch3(const float *data, int channels, int height,
                                    int width, float x, float y, float *out)
    {
        if (out == nullptr) {
            return;
        }
        for (int c = 0; c < channels; ++c) {
            for (int py = 0; py < kPatchSize; ++py) {
                for (int px = 0; px < kPatchSize; ++px) {
                    const float sx = x + static_cast<float>(px - kPatchRadius);
                    const float sy = y + static_cast<float>(py - kPatchRadius);
                    const size_t idx = (static_cast<size_t>(c) * kPatchArea) +
                                       static_cast<size_t>(py * kPatchSize + px);
                    out[idx] =
                        SampleFeatureBilinear(data, channels, height, width, c, sx, sy);
                }
            }
        }
    }

    static std::vector<float> BuildPooledFmap(const std::vector<float> &src,
                                              int channels, int height, int width,
                                              int level)
    {
        if (src.empty() || channels <= 0 || height <= 0 || width <= 0 ||
            level <= 1) {
            return src;
        }
        const int outHeight = height / level;
        const int outWidth = width / level;
        std::vector<float> dst(static_cast<size_t>(channels) *
                                   static_cast<size_t>(outHeight) *
                                   static_cast<size_t>(outWidth),
                               0.0f);
        if (outHeight <= 0 || outWidth <= 0) {
            return {};
        }
        const float denom = 1.0f / static_cast<float>(level * level);
        for (int c = 0; c < channels; ++c) {
            for (int y = 0; y < outHeight; ++y) {
                for (int x = 0; x < outWidth; ++x) {
                    float sum = 0.0f;
                    for (int dy = 0; dy < level; ++dy) {
                        for (int dx = 0; dx < level; ++dx) {
                            sum += src[(static_cast<size_t>(c) * static_cast<size_t>(height) +
                                        static_cast<size_t>(y * level + dy)) *
                                           static_cast<size_t>(width) +
                                       static_cast<size_t>(x * level + dx)];
                        }
                    }
                    dst[(static_cast<size_t>(c) * static_cast<size_t>(outHeight) +
                         static_cast<size_t>(y)) *
                            static_cast<size_t>(outWidth) +
                        static_cast<size_t>(x)] = sum * denom;
                }
            }
        }
        return dst;
    }

    std::vector<DpvoPatchState> SelectPatches(const DpvoFrameState &frame,
                                              const cv::Mat &gray) const
    {
        std::vector<DpvoPatchState> patches;
        patches.reserve(static_cast<size_t>(m_patchesPerFrame));
        const int width =
            frame.fmapWidth > 2 ? frame.fmapWidth : std::max(3, gray.cols / 4);
        const int height =
            frame.fmapHeight > 2 ? frame.fmapHeight : std::max(3, gray.rows / 4);
        std::mt19937 rng(
            static_cast<uint32_t>(0x9e3779b9U ^ (m_counter * 0x85ebca6bU)));
        std::uniform_int_distribution<int> xDist(1, std::max(1, width - 2));
        std::uniform_int_distribution<int> yDist(1, std::max(1, height - 2));
        std::uniform_real_distribution<float> depthDist(0.2f, 1.0f);
        for (int i = 0; i < m_patchesPerFrame; ++i) {
            float invDepth = depthDist(rng);
            if (m_initialized && !m_frames.empty()) {
                invDepth = MedianRecentDepth();
            }
            patches.push_back({static_cast<float>(xDist(rng)),
                               static_cast<float>(yDist(rng)), invDepth, invDepth,
                               false});
        }
        return patches;
    }

    float MedianRecentDepth() const
    {
        std::vector<float> depths;
        const int start = std::max(0, FrameCount() - 3);
        for (int i = start; i < FrameCount(); ++i) {
            for (const DpvoPatchState &patch :
                 m_frames[static_cast<size_t>(i)].patches) {
                if (std::isfinite(patch.invDepth) && patch.invDepth > 1e-3f &&
                    patch.invDepth < 10.0f) {
                    depths.push_back(patch.invDepth);
                }
            }
        }
        if (depths.empty()) {
            return 1.0f;
        }
        const size_t mid = depths.size() / 2;
        std::nth_element(depths.begin(),
                         depths.begin() + static_cast<std::ptrdiff_t>(mid),
                         depths.end());
        return std::clamp(depths[mid], 1e-3f, 10.0f);
    }

    static std::array<float, 2>
    ProjectPatchCenter(const DpvoFrameState &source, const DpvoFrameState &target,
                       const DpvoPatchState &patch,
                       const Eigen::Matrix3f &overrideR, bool useOverrideR,
                       const DpvoIntrinsics &intrinsics, bool *valid)
    {
        if (valid != nullptr) {
            *valid = false;
        }
        if (!(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
            return {patch.x, patch.y};
        }
        const Sophus::SE3f Tji = target.Tcw * source.Tcw.inverse();
        const Eigen::Matrix3f R = useOverrideR ? overrideR : Tji.so3().matrix();
        const Eigen::Vector3f t = Tji.translation();
        const Eigen::Vector3f Xi((patch.x - intrinsics.cx) / intrinsics.fx,
                                 (patch.y - intrinsics.cy) / intrinsics.fy, 1.0f);
        const Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
        if (!(Xj.z() > 0.2f) || !Xj.allFinite()) {
            return {patch.x, patch.y};
        }
        if (valid != nullptr) {
            *valid = true;
        }
        return {intrinsics.fx * (Xj.x() / Xj.z()) + intrinsics.cx,
                intrinsics.fy * (Xj.y() / Xj.z()) + intrinsics.cy};
    }

    float MotionMagnitude(int sourceFrame, int targetFrame,
                          const DpvoIntrinsics &intrinsics) const
    {
        if (sourceFrame < 0 || targetFrame < 0 || sourceFrame >= FrameCount() ||
            targetFrame >= FrameCount()) {
            return std::numeric_limits<float>::infinity();
        }
        const DpvoFrameState &source = m_frames[static_cast<size_t>(sourceFrame)];
        const DpvoFrameState &target = m_frames[static_cast<size_t>(targetFrame)];
        if (!(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
            return std::numeric_limits<float>::infinity();
        }
        double sum = 0.0;
        int count = 0;
        const Eigen::Matrix3f identityR = Eigen::Matrix3f::Identity();
        for (const DpvoPatchState &patch : source.patches) {
            bool validFull = false;
            bool validTonly = false;
            const std::array<float, 2> full = ProjectPatchCenter(
                source, target, patch, identityR, false, intrinsics, &validFull);
            const std::array<float, 2> tonly = ProjectPatchCenter(
                source, target, patch, identityR, true, intrinsics, &validTonly);
            if (!validFull || !validTonly) {
                continue;
            }
            const float dxFull = full[0] - patch.x;
            const float dyFull = full[1] - patch.y;
            const float dxT = tonly[0] - patch.x;
            const float dyT = tonly[1] - patch.y;
            sum += 0.5 * std::sqrt(
                             static_cast<double>(dxFull * dxFull + dyFull * dyFull)) +
                   0.5 * std::sqrt(static_cast<double>(dxT * dxT + dyT * dyT));
            ++count;
        }
        return count > 0 ? static_cast<float>(sum / static_cast<double>(count))
                         : std::numeric_limits<float>::infinity();
    }

    void AppendEdgesForNewest()
    {
        const int n = FrameCount();
        if (n <= 1) {
            return;
        }
        const int newest = n - 1;
        const int forwardPatchStart =
            m_patchesPerFrame * std::max(newest - m_patchLifetime, 0);
        const int forwardPatchEnd = m_patchesPerFrame * std::max(newest, 0);
        for (int k = forwardPatchStart; k < forwardPatchEnd; ++k) {
            m_edges.push_back({k, k / m_patchesPerFrame, newest});
        }
        const int backPatchStart = m_patchesPerFrame * newest;
        const int backPatchEnd = m_patchesPerFrame * n;
        const int backTargetStart = std::max(n - m_patchLifetime, 0);
        for (int k = backPatchStart; k < backPatchEnd; ++k) {
            for (int j = backTargetStart; j < n; ++j) {
                m_edges.push_back({k, newest, j});
            }
        }
    }

    void PruneOldEdges()
    {
        const int n = FrameCount();
        const int oldestActivePatchFrame = std::max(n - m_removalWindow, 0);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                     [&](const DpvoEdgeState &edge) {
                                         const int patchFrame =
                                             edge.patchGlobal / m_patchesPerFrame;
                                         return edge.sourceFrame < 0 ||
                                                edge.targetFrame < 0 ||
                                                edge.sourceFrame >= n ||
                                                edge.targetFrame >= n ||
                                                patchFrame < oldestActivePatchFrame ||
                                                patchFrame >= n;
                                     }),
                      m_edges.end());
    }

    void CapActiveEdges()
    {
        if ((!m_persistentEdges && !m_capRebuiltEdges) || m_maxActiveEdges <= 0 ||
            static_cast<int>(m_edges.size()) <= m_maxActiveEdges) {
            return;
        }
        const int removeCount = static_cast<int>(m_edges.size()) - m_maxActiveEdges;
        m_edges.erase(m_edges.begin(), m_edges.begin() + removeCount);
    }

    void RemoveFrameAt(int frameIndex)
    {
        if (frameIndex < 0 || frameIndex >= FrameCount()) {
            return;
        }
        m_frames.erase(m_frames.begin() + frameIndex);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                     [&](DpvoEdgeState &edge) {
                                         const int patchFrame =
                                             edge.patchGlobal / m_patchesPerFrame;
                                         if (edge.sourceFrame == frameIndex ||
                                             edge.targetFrame == frameIndex ||
                                             patchFrame == frameIndex) {
                                             return true;
                                         }
                                         if (edge.sourceFrame > frameIndex) {
                                             --edge.sourceFrame;
                                         }
                                         if (edge.targetFrame > frameIndex) {
                                             --edge.targetFrame;
                                         }
                                         if (patchFrame > frameIndex) {
                                             edge.patchGlobal -= m_patchesPerFrame;
                                         }
                                         return false;
                                     }),
                      m_edges.end());
    }

    void PruneFrames()
    {
        const int maxFrames =
            std::max(36, m_removalWindow + m_optimizationWindow + 4);
        if (FrameCount() <= maxFrames) {
            return;
        }
        const int drop = FrameCount() - maxFrames;
        m_frames.erase(m_frames.begin(), m_frames.begin() + drop);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                     [&](DpvoEdgeState &edge) {
                                         const int patchFrame =
                                             edge.patchGlobal / m_patchesPerFrame;
                                         if (edge.sourceFrame < drop ||
                                             edge.targetFrame < drop ||
                                             patchFrame < drop) {
                                             return true;
                                         }
                                         edge.sourceFrame -= drop;
                                         edge.targetFrame -= drop;
                                         edge.patchGlobal -= drop * m_patchesPerFrame;
                                         return false;
                                     }),
                      m_edges.end());
        PruneOldEdges();
        CapActiveEdges();
    }

    static constexpr int kInitializationFrames = 8;
    static constexpr int kDim = 384;
    static constexpr int kFmapChannels = 128;
    static constexpr int kPatchSize = 3;
    static constexpr int kPatchRadius = 1;
    static constexpr int kPatchArea = kPatchSize * kPatchSize;
    static constexpr int kKeyframeIndex = 4;
    static constexpr float kKeyframeThreshold = 15.0f;
    int m_patchesPerFrame{48};
    int m_optimizationWindow{7};
    int m_patchLifetime{11};
    int m_removalWindow{16};
    int m_maxActiveEdges{4096};
    int m_counter{0};
    int m_lastStereoDepthUpdates{0};
    int m_keyframeRemovals{0};
    bool m_persistentEdges{false};
    bool m_keyframeRemovalEnabled{false};
    bool m_capRebuiltEdges{false};
    bool m_initialized{false};
    std::vector<DpvoFrameState> m_frames;
    std::vector<DpvoEdgeState> m_edges;
};

struct DpvoEdgeKey {
    uint64_t sourceFrameId{0};
    uint64_t targetFrameId{0};
    int patchLocal{0};

    bool operator==(const DpvoEdgeKey &other) const
    {
        return sourceFrameId == other.sourceFrameId &&
               targetFrameId == other.targetFrameId &&
               patchLocal == other.patchLocal;
    }
};

struct DpvoEdgeKeyHash {
    size_t operator()(const DpvoEdgeKey &key) const
    {
        uint64_t x = key.sourceFrameId + 0x9e3779b97f4a7c15ULL;
        x ^= key.targetFrameId + 0x9e3779b97f4a7c15ULL + (x << 6U) + (x >> 2U);
        x ^= static_cast<uint64_t>(key.patchLocal + 0x9e3779b9) + (x << 6U) +
             (x >> 2U);
        x ^= x >> 30U;
        x *= 0xbf58476d1ce4e5b9ULL;
        x ^= x >> 27U;
        x *= 0x94d049bb133111ebULL;
        x ^= x >> 31U;
        return static_cast<size_t>(x);
    }
};
