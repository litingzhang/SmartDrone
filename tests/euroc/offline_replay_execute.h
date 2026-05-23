ReplayOutputAdjustment BuildReplayOutputAdjustment(
    const OfflineReplayOptions &opts)
{
    return {.timestampOffsetNs = EnvTimestampOffsetNs(),
            .positionScale = EnvOutputPositionScale(),
            .bodyFromCameraExtrinsics = LoadReplayBodyExtrinsics(opts)};
}

int RunOfflineReplay(const OfflineReplayOptions &opts)
{
    if (!opts.epgProfileOut.empty()) {
        return RunEpgProfileReplay(opts);
    }
    const SmartDrone::Tests::ReplayDataset dataset =
        SmartDrone::Tests::ReplayDataset::Load(opts.datasetRoot);
    if (dataset.Empty()) {
        std::cerr << "dataset is empty: " << opts.datasetRoot << "\n";
        return 1;
    }
    LogReplayStart(opts);
    if (!EnsureOrbSlamReplayAvailable(opts)) {
        return 2;
    }
    ReplayRuntime runtime(dataset);
    if (const int status = InitializeReplayRuntime(opts, runtime); status != 0) {
        return status;
    }
    PrepareReplayOutputDirectories(opts);
    std::ofstream realtimeCsv(opts.outputCsv);
    if (!realtimeCsv) {
        std::cerr << "failed to open output csv: " << opts.outputCsv << "\n";
        return 1;
    }
    WriteReplayCsvHeader(realtimeCsv);
    const ReplayOutputAdjustment adjustment = BuildReplayOutputAdjustment(opts);
    ReplayPoseSamples outputs =
        RunReplayFrames(opts, runtime, realtimeCsv, adjustment);
    realtimeCsv.flush();
    if (outputs.empty()) {
        return HandleEmptyReplayOutput(runtime);
    }
    WarnRealtimeLoopClosureDisabled(opts);
    const ReplaySummary summary = BuildReplaySummary(outputs);
    if (const int status = SaveFinalTrajectoryOrStop(opts, runtime); status != 0) {
        return status;
    }
    PrintReplaySummary(opts, adjustment, summary);
    if (const int status =
            WriteReplaySummaryJsonIfRequested(opts, summary, adjustment);
        status != 0) {
        return status;
    }
    StopReplayVisualFrontend(runtime);
    std::cout.flush();
    std::_Exit(0);
}
