const Epg::TaskConfig *FindRuntimeTopologyTask(
    const Epg::GraphConfig &config,
    const std::string &name)
{
    for (const auto &task : config.tasks) {
        if (task.name == name) {
            return &task;
        }
    }
    return nullptr;
}

void ExpectSlamTimingTopology(const Epg::GraphConfig &config)
{
    const auto *resource = FindRuntimeTopologyTask(config, "SlamResourceTask");
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->trigger.mode, Epg::TriggerMode::Periodic);
    EXPECT_EQ(resource->trigger.interval, std::chrono::milliseconds(100));

    const auto *clock = FindRuntimeTopologyTask(config, "SlamClockTask");
    ASSERT_NE(clock, nullptr);
    EXPECT_EQ(clock->trigger.mode, Epg::TriggerMode::Periodic);
    EXPECT_EQ(clock->trigger.interval, std::chrono::milliseconds(50));

    const auto *backendTick =
        FindRuntimeTopologyTask(config, "SlamBackendTickTask");
    ASSERT_NE(backendTick, nullptr);
    EXPECT_TRUE(backendTick->inputs.empty());
    EXPECT_EQ(backendTick->trigger.mode, Epg::TriggerMode::Periodic);
    EXPECT_EQ(backendTick->trigger.interval, std::chrono::milliseconds(5));
    EXPECT_TRUE(backendTick->trigger.queues.empty());
}

void ExpectSlamIngestTopology(const Epg::GraphConfig &config)
{
    const auto *imuPoll = FindRuntimeTopologyTask(config, "SlamImuPollTask");
    ASSERT_NE(imuPoll, nullptr);
    EXPECT_EQ(imuPoll->inputs.at(0),
              "SlamResourceTask_0_to_SlamImuPollTask_0");
    EXPECT_EQ(imuPoll->trigger.mode,
              Epg::TriggerMode::PeriodicOrAnyQueueReady);
    EXPECT_EQ(imuPoll->trigger.interval, std::chrono::milliseconds(1));
    EXPECT_FALSE(imuPoll->scheduling.realtime);
    EXPECT_EQ(imuPoll->scheduling.priority, 0);
    EXPECT_EQ(
        imuPoll->trigger.queues,
        (std::vector<std::string>{
            "SlamResourceTask_0_to_SlamImuPollTask_0"}));

    const auto *imuGate = FindRuntimeTopologyTask(config, "SlamImuGateTask");
    ASSERT_NE(imuGate, nullptr);
    EXPECT_EQ(imuGate->inputs.at(0),
              "SlamImuPollTask_0_to_SlamImuGateTask_0");
    EXPECT_EQ(imuGate->inputs.at(1),
              "SlamClockTask_0_to_SlamImuGateTask_1");
    EXPECT_EQ(
        imuGate->trigger.queues,
        (std::vector<std::string>{
            "SlamImuPollTask_0_to_SlamImuGateTask_0",
            "SlamClockTask_0_to_SlamImuGateTask_1"}));
}

void ExpectSlamTrackingTopology(const Epg::GraphConfig &config)
{
    const auto *acquire = FindRuntimeTopologyTask(config, "SlamAcquireTask");
    ASSERT_NE(acquire, nullptr);
    EXPECT_EQ(acquire->inputs.at(0),
              "SlamImuGateTask_0_to_SlamAcquireTask_0");
    EXPECT_EQ(
        acquire->trigger.queues,
        (std::vector<std::string>{
            "SlamImuGateTask_0_to_SlamAcquireTask_0"}));

    const auto *tracking =
        FindRuntimeTopologyTask(config, "SlamTrackingTask");
    ASSERT_NE(tracking, nullptr);
    EXPECT_EQ(tracking->inputs.at(0),
              "SlamAcquireTask_0_to_SlamTrackingTask_0");
    EXPECT_EQ(tracking->outputs.at(0),
              "SlamTrackingTask_0_to_SlamPosePostprocessTask_0");
    EXPECT_EQ(
        tracking->trigger.queues,
        (std::vector<std::string>{
            "SlamAcquireTask_0_to_SlamTrackingTask_0"}));
}

void ExpectSlamOutputTopology(const Epg::GraphConfig &config)
{
    const auto *posePostprocess =
        FindRuntimeTopologyTask(config, "SlamPosePostprocessTask");
    ASSERT_NE(posePostprocess, nullptr);
    EXPECT_EQ(posePostprocess->inputs.at(0),
              "SlamTrackingTask_0_to_SlamPosePostprocessTask_0");

    const auto *pointCloud =
        FindRuntimeTopologyTask(config, "SlamPointCloudTask");
    ASSERT_NE(pointCloud, nullptr);
    EXPECT_EQ(pointCloud->inputs.at(0),
              "SlamPosePostprocessTask_0_to_SlamPointCloudTask_0");

    const auto *livePose = FindRuntimeTopologyTask(config, "SlamLivePoseTask");
    ASSERT_NE(livePose, nullptr);
    EXPECT_EQ(livePose->inputs.at(0),
              "SlamPosePostprocessTask_1_to_SlamLivePoseTask_0");
}

void ExpectSlamPublishTopology(const Epg::GraphConfig &config)
{
    const auto *mavlink = FindRuntimeTopologyTask(config, "SlamMavlinkTask");
    ASSERT_NE(mavlink, nullptr);
    EXPECT_EQ(mavlink->inputs.at(0),
              "SlamPosePostprocessTask_2_to_SlamMavlinkTask_0");

    const auto *udp = FindRuntimeTopologyTask(config, "SlamUdpTask");
    ASSERT_NE(udp, nullptr);
    EXPECT_EQ(udp->inputs.at(0),
              "SlamPosePostprocessTask_3_to_SlamUdpTask_0");

    const auto *dfx = FindRuntimeTopologyTask(config, "SlamDfxTask");
    ASSERT_NE(dfx, nullptr);
    EXPECT_EQ(dfx->inputs.at(0),
              "SlamPosePostprocessTask_4_to_SlamDfxTask_0");

    const auto *dfxSnapshot =
        FindRuntimeTopologyTask(config, "SlamGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));
}

void ExpectSlamMonitorTopology(const Epg::GraphConfig &config)
{
    const auto *monitor = FindRuntimeTopologyTask(config, "SlamMonitorTask");
    ASSERT_NE(monitor, nullptr);
    EXPECT_EQ(monitor->inputs.at(8),
              "SlamUdpTask_1_to_SlamMonitorTask_8");
    EXPECT_EQ(monitor->inputs.at(9),
              "SlamPreviewTxTask_1_to_SlamMonitorTask_9");
    EXPECT_NE(std::find(monitor->trigger.queues.begin(),
                        monitor->trigger.queues.end(),
                        "SlamUdpTask_1_to_SlamMonitorTask_8"),
              monitor->trigger.queues.end());
    EXPECT_NE(std::find(monitor->trigger.queues.begin(),
                        monitor->trigger.queues.end(),
                        "SlamPreviewTxTask_1_to_SlamMonitorTask_9"),
              monitor->trigger.queues.end());
}

void ExpectSystemIngressTopology(const Epg::GraphConfig &config)
{
    const auto *vehicleTelemetryRx =
        FindRuntimeTopologyTask(config, "VehicleTelemetryRxTask");
    ASSERT_NE(vehicleTelemetryRx, nullptr);
    EXPECT_EQ(vehicleTelemetryRx->trigger.interval,
              std::chrono::milliseconds(2));
    EXPECT_EQ(vehicleTelemetryRx->outputs.at(0),
              "VehicleTelemetryRxTask_0_to_SetpointStreamTask_0");

    const auto *setpointStream =
        FindRuntimeTopologyTask(config, "SetpointStreamTask");
    ASSERT_NE(setpointStream, nullptr);
    EXPECT_EQ(setpointStream->trigger.interval,
              std::chrono::milliseconds(5));
    EXPECT_EQ(setpointStream->inputs.at(0),
              "VehicleTelemetryRxTask_0_to_SetpointStreamTask_0");
    EXPECT_EQ(setpointStream->outputs.at(0),
              "SetpointStreamTask_0_to_UdpReceiveTask_0");

    const auto *udpReceive = FindRuntimeTopologyTask(config, "UdpReceiveTask");
    ASSERT_NE(udpReceive, nullptr);
    EXPECT_EQ(udpReceive->trigger.interval, std::chrono::milliseconds(2));
    EXPECT_EQ(udpReceive->inputs.at(0),
              "SetpointStreamTask_0_to_UdpReceiveTask_0");
    EXPECT_EQ(udpReceive->outputs.at(0),
              "UdpReceiveTask_0_to_UdpHeartbeatTxTask_0");
}

void ExpectSystemPeriodicTopology(const Epg::GraphConfig &config)
{
    const auto *udpHeartbeat =
        FindRuntimeTopologyTask(config, "UdpHeartbeatTxTask");
    ASSERT_NE(udpHeartbeat, nullptr);
    EXPECT_EQ(udpHeartbeat->trigger.interval,
              std::chrono::milliseconds(500));

    const auto *udpTimeout =
        FindRuntimeTopologyTask(config, "UdpHeartbeatTimeoutTask");
    ASSERT_NE(udpTimeout, nullptr);
    EXPECT_EQ(udpTimeout->trigger.interval, std::chrono::milliseconds(100));

    const auto *udpState = FindRuntimeTopologyTask(config, "UdpStateTxTask");
    ASSERT_NE(udpState, nullptr);
    EXPECT_EQ(udpState->trigger.interval, std::chrono::milliseconds(100));

    const auto *udpPointCloud =
        FindRuntimeTopologyTask(config, "UdpPointCloudTxTask");
    ASSERT_NE(udpPointCloud, nullptr);
    EXPECT_EQ(udpPointCloud->trigger.interval,
              std::chrono::milliseconds(100));
    EXPECT_EQ(udpPointCloud->outputs.at(0),
              "UdpPointCloudTxTask_0_to_ManualControlTask_0");
}

void ExpectSystemControlTopology(const Epg::GraphConfig &config)
{
    const auto *manualControl =
        FindRuntimeTopologyTask(config, "ManualControlTask");
    ASSERT_NE(manualControl, nullptr);
    EXPECT_EQ(manualControl->trigger.interval,
              std::chrono::milliseconds(50));
    EXPECT_EQ(manualControl->inputs.at(0),
              "UdpPointCloudTxTask_0_to_ManualControlTask_0");

    const auto *forceRestart =
        FindRuntimeTopologyTask(config, "ForceRestartTask");
    ASSERT_NE(forceRestart, nullptr);
    EXPECT_EQ(forceRestart->trigger.interval, std::chrono::milliseconds(50));

    const auto *supervisor =
        FindRuntimeTopologyTask(config, "RuntimeSupervisorTask");
    ASSERT_NE(supervisor, nullptr);
    EXPECT_EQ(supervisor->trigger.interval, std::chrono::milliseconds(100));
}

void ExpectSystemRedeployTopology(const Epg::GraphConfig &config)
{
    const auto *redeploy = FindRuntimeTopologyTask(config, "EpgRedeployTask");
    ASSERT_NE(redeploy, nullptr);
    EXPECT_EQ(redeploy->type, "EpgRedeployTask");
    EXPECT_EQ(redeploy->trigger.interval, std::chrono::milliseconds(500));
    EXPECT_EQ(redeploy->inputs.at(0),
              "RuntimeSupervisorTask_0_to_EpgRedeployTask_0");

    const auto *dfxSnapshot =
        FindRuntimeTopologyTask(config, "EpgDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));
    EXPECT_EQ(dfxSnapshot->inputs.at(0),
              "DiscoveryBeaconTask_0_to_EpgDfxSnapshotTask_0");
    EXPECT_EQ(dfxSnapshot->outputs.at(0),
              "EpgDfxSnapshotTask_0_to_EpgOptimizeTask_0");

    const auto *optimizer = FindRuntimeTopologyTask(config, "EpgOptimizeTask");
    ASSERT_NE(optimizer, nullptr);
    EXPECT_EQ(optimizer->type, "EpgOptimizeTask");
    EXPECT_EQ(optimizer->trigger.interval, std::chrono::milliseconds(5000));
    EXPECT_EQ(optimizer->inputs.at(0),
              "EpgDfxSnapshotTask_0_to_EpgOptimizeTask_0");
}

void ExpectCalibCaptureTopology(const Epg::GraphConfig &config)
{
    const auto *camera =
        FindRuntimeTopologyTask(config, "CalibCameraAcquireTask");
    ASSERT_NE(camera, nullptr);
    EXPECT_EQ(camera->inputs.at(0),
              "CalibResourceTask_0_to_CalibCameraAcquireTask_0");
    EXPECT_EQ(camera->inputs.at(1),
              "CalibClockTask_0_to_CalibCameraAcquireTask_1");

    const auto *pace =
        FindRuntimeTopologyTask(config, "CalibPacingFilterTask");
    ASSERT_NE(pace, nullptr);
    EXPECT_EQ(pace->inputs.at(0),
              "CalibCameraAcquireTask_0_to_CalibPacingFilterTask_0");

    const auto *preview =
        FindRuntimeTopologyTask(config, "CalibUdpPreviewTask");
    ASSERT_NE(preview, nullptr);
    EXPECT_EQ(preview->inputs.at(0),
              "CalibCameraAcquireTask_1_to_CalibUdpPreviewTask_0");
}

void ExpectCalibStorageTopology(const Epg::GraphConfig &config)
{
    const auto *storage =
        FindRuntimeTopologyTask(config, "CalibStorageWriteTask");
    ASSERT_NE(storage, nullptr);
    EXPECT_EQ(storage->inputs.at(0),
              "CalibPacingFilterTask_0_to_CalibStorageWriteTask_0");
    EXPECT_EQ(storage->inputs.at(1),
              "CalibImuWriterTask_0_to_CalibStorageWriteTask_1");
    EXPECT_EQ(storage->inputs.at(2),
              "CalibCompletionTask_0_to_CalibStorageWriteTask_2");
    EXPECT_EQ(storage->outputs.at(0),
              "CalibStorageWriteTask_0_to_CalibFlushSyncTask_0");

    const auto *flush = FindRuntimeTopologyTask(config, "CalibFlushSyncTask");
    ASSERT_NE(flush, nullptr);
    EXPECT_EQ(flush->inputs.at(0),
              "CalibStorageWriteTask_0_to_CalibFlushSyncTask_0");
}

void ExpectCalibCompletionTopology(const Epg::GraphConfig &config)
{
    const auto *completion =
        FindRuntimeTopologyTask(config, "CalibCompletionTask");
    ASSERT_NE(completion, nullptr);
    EXPECT_EQ(completion->inputs.at(1),
              "CalibUdpPreviewTask_0_to_CalibCompletionTask_1");
    EXPECT_EQ(completion->inputs.at(2),
              "CalibResourceTask_2_to_CalibCompletionTask_2");
    EXPECT_EQ(completion->outputs.at(0),
              "CalibCompletionTask_0_to_CalibStorageWriteTask_2");
    EXPECT_NE(std::find(completion->trigger.queues.begin(),
                        completion->trigger.queues.end(),
                        "CalibResourceTask_2_to_CalibCompletionTask_2"),
              completion->trigger.queues.end());

    const auto *dfxSnapshot =
        FindRuntimeTopologyTask(config, "CalibGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));
}
