TEST(EventPipelineGraphTopology, RunsLinearChainTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "chain_pipeline.json",
        {"source_to_forward", "forward_to_sink"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphTopology, RunsParallelIndependentTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "parallel_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(EventPipelineGraphTopology, RunsFanInJoinTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "fanin_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "right_source", "join_sink"});
}

TEST(EventPipelineGraphTopology, RunsDiamondTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "diamond_pipeline.json",
        {"left_packets", "right_packets", "forwarded_left_packets"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(EventPipelineGraphMermaid, ConvertsMermaidTopologyToRuntimeConfig)
{
    auto registry = MakeRegistry();
    const auto config = Epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1; resource=cpu; cpu_affinity=-1; budget_us=500; deadline_us=900; backpressure_outputs=0; realtime=true; priority=42"]
        forward["type=TestForwardTask; trigger=any_queue_ready"]
        sink["type=TestSinkTask; trigger=any_queue_ready"]

        source -->|"type=TestPacket; depth=8; overflow=drop_newest"| forward
        forward -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
    )",
                                                     registry);

    ASSERT_EQ(config.queues.size(), 2u);
    EXPECT_EQ(config.queues[0].name, "source_0_to_forward_0");
    EXPECT_EQ(config.queues[0].type, "TestPacket");
    EXPECT_EQ(config.queues[0].depth, 8u);
    EXPECT_EQ(config.queues[1].name, "forward_0_to_sink_0");

    ASSERT_EQ(config.tasks.size(), 3u);
    EXPECT_EQ(config.tasks[0].name, "source");
    EXPECT_EQ(config.tasks[0].scheduling.resource, "cpu");
    EXPECT_EQ(config.tasks[0].scheduling.cpuAffinity, -1);
    EXPECT_EQ(config.tasks[0].scheduling.budgetUs, 500u);
    EXPECT_EQ(config.tasks[0].scheduling.deadlineUs, 900u);
    EXPECT_EQ(config.tasks[0].scheduling.backpressureOutputs,
              std::vector<Epg::PortId>{0});
    EXPECT_TRUE(config.tasks[0].scheduling.realtime);
    EXPECT_EQ(config.tasks[0].scheduling.priority, 42);
    EXPECT_EQ(config.tasks[0].outputs.at(0), "source_0_to_forward_0");
    EXPECT_EQ(config.tasks[1].inputs.at(0), "source_0_to_forward_0");
    EXPECT_EQ(config.tasks[1].outputs.at(0), "forward_0_to_sink_0");
    EXPECT_EQ(config.tasks[1].trigger.queues, std::vector<std::string>{"source_0_to_forward_0"});
    EXPECT_EQ(config.tasks[2].inputs.at(0), "forward_0_to_sink_0");
    EXPECT_EQ(config.tasks[2].trigger.queues, std::vector<std::string>{"forward_0_to_sink_0"});
}

TEST(EventPipelineGraphDot, RunsTopologyCompiledFromDotFile)
{
    RunTopologyFromDotFile(
        "chain_pipeline.dot",
        {"source_0_to_forward_0", "forward_0_to_sink_0"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphMermaid, ConvertsMarkdownMermaidBlockToRuntimeConfig)
{
    auto registry = MakeRegistry();
    const auto config = Epg::ParseGraphConfigMermaid(R"(
# EventPipelineGraph

```mermaid
flowchart LR
  source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
  sink["type=TestSinkTask; trigger=any_queue_ready"]
  source -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
```
    )",
                                                     registry);

    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues[0].name, "source_0_to_sink_0");
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks[0].outputs.at(0), "source_0_to_sink_0");
    EXPECT_EQ(config.tasks[1].inputs.at(0), "source_0_to_sink_0");
}

TEST(EventPipelineGraphDot, CompilesSlamSubgraphFromMaintainedTopology)
{
    auto registry = MakeSlamShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_slam_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 20u);
    ASSERT_EQ(config.queues.size(), 35u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string &name) -> const Epg::TaskConfig * {
        for (const auto &task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto *resource = findTask("SlamResourceTask");
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->trigger.mode, Epg::TriggerMode::Periodic);
    EXPECT_EQ(resource->trigger.interval, std::chrono::milliseconds(100));

    const auto *clock = findTask("SlamClockTask");
    ASSERT_NE(clock, nullptr);
    EXPECT_EQ(clock->trigger.mode, Epg::TriggerMode::Periodic);
    EXPECT_EQ(clock->trigger.interval, std::chrono::milliseconds(50));

    const auto *imuPoll = findTask("SlamImuPollTask");
    ASSERT_NE(imuPoll, nullptr);
    EXPECT_EQ(imuPoll->inputs.at(0), "SlamResourceTask_0_to_SlamImuPollTask_0");
    EXPECT_EQ(imuPoll->trigger.mode, Epg::TriggerMode::PeriodicOrAnyQueueReady);
    EXPECT_EQ(imuPoll->trigger.interval, std::chrono::milliseconds(1));
    EXPECT_FALSE(imuPoll->scheduling.realtime);
    EXPECT_EQ(imuPoll->scheduling.priority, 0);
    EXPECT_EQ(imuPoll->trigger.queues,
              (std::vector<std::string>{"SlamResourceTask_0_to_SlamImuPollTask_0"}));

    const auto *backendTick = findTask("SlamBackendTickTask");
    ASSERT_NE(backendTick, nullptr);
    EXPECT_TRUE(backendTick->inputs.empty());
    EXPECT_EQ(backendTick->trigger.mode, Epg::TriggerMode::Periodic);
    EXPECT_EQ(backendTick->trigger.interval, std::chrono::milliseconds(5));
    EXPECT_TRUE(backendTick->trigger.queues.empty());

    const auto *imuGate = findTask("SlamImuGateTask");
    ASSERT_NE(imuGate, nullptr);
    EXPECT_EQ(imuGate->inputs.at(0), "SlamImuPollTask_0_to_SlamImuGateTask_0");
    EXPECT_EQ(imuGate->inputs.at(1), "SlamClockTask_0_to_SlamImuGateTask_1");
    EXPECT_EQ(imuGate->trigger.queues,
              (std::vector<std::string>{"SlamImuPollTask_0_to_SlamImuGateTask_0",
                                        "SlamClockTask_0_to_SlamImuGateTask_1"}));

    const auto *acquire = findTask("SlamAcquireTask");
    ASSERT_NE(acquire, nullptr);
    EXPECT_EQ(acquire->inputs.at(0),
              "SlamImuGateTask_0_to_SlamAcquireTask_0");
    EXPECT_EQ(acquire->trigger.queues,
              (std::vector<std::string>{"SlamImuGateTask_0_to_SlamAcquireTask_0"}));

    const auto *route = findTask("SlamTrackingRouteTask");
    ASSERT_NE(route, nullptr);
    EXPECT_EQ(route->inputs.at(0),
              "SlamAcquireTask_0_to_SlamTrackingRouteTask_0");
    EXPECT_EQ(route->outputs.at(0),
              "SlamTrackingRouteTask_0_to_SlamKltTrackingTask_0");
    EXPECT_EQ(route->outputs.at(1),
              "SlamTrackingRouteTask_1_to_SlamDpvoTrackingTask_0");
    EXPECT_EQ(route->outputs.at(2),
              "SlamTrackingRouteTask_2_to_SlamOrbTrackingTask_0");
    EXPECT_EQ(route->outputs.at(3),
              "SlamTrackingRouteTask_3_to_SlamVisualFeatureTrackingTask_0");
    EXPECT_EQ(route->trigger.queues,
              (std::vector<std::string>{"SlamAcquireTask_0_to_SlamTrackingRouteTask_0"}));

    const auto *posePostprocess = findTask("SlamPosePostprocessTask");
    ASSERT_NE(posePostprocess, nullptr);
    EXPECT_EQ(posePostprocess->inputs.at(0),
              "SlamKltTrackingTask_0_to_SlamPosePostprocessTask_0");
    EXPECT_EQ(posePostprocess->inputs.at(1),
              "SlamDpvoTrackingTask_0_to_SlamPosePostprocessTask_1");
    EXPECT_EQ(posePostprocess->inputs.at(2),
              "SlamOrbTrackingTask_0_to_SlamPosePostprocessTask_2");
    EXPECT_EQ(posePostprocess->inputs.at(3),
              "SlamVisualFeatureTrackingTask_0_to_SlamPosePostprocessTask_3");

    const auto *dfxSnapshot = findTask("SlamGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));

    const auto *monitor = findTask("SlamMonitorTask");
    ASSERT_NE(monitor, nullptr);
    EXPECT_EQ(monitor->inputs.at(11), "SlamUdpTask_1_to_SlamMonitorTask_11");
    EXPECT_EQ(monitor->inputs.at(12),
              "SlamPreviewTxTask_1_to_SlamMonitorTask_12");
    EXPECT_NE(std::find(monitor->trigger.queues.begin(),
                        monitor->trigger.queues.end(),
                        "SlamUdpTask_1_to_SlamMonitorTask_11"),
              monitor->trigger.queues.end());
    EXPECT_NE(std::find(monitor->trigger.queues.begin(),
                        monitor->trigger.queues.end(),
                        "SlamPreviewTxTask_1_to_SlamMonitorTask_12"),
              monitor->trigger.queues.end());
}

TEST(EventPipelineGraphDot, CompilesSystemRuntimeSubgraphFromMaintainedTopology)
{
    auto registry = MakeSystemShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_system_runtime_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 14u);
    ASSERT_EQ(config.queues.size(), 13u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string &name) -> const Epg::TaskConfig * {
        for (const auto &task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto *vehicleTelemetryRx = findTask("VehicleTelemetryRxTask");
    ASSERT_NE(vehicleTelemetryRx, nullptr);
    EXPECT_EQ(vehicleTelemetryRx->trigger.interval, std::chrono::milliseconds(2));
    EXPECT_EQ(vehicleTelemetryRx->outputs.at(0),
              "VehicleTelemetryRxTask_0_to_SetpointStreamTask_0");

    const auto *setpointStream = findTask("SetpointStreamTask");
    ASSERT_NE(setpointStream, nullptr);
    EXPECT_EQ(setpointStream->trigger.interval, std::chrono::milliseconds(5));
    EXPECT_EQ(setpointStream->inputs.at(0),
              "VehicleTelemetryRxTask_0_to_SetpointStreamTask_0");
    EXPECT_EQ(setpointStream->outputs.at(0),
              "SetpointStreamTask_0_to_UdpReceiveTask_0");

    const auto *udpReceive = findTask("UdpReceiveTask");
    ASSERT_NE(udpReceive, nullptr);
    EXPECT_EQ(udpReceive->trigger.interval, std::chrono::milliseconds(2));
    EXPECT_EQ(udpReceive->inputs.at(0),
              "SetpointStreamTask_0_to_UdpReceiveTask_0");
    EXPECT_EQ(udpReceive->outputs.at(0),
              "UdpReceiveTask_0_to_UdpHeartbeatTxTask_0");

    const auto *udpHeartbeat = findTask("UdpHeartbeatTxTask");
    ASSERT_NE(udpHeartbeat, nullptr);
    EXPECT_EQ(udpHeartbeat->trigger.interval, std::chrono::milliseconds(500));

    const auto *udpTimeout = findTask("UdpHeartbeatTimeoutTask");
    ASSERT_NE(udpTimeout, nullptr);
    EXPECT_EQ(udpTimeout->trigger.interval, std::chrono::milliseconds(100));

    const auto *udpState = findTask("UdpStateTxTask");
    ASSERT_NE(udpState, nullptr);
    EXPECT_EQ(udpState->trigger.interval, std::chrono::milliseconds(100));

    const auto *udpPointCloud = findTask("UdpPointCloudTxTask");
    ASSERT_NE(udpPointCloud, nullptr);
    EXPECT_EQ(udpPointCloud->trigger.interval, std::chrono::milliseconds(100));
    EXPECT_EQ(udpPointCloud->outputs.at(0),
              "UdpPointCloudTxTask_0_to_ManualControlTask_0");

    const auto *manualControl = findTask("ManualControlTask");
    ASSERT_NE(manualControl, nullptr);
    EXPECT_EQ(manualControl->trigger.interval, std::chrono::milliseconds(50));
    EXPECT_EQ(manualControl->inputs.at(0),
              "UdpPointCloudTxTask_0_to_ManualControlTask_0");

    const auto *forceRestart = findTask("ForceRestartTask");
    ASSERT_NE(forceRestart, nullptr);
    EXPECT_EQ(forceRestart->trigger.interval, std::chrono::milliseconds(50));

    const auto *supervisor = findTask("RuntimeSupervisorTask");
    ASSERT_NE(supervisor, nullptr);
    EXPECT_EQ(supervisor->trigger.interval, std::chrono::milliseconds(100));

    const auto *redeploy = findTask("EpgRedeployTask");
    ASSERT_NE(redeploy, nullptr);
    EXPECT_EQ(redeploy->type, "EpgRedeployTask");
    EXPECT_EQ(redeploy->trigger.interval, std::chrono::milliseconds(500));
    EXPECT_EQ(redeploy->inputs.at(0),
              "RuntimeSupervisorTask_0_to_EpgRedeployTask_0");

    const auto *dfxSnapshot = findTask("EpgDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));
    EXPECT_EQ(dfxSnapshot->inputs.at(0),
              "DiscoveryBeaconTask_0_to_EpgDfxSnapshotTask_0");
    EXPECT_EQ(dfxSnapshot->outputs.at(0),
              "EpgDfxSnapshotTask_0_to_EpgOptimizeTask_0");

    const auto *optimizer = findTask("EpgOptimizeTask");
    ASSERT_NE(optimizer, nullptr);
    EXPECT_EQ(optimizer->type, "EpgOptimizeTask");
    EXPECT_EQ(optimizer->trigger.interval, std::chrono::milliseconds(5000));
    EXPECT_EQ(optimizer->inputs.at(0),
              "EpgDfxSnapshotTask_0_to_EpgOptimizeTask_0");
}

TEST(EventPipelineGraphDot, CompilesCalibSubgraphFromMaintainedTopology)
{
    auto registry = MakeCalibShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_calib_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 11u);
    ASSERT_EQ(config.queues.size(), 13u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string &name) -> const Epg::TaskConfig * {
        for (const auto &task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto *camera = findTask("CalibCameraAcquireTask");
    ASSERT_NE(camera, nullptr);
    EXPECT_EQ(camera->inputs.at(0), "CalibResourceTask_0_to_CalibCameraAcquireTask_0");
    EXPECT_EQ(camera->inputs.at(1), "CalibClockTask_0_to_CalibCameraAcquireTask_1");

    const auto *pace = findTask("CalibPacingFilterTask");
    ASSERT_NE(pace, nullptr);
    EXPECT_EQ(pace->inputs.at(0), "CalibCameraAcquireTask_0_to_CalibPacingFilterTask_0");

    const auto *preview = findTask("CalibUdpPreviewTask");
    ASSERT_NE(preview, nullptr);
    EXPECT_EQ(preview->inputs.at(0), "CalibCameraAcquireTask_1_to_CalibUdpPreviewTask_0");

    const auto *completion = findTask("CalibCompletionTask");
    ASSERT_NE(completion, nullptr);
    EXPECT_EQ(completion->inputs.at(4), "CalibResourceTask_2_to_CalibCompletionTask_4");
    EXPECT_NE(std::find(completion->trigger.queues.begin(), completion->trigger.queues.end(),
                        "CalibResourceTask_2_to_CalibCompletionTask_4"),
              completion->trigger.queues.end());

    const auto *dfxSnapshot = findTask("CalibGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));
}
