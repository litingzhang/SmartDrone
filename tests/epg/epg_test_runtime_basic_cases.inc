TEST(EventPipelineGraphQueue, DropNewestKeepsOldItemsAndCountsDrops)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 2, OverflowPolicy::DropNewest);

    EXPECT_EQ(queue.Name(), "packets");
    EXPECT_EQ(queue.TypeName(), "TestPacket");
    EXPECT_EQ(queue.Depth(), 2u);
    EXPECT_TRUE(queue.Empty());

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));
    EXPECT_FALSE(queue.Push(std::make_shared<TestPacket>(TestPacket{3})));

    EXPECT_EQ(queue.Size(), 2u);
    EXPECT_EQ(queue.TryPop()->sequence, 1);
    EXPECT_EQ(queue.TryPop()->sequence, 2);
    EXPECT_FALSE(queue.TryPop());

    const auto diag = queue.Diagnostics();
    EXPECT_EQ(diag.pushed, 2u);
    EXPECT_EQ(diag.popped, 2u);
    EXPECT_EQ(diag.droppedNewest, 1u);
    EXPECT_EQ(diag.overwrittenOldest, 0u);
    EXPECT_EQ(diag.maxDepthObserved, 2u);
}

TEST(EventPipelineGraphQueue, OverwriteOldestKeepsNewestItemsAndCountsOverwrites)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 2, OverflowPolicy::OverwriteOldest);

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{3})));

    EXPECT_EQ(queue.TryPop()->sequence, 2);
    EXPECT_EQ(queue.TryPop()->sequence, 3);
    EXPECT_FALSE(queue.TryPop());

    const auto diag = queue.Diagnostics();
    EXPECT_EQ(diag.pushed, 3u);
    EXPECT_EQ(diag.popped, 2u);
    EXPECT_EQ(diag.droppedNewest, 0u);
    EXPECT_EQ(diag.overwrittenOldest, 1u);
}

TEST(EventPipelineGraphQueue, TryPopLatestDrainsQueueAndReturnsNewestItem)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 4, OverflowPolicy::DropNewest);

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{3})));

    const auto latest = queue.TryPopLatest();
    ASSERT_TRUE(latest);
    EXPECT_EQ(latest->sequence, 3);
    EXPECT_TRUE(queue.Empty());
    EXPECT_EQ(queue.Diagnostics().popped, 3u);
}

TEST(EventPipelineGraphQueue, NotifierRunsForAcceptedPushesOnly)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 1, OverflowPolicy::DropNewest);
    int notifications = 0;
    queue.SetNotifier([&notifications]() { ++notifications; });

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_FALSE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));

    EXPECT_EQ(notifications, 1);
    EXPECT_EQ(queue.Diagnostics().wakeups, 1u);
}

TEST(EventPipelineGraphIngress, ExternalInterruptEventWakesConsumerTask)
{
    Registry registry;
    registry.RegisterMessageType<TestPacket>("TestPacket");
    int packets = 0;
    registry.RegisterTaskFactory(
        "TestCountingSinkTask",
        {PortSpec{0, "TestPacket"}},
        {},
        [&packets]() {
            return std::unique_ptr<ITask>(new TestCountingSinkTask(packets));
        });

    EventPipelineGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [
        {"name": "irq_events", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "interrupt_consumer",
          "type": "TestCountingSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["irq_events"]},
          "inputs": {"0": "irq_events"}
        }
      ]
    })");

    auto ingress = graph.CreateExternalIngress<TestPacket>("irq_events");
    ASSERT_TRUE(ingress.Valid());
    EXPECT_EQ(ingress.QueueName(), "irq_events");

    graph.Start();
    EXPECT_TRUE(ingress.Emplace(TestPacket{1}));
    EXPECT_TRUE(ingress.Emplace(TestPacket{2}));

    for (int i = 0; i < 50 && packets < 2; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    graph.Stop();

    EXPECT_EQ(packets, 2);
    const auto queueDiagnostics = graph.QueueDiagnostics().at("irq_events");
    EXPECT_EQ(queueDiagnostics.pushed, 2u);
    EXPECT_EQ(queueDiagnostics.popped, 2u);
    EXPECT_GE(queueDiagnostics.wakeups, 1u);
    EXPECT_GT(graph.TaskDiagnostics().at("interrupt_consumer").loopCount, 0u);
}

TEST(EventPipelineGraphIngress, RejectsTypeMismatchDuplicateIngressAndTaskProducerConflict)
{
    auto registry = MakeRegistry();

    {
        EventPipelineGraph graph(registry);
        graph.ConfigureJson(R"({
          "queues": [
            {"name": "irq_events", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
          ],
          "tasks": [
            {
              "name": "sink",
              "type": "TestSinkTask",
              "trigger": {"mode": "any_queue_ready", "queues": ["irq_events"]},
              "inputs": {"0": "irq_events"}
            }
          ]
        })");

        EXPECT_THROW(graph.CreateExternalIngress<OtherPacket>("irq_events"), std::runtime_error);
        auto ingress = graph.CreateExternalIngress<TestPacket>("irq_events");
        EXPECT_TRUE(ingress.Valid());
        EXPECT_THROW(graph.CreateExternalIngress<TestPacket>("irq_events"), std::runtime_error);
    }

    {
        EventPipelineGraph graph(registry);
        graph.ConfigureJson(MinimalValidJson());
        EXPECT_THROW(graph.CreateExternalIngress<TestPacket>("packets"), std::runtime_error);
    }
}

TEST(EventPipelineGraph, RunsPipelineConfiguredFromJsonFile)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(Epg::ParseGraphConfigJsonFile(
        std::string(TEST_EPG_DIR) + "/basic_pipeline.json"));

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    graph.Stop();

    const auto queueDiagnostics = graph.QueueDiagnostics();
    const auto packets = queueDiagnostics.at("packets");
    EXPECT_GT(packets.pushed, 0u);
    EXPECT_GT(packets.popped, 0u);
    EXPECT_GT(packets.wakeups, 0u);
    EXPECT_LE(packets.maxDepthObserved, 8u);

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("source").loopCount, 0u);
    EXPECT_GT(taskDiagnostics.at("sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("source").errorCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("sink").errorCount, 0u);
}

TEST(EventPipelineGraph, TaskCanPublishToMultipleQueuesForDifferentConsumers)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(Epg::ParseGraphConfigJsonFile(
        std::string(TEST_EPG_DIR) + "/fanout_pipeline.json"));

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    graph.Stop();

    const auto queueDiagnostics = graph.QueueDiagnostics();
    const auto left = queueDiagnostics.at("left_packets");
    const auto right = queueDiagnostics.at("right_packets");

    EXPECT_GT(left.pushed, 0u);
    EXPECT_GT(right.pushed, 0u);
    EXPECT_GT(left.popped, 0u);
    EXPECT_GT(right.popped, 0u);
    EXPECT_GT(left.wakeups, 0u);
    EXPECT_GT(right.wakeups, 0u);

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("fanout_source").loopCount, 0u);
    EXPECT_GT(taskDiagnostics.at("left_sink").loopCount, 0u);
    EXPECT_GT(taskDiagnostics.at("right_sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("fanout_source").errorCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("left_sink").errorCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("right_sink").errorCount, 0u);
}

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

    ASSERT_EQ(config.tasks.size(), 15u);
    ASSERT_EQ(config.queues.size(), 23u);

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

    const auto *tracking = findTask("SlamTrackingTask");
    ASSERT_NE(tracking, nullptr);
    EXPECT_EQ(tracking->inputs.at(0),
              "SlamAcquireTask_0_to_SlamTrackingTask_0");
    EXPECT_EQ(tracking->trigger.queues,
              (std::vector<std::string>{"SlamAcquireTask_0_to_SlamTrackingTask_0"}));

    const auto *dfxSnapshot = findTask("SlamGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));

    const auto *monitor = findTask("SlamMonitorTask");
    ASSERT_NE(monitor, nullptr);
    EXPECT_EQ(monitor->inputs.at(8), "SlamUdpTask_1_to_SlamMonitorTask_8");
    EXPECT_NE(std::find(monitor->trigger.queues.begin(),
                        monitor->trigger.queues.end(),
                        "SlamUdpTask_1_to_SlamMonitorTask_8"),
              monitor->trigger.queues.end());
}

TEST(EventPipelineGraphDot, CompilesSystemRuntimeSubgraphFromMaintainedTopology)
{
    auto registry = MakeSystemShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_system_runtime_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 10u);
    ASSERT_EQ(config.queues.size(), 9u);

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
              "SetpointStreamTask_0_to_UdpCommandTask_0");

    const auto *udpCommand = findTask("UdpCommandTask");
    ASSERT_NE(udpCommand, nullptr);
    EXPECT_EQ(udpCommand->inputs.at(0),
              "SetpointStreamTask_0_to_UdpCommandTask_0");
    EXPECT_EQ(udpCommand->outputs.at(0),
              "UdpCommandTask_0_to_ManualControlTask_0");

    const auto *manualControl = findTask("ManualControlTask");
    ASSERT_NE(manualControl, nullptr);
    EXPECT_EQ(manualControl->trigger.interval, std::chrono::milliseconds(50));
    EXPECT_EQ(manualControl->inputs.at(0),
              "UdpCommandTask_0_to_ManualControlTask_0");

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

TEST(EventPipelineGraphLifecycle, ResetForStartClearsStopRequest)
{
    std::atomic<bool> stop{true};
    SmartDrone::Core::Application::EpgGraphLifecycle lifecycle({
        stop,
        []() { return true; },
        []() {},
        []() {},
    });

    lifecycle.ResetForStart();

    EXPECT_FALSE(stop.load());
    EXPECT_FALSE(lifecycle.Done());
    EXPECT_FALSE(lifecycle.StopRequested());
}

TEST(EventPipelineGraphRedeploy, TracksSystemAndSessionRequestsIndependently)
{
    SmartDrone::Core::Application::EpgRedeployCoordinator redeploy;

    redeploy.RequestSystemRedeploy({
        "cluster_system_runtime_graph",
        "optimized config changed",
        "config/epg/epg_topology.dot:v3",
        "cluster_system_runtime_graph",
        "/tmp/smartdrone_epg_system_profile.json",
        1000,
        1010,
        "native-exact-v1",
        "output/epg/optimized_system_runtime_graph.json",
        "output/epg/optimized_system_runtime_graph_report.json",
    });
    EXPECT_TRUE(redeploy.SystemRedeployRequested());
    EXPECT_FALSE(redeploy.SessionRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest systemRequest;
    EXPECT_TRUE(redeploy.TakeSystemRedeployRequest(systemRequest));
    EXPECT_EQ(systemRequest.graphName, "cluster_system_runtime_graph");
    EXPECT_EQ(systemRequest.reason, "optimized config changed");
    EXPECT_EQ(systemRequest.topologyVersion, "config/epg/epg_topology.dot:v3");
    EXPECT_EQ(systemRequest.sourceProfile, "cluster_system_runtime_graph");
    EXPECT_EQ(systemRequest.sourceProfilePath,
              "/tmp/smartdrone_epg_system_profile.json");
    EXPECT_EQ(systemRequest.sourceTimestampMs, 1000u);
    EXPECT_EQ(systemRequest.generatedAtMs, 1010u);
    EXPECT_EQ(systemRequest.solverVersion, "native-exact-v1");
    EXPECT_EQ(systemRequest.optimizedConfigPath,
              "output/epg/optimized_system_runtime_graph.json");
    EXPECT_EQ(systemRequest.solverReportPath,
              "output/epg/optimized_system_runtime_graph_report.json");
    const auto description =
        SmartDrone::Core::Application::DescribeEpgRedeployRequest(
            systemRequest);
    EXPECT_NE(description.find("graph=cluster_system_runtime_graph"),
              std::string::npos);
    EXPECT_NE(description.find("source_ts_ms=1000"), std::string::npos);
    EXPECT_NE(
        description.find(
            "optimized=output/epg/optimized_system_runtime_graph.json"),
        std::string::npos);
    EXPECT_FALSE(redeploy.TakeSystemRedeployRequest(systemRequest));

    redeploy.RequestSessionRedeploy({
        "cluster_slam_session_graph",
        "optimized config changed",
        "config/epg/epg_topology.dot:v3",
        "cluster_slam_session_graph",
        "/tmp/smartdrone_epg_slam_profile.json",
        2000,
        2010,
        "native-exact-v1",
        "output/epg/optimized_slam_session_graph.json",
        "output/epg/optimized_slam_session_graph_report.json",
    });
    EXPECT_TRUE(redeploy.SessionRedeployRequested());
    EXPECT_FALSE(redeploy.SystemRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest sessionRequest;
    EXPECT_TRUE(redeploy.TakeSessionRedeployRequest(sessionRequest));
    EXPECT_EQ(sessionRequest.graphName, "cluster_slam_session_graph");
    EXPECT_EQ(sessionRequest.reason, "optimized config changed");
    EXPECT_EQ(sessionRequest.sourceTimestampMs, 2000u);
    EXPECT_EQ(sessionRequest.generatedAtMs, 2010u);
    EXPECT_EQ(sessionRequest.optimizedConfigPath,
              "output/epg/optimized_slam_session_graph.json");
    EXPECT_EQ(sessionRequest.solverReportPath,
              "output/epg/optimized_slam_session_graph_report.json");
    EXPECT_FALSE(redeploy.TakeSessionRedeployRequest(sessionRequest));
}
