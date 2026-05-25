TEST(EventPipelineGraphRedeploy, TakesSystemRedeploySignal)
{
    SmartDrone::Core::Application::EpgRedeployCoordinator redeploy;
    redeploy.RequestSystemRedeploy({
        "cluster_system_runtime_graph",
        "optimized config changed",
    });

    EXPECT_TRUE(redeploy.SystemRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest request;
    EXPECT_TRUE(redeploy.TakeSystemRedeployRequest(request));
    EXPECT_EQ(request.graphName, "cluster_system_runtime_graph");
    EXPECT_EQ(request.reason, "optimized config changed");
    EXPECT_FALSE(redeploy.TakeSystemRedeployRequest(request));
}

TEST(EventPipelineGraphDotTopology, RunsBasicTopologyFromDot)
{
    RunTopologyFromDotFile(
        "basic_pipeline.dot",
        {"source_0_to_sink_0"},
        {"source", "sink"});
}

TEST(EventPipelineGraphDotTopology, RunsFanoutTopologyFromDot)
{
    RunTopologyFromDotFile(
        "fanout_pipeline.dot",
        {"fanout_source_0_to_left_sink_0", "fanout_source_1_to_right_sink_0"},
        {"fanout_source", "left_sink", "right_sink"});
}

TEST(EventPipelineGraphDotTopology, RunsLinearChainTopologyFromDot)
{
    RunTopologyFromDotFile(
        "chain_pipeline.dot",
        {"source_0_to_forward_0", "forward_0_to_sink_0"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphDotTopology, RunsParallelIndependentTopologyFromDot)
{
    RunTopologyFromDotFile(
        "parallel_pipeline.dot",
        {"left_source_0_to_left_sink_0", "right_source_0_to_right_sink_0"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(EventPipelineGraphDotTopology, RunsFanInJoinTopologyFromDot)
{
    RunTopologyFromDotFile(
        "fanin_pipeline.dot",
        {"left_source_0_to_join_sink_0", "right_source_0_to_join_sink_1"},
        {"left_source", "right_source", "join_sink"});
}

TEST(EventPipelineGraphDotTopology, RunsDiamondTopologyFromDot)
{
    RunTopologyFromDotFile(
        "diamond_pipeline.dot",
        {"fanout_source_0_to_left_forward_0",
         "fanout_source_1_to_join_sink_1",
         "left_forward_0_to_join_sink_0"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(EventPipelineGraphMermaid, RejectsInvalidMermaidTopology)
{
    EXPECT_THROW(Epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source[type=TestSourceTask; trigger=periodic; interval_ms=1]
        source.out -->|type=TestPacket; depth=8; overflow=drop_newest| missing.in
    )"),
                 std::runtime_error);

    EXPECT_THROW(Epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source[type=TestSourceTask; trigger=periodic; interval_ms=1]
        sink[type=TestSinkTask; trigger=any_queue_ready]
        source.out -->|type=TestPacket; overflow=drop_newest| sink.in
    )"),
                 std::runtime_error);

    auto registry = MakeRegistry();
    EXPECT_THROW(Epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
        sink["type=TestSinkTask; trigger=any_queue_ready; trigger_queues=OtherPacket"]
        source -->|"type=TestPacket; depth=1; overflow=drop_newest"| sink
    )",
                                              registry),
                 std::runtime_error);
}

TEST(EventPipelineGraph, RejectsMultipleConsumersForSpscQueue)
{
    const char *json = R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "packets"}
        },
        {
          "name": "sink_a",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        },
        {
          "name": "sink_b",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })";

    ExpectConfigureThrows(json);
}

TEST(EventPipelineGraph, AllQueueReadyWaitsForAllInputs)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "left_packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"},
        {"name": "right_packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "left_source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "left_packets"}
        },
        {
          "name": "right_source",
          "type": "TestSecondSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 2},
          "outputs": {"0": "right_packets"}
        },
        {
          "name": "all_sink",
          "type": "TestAllInputsSinkTask",
          "trigger": {"mode": "all_queue_ready", "queues": ["left_packets", "right_packets"]},
          "inputs": {"0": "left_packets", "1": "right_packets"}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    graph.Stop();

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("all_sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("all_sink").errorCount, 0u);
}

TEST(EventPipelineGraph, PeriodicOrAnyQueueReadyRunsWithQueueTrigger)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "packets"}
        },
        {
          "name": "hybrid_sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "periodic_or_any_queue_ready", "interval_ms": 50, "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    graph.Stop();

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("hybrid_sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("hybrid_sink").errorCount, 0u);
}

TEST(EventPipelineGraph, PeriodicOrAnyQueueReadyRunsWithoutQueuedItems)
{
    auto registry = MakeRegistry();
    int ticks = 0;
    registry.RegisterTaskFactory(
        "CountingHybridTask", {PortSpec{0, "TestPacket"}}, {},
        [&ticks]() { return std::unique_ptr<ITask>(new TestCountingTask(ticks)); });
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "hybrid",
          "type": "CountingHybridTask",
          "trigger": {"mode": "periodic_or_any_queue_ready", "interval_ms": 5, "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    graph.Stop();

    EXPECT_GT(ticks, 0);
    EXPECT_EQ(graph.TaskDiagnostics().at("hybrid").errorCount, 0u);
}

TEST(EventPipelineGraph, PeriodicTaskCanRunWithoutQueues)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "heartbeat",
          "type": "TestHeartbeatTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("heartbeat").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("heartbeat").errorCount, 0u);
}

TEST(EventPipelineGraph, SupportsCapturedTaskFactoryForNativeAdapters)
{
    int ticks = 0;
    Registry registry;
    registry.RegisterTaskFactory(
        "TestCountingTask",
        {},
        {},
        [&ticks]() {
            return std::unique_ptr<ITask>(new TestCountingTask(ticks));
        });

    EventPipelineGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "counter",
          "type": "TestCountingTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    EXPECT_GT(ticks, 0);
    EXPECT_GT(graph.TaskDiagnostics().at("counter").loopCount, 0u);
}

TEST(EventPipelineGraph, TaskExceptionsAreCountedAndRunnerKeepsAlive)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "thrower",
          "type": "TestThrowingTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const auto diag = graph.TaskDiagnostics().at("thrower");
    EXPECT_GT(diag.loopCount, 0u);
    EXPECT_EQ(diag.errorCount, diag.loopCount);
}

TEST(EventPipelineGraph, TaskBudgetAndDeadlineViolationsAreCounted)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "slow",
          "type": "TestSlowTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "scheduling": {"budget_us": 1, "deadline_us": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    graph.Stop();

    const auto diag = graph.TaskDiagnostics().at("slow");
    EXPECT_GT(diag.loopCount, 0u);
    EXPECT_GT(diag.budgetOverrunCount, 0u);
    EXPECT_GT(diag.deadlineMissCount, 0u);
    EXPECT_GT(diag.p50LoopUs, 0u);
}

TEST(EventPipelineGraph, ContextSlotErrorsAreCountedAsTaskErrors)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "bad_context",
          "type": "TestBadContextTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const auto diag = graph.TaskDiagnostics().at("bad_context");
    EXPECT_GT(diag.errorCount, 0u);
}

TEST(EventPipelineGraph, LifecycleStateAndAccessorsBehaveAsExpected)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    EXPECT_FALSE(graph.Running());
    EXPECT_THROW(graph.Start(), std::runtime_error);

    graph.ConfigureJson(MinimalValidJson());
    ASSERT_NE(graph.Queue("packets"), nullptr);
    EXPECT_EQ(graph.Queue("missing"), nullptr);
    EXPECT_FALSE(graph.Running());

    graph.Start();
    EXPECT_TRUE(graph.Running());
    EXPECT_THROW(graph.ConfigureJson(MinimalValidJson()), std::runtime_error);
    graph.RequestStop();
    EXPECT_TRUE(graph.Running());
    for (int i = 0; i < 50 && graph.Running(); ++i) {
        if (graph.JoinStopped()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(graph.JoinStopped());
    EXPECT_FALSE(graph.Running());

    graph.Start();
    EXPECT_TRUE(graph.Running());
    graph.Stop();
    EXPECT_FALSE(graph.Running());

    graph.Stop();
    EXPECT_FALSE(graph.Running());
}
