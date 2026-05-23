TEST(GraphConfig, ParsesEscapesAndRejectsMissingFile)
{
    const auto config = Epg::ParseGraphConfigJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "tail_drop"}
      ],
      "tasks": [
        {
          "name": "source\nname",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "scheduling": {
            "resource": "cpu",
            "cpu_affinity": -1,
            "budget_us": 500,
            "deadline_us": 900,
            "backpressure_outputs": [0],
            "realtime": true,
            "priority": 42
          },
          "outputs": {"0": "packets"}
        }
      ]
    })");

    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.tasks.front().name, "source\nname");
    EXPECT_EQ(config.tasks.front().scheduling.resource, "cpu");
    EXPECT_EQ(config.tasks.front().scheduling.cpuAffinity, -1);
    EXPECT_EQ(config.tasks.front().scheduling.budgetUs, 500u);
    EXPECT_EQ(config.tasks.front().scheduling.deadlineUs, 900u);
    EXPECT_EQ(config.tasks.front().scheduling.backpressureOutputs,
              std::vector<Epg::PortId>{0});
    EXPECT_TRUE(config.tasks.front().scheduling.realtime);
    EXPECT_EQ(config.tasks.front().scheduling.priority, 42);
    EXPECT_THROW(
        Epg::ParseGraphConfigJsonFile("/tmp/smart_drone_missing_epg.json"),
        std::runtime_error);
}

TEST(GraphConfig, ParsesOptimizedRuntimeConfigJson)
{
    const auto config = Epg::ParseGraphConfigJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 6, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 3},
          "outputs": {"0": "packets"}
        },
        {
          "name": "sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })");

    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues.front().depth, 6u);
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks.front().trigger.interval,
              std::chrono::milliseconds(3));

    const auto metadata = Epg::ParseOptimizedGraphMetadataJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [],
      "tasks": []
    })");
    EXPECT_EQ(metadata.schema, Epg::OPTIMIZED_GRAPH_SCHEMA);
    EXPECT_EQ(metadata.targetGraph, "test_graph");
    EXPECT_EQ(metadata.topologyVersion, "test-topology");
    EXPECT_EQ(metadata.solverVersion, "unit-test");
    EXPECT_EQ(metadata.sourceProfile, "test_graph");
    EXPECT_EQ(metadata.sourceTimestampMs, 123u);
    EXPECT_EQ(metadata.generatedAtMs, 456u);

    const auto optimized = Epg::ParseOptimizedGraphJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 6, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 3},
          "outputs": {"0": "packets"}
        }
      ]
    })");
    EXPECT_EQ(optimized.metadata.targetGraph, "test_graph");
    ASSERT_EQ(optimized.config.queues.size(), 1u);
    ASSERT_EQ(optimized.config.tasks.size(), 1u);
    const auto report = Epg::ParseSolverReportJson(R"({
      "schema": "smartdrone.epg.solver_report.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "solverVersion": "unit-test",
      "objective": {
        "name": "unit",
        "score": {
          "queuePressure": 0,
          "periodicOverloadUs": 0,
          "resourceWaitUs": 0,
          "schedulingErrors": 0,
          "budgetOverruns": 0,
          "deadlineMisses": 0,
          "utilizationOverPpm": 0,
          "totalPenalty": 0
        }
      },
      "constraints": {
        "maxQueueDepth": 16,
        "maxPeriodicIntervalMs": 1000,
        "targetUtilizationPpm": 800000
      },
      "decisions": [
        {
          "kind": "queue",
          "name": "packets",
          "depthBefore": 6,
          "depthAfter": 6,
          "pressureBefore": 0,
          "pressureAfter": 0,
          "maxDepthObserved": 0,
          "droppedNewest": 0,
          "overwrittenOldest": 0,
          "pushedPerSecond": 0,
          "poppedPerSecond": 0,
          "droppedPerSecond": 0,
          "reason": "keep"
        },
        {
          "kind": "task",
          "name": "source",
          "intervalBeforeMs": 3,
          "intervalAfterMs": 3,
          "maxLoopUs": 0,
          "averageLoopUs": 0,
          "p90LoopUs": 0,
          "p99LoopUs": 0,
          "effectiveLoopUs": 0,
          "resourceWaitCount": 0,
          "maxResourceWaitUs": 0,
          "averageResourceWaitUs": 0,
          "totalResourceWaitUs": 0,
          "utilizationPpm": 0,
          "targetUtilizationPpm": 800000,
          "budgetUs": 0,
          "deadlineUs": 0,
          "catalogRole": "source",
          "replaceable": false,
          "budgetOverrunCount": 0,
          "deadlineMissCount": 0,
          "schedulingErrorCount": 0,
          "reason": "keep"
        }
      ]
    })");
    EXPECT_EQ(report.metadata.schema, Epg::SOLVER_REPORT_SCHEMA);
    EXPECT_EQ(report.metadata.targetGraph, "test_graph");
    EXPECT_EQ(report.metadata.topologyVersion, "test-topology");
    EXPECT_EQ(report.metadata.sourceProfile, "test_graph");
    EXPECT_EQ(report.metadata.sourceTimestampMs, 123u);
    EXPECT_EQ(report.metadata.generatedAtMs, 456u);
    EXPECT_EQ(report.metadata.solverVersion, "unit-test");
    EXPECT_EQ(report.objectiveName, "unit");
    EXPECT_EQ(report.score.resourceWaitUs, 0u);
    EXPECT_EQ(report.score.totalPenalty, 0u);
    EXPECT_EQ(report.constraints.maxQueueDepth, 16u);
    ASSERT_EQ(report.decisions.size(), 2u);
    EXPECT_EQ(report.decisions.front().name, "packets");
    EXPECT_EQ(report.decisions.front().depthAfter, 6u);
    EXPECT_EQ(report.decisions.back().intervalAfterMs, 3u);
    EXPECT_THROW(
        Epg::ParseSolverReportJson(R"({
          "schema": "smartdrone.epg.solver_report.v1",
          "targetGraph": "test_graph",
          "topologyVersion": "test-topology",
          "sourceProfile": "test_graph",
          "sourceTimestampMs": 123,
          "generatedAtMs": 456,
          "solverVersion": "unit-test",
          "objective": {"name": "unit", "score": {}},
          "constraints": {},
          "decisions": []
        })"),
        std::runtime_error);
    EXPECT_THROW(
        Epg::ParseOptimizedGraphJson(R"({
          "schema": "smartdrone.epg.optimized_config.v1",
          "targetGraph": "test_graph",
          "topologyVersion": "test-topology",
          "solverVersion": "unit-test",
          "queues": [],
          "tasks": []
        })"),
        std::runtime_error);
}

TEST(GraphConfig, ParsesNestedGraphConfigJsonField)
{
    const auto config = Epg::ParseGraphConfigJsonField(R"({
      "schema": "smartdrone.epg.profile.v1",
      "topology": {
        "queues": [
          {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
        ],
        "tasks": [
          {
            "name": "source",
            "type": "TestSourceTask",
            "trigger": {"mode": "periodic", "interval_ms": 1},
            "outputs": {"0": "packets"}
          }
        ]
      },
      "diagnostics": {}
    })",
                                                       "topology");

    ASSERT_EQ(config.queues.size(), 1u);
    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.queues.front().name, "packets");
    EXPECT_EQ(config.tasks.front().name, "source");
    EXPECT_THROW(
        Epg::ParseGraphConfigJsonField("{\"diagnostics\": {}}", "topology"),
        std::runtime_error);
}

TEST(GraphConfig, RejectsInvalidJsonAndUnsupportedEnumValues)
{
    EXPECT_THROW(Epg::ParseGraphConfigJson("{"), std::runtime_error);
    EXPECT_THROW(Epg::ParseGraphConfigJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "bad_policy"}
      ],
      "tasks": []
    })"),
                 std::runtime_error);
    EXPECT_THROW(Epg::ParseGraphConfigJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "heartbeat",
          "type": "TestHeartbeatTask",
          "trigger": {"mode": "bad_trigger"}
        }
      ]
    })"),
                 std::runtime_error);
}

TEST(EventPipelineGraphReflection, RegistersMessagesAndTaskPortsFromCatalog)
{
    Registry registry;
    Epg::TypeCatalog::Global().RegisterReflectedMessageTypes(registry);
    Epg::TypeCatalog::Global().RegisterReflectedTaskTypes(
        registry,
        {"ReflectedSourceTask", "ReflectedSinkTask"},
        [](const std::string &type) {
            if (type == "ReflectedSourceTask") {
                return Registry::TaskFactory([]() {
                    return std::unique_ptr<ITask>(new ReflectedSourceTask());
                });
            }
            if (type == "ReflectedSinkTask") {
                return Registry::TaskFactory([]() {
                    return std::unique_ptr<ITask>(new ReflectedSinkTask());
                });
            }
            return Registry::TaskFactory{};
        });

    EventPipelineGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [
        {"name": "reflected_packets", "type": "ReflectedPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "ReflectedSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "reflected_packets"}
        },
        {
          "name": "sink",
          "type": "ReflectedSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["reflected_packets"]},
          "inputs": {"0": "reflected_packets"}
        }
      ]
    })");
    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    graph.Stop();

    const auto taskDiag = graph.TaskDiagnostics();
    EXPECT_GT(taskDiag.at("source").loopCount, 0u);
    EXPECT_GT(taskDiag.at("sink").loopCount, 0u);
}

TEST(EventPipelineGraphValidation, RejectsInvalidTopologyConfigurations)
{
    const std::vector<std::string> invalidJsons = {
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"},{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[{"name":"packets","type":"MissingPacket","depth":4,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":0,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1}},{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1}}]})",
        R"({"queues":[],"tasks":[{"name":"missing","type":"MissingTask","trigger":{"mode":"periodic","interval_ms":1}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"realtime":true,"priority":0}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"realtime":true,"priority":100}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"resource":""}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"cpu_affinity":-2}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"budget_us":2000,"deadline_us":1000}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":0}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":[]},"inputs": {"0":"packets"}}]})",
        R"({"queues":[],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["missing"]},"inputs": {"0":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs": {"999":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"999":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs": {"0":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"OtherPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs": {"0":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"OtherPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source_a","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"packets"}},{"name":"source_b","type":"TestSecondSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"},{"name":"other","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["other"]},"inputs": {"0":"packets"}}]})"};

    for (const auto &json : invalidJsons) {
        ExpectConfigureThrows(json);
    }
}
