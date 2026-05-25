const char *const OPTIMIZED_RUNTIME_CONFIG_JSON = R"({
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
    })";

const char *const OPTIMIZED_METADATA_JSON = R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [],
      "tasks": []
    })";

const char *const OPTIMIZED_GRAPH_JSON = R"({
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
    })";

const char *const UNIT_SOLVER_REPORT_JSON = R"({
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
          "topologyLevel": 0,
          "phaseOffsetMs": 0,
          "durationMs": 0,
          "catalogRole": "source",
          "replaceable": false,
          "budgetOverrunCount": 0,
          "deadlineMissCount": 0,
          "schedulingErrorCount": 0,
          "reason": "keep"
        }
      ]
    })";

const char *const INVALID_SOLVER_REPORT_JSON = R"({
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
        })";

const char *const INVALID_OPTIMIZED_GRAPH_JSON = R"({
          "schema": "smartdrone.epg.optimized_config.v1",
          "targetGraph": "test_graph",
          "topologyVersion": "test-topology",
          "solverVersion": "unit-test",
          "queues": [],
          "tasks": []
        })";

void ExpectOptimizedRuntimeConfig(const Epg::GraphConfig &config)
{
    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues.front().depth, 6u);
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks.front().trigger.interval,
              std::chrono::milliseconds(3));
}

void ExpectOptimizedMetadata(const Epg::OptimizedGraphMetadata &metadata)
{
    EXPECT_EQ(metadata.schema, Epg::OPTIMIZED_GRAPH_SCHEMA);
    EXPECT_EQ(metadata.targetGraph, "test_graph");
    EXPECT_EQ(metadata.topologyVersion, "test-topology");
    EXPECT_EQ(metadata.solverVersion, "unit-test");
    EXPECT_EQ(metadata.sourceProfile, "test_graph");
    EXPECT_EQ(metadata.sourceTimestampMs, 123u);
    EXPECT_EQ(metadata.generatedAtMs, 456u);
}

void ExpectOptimizedGraph(const Epg::OptimizedGraph &optimized)
{
    EXPECT_EQ(optimized.metadata.targetGraph, "test_graph");
    ASSERT_EQ(optimized.config.queues.size(), 1u);
    ASSERT_EQ(optimized.config.tasks.size(), 1u);
}

void ExpectUnitSolverReport(const Epg::SolverReport &report)
{
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
    EXPECT_EQ(report.decisions.back().cpuAffinityBefore, -1);
    EXPECT_EQ(report.decisions.back().cpuAffinityAfter, -1);
}
