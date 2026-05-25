const char *const TOPOLOGY_OPTIMIZER_PROFILE_JSON = R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {"taskType": "TestSourceTask", "role": "source", "resource": "cpu", "budgetUs": 1000, "deadlineUs": 5000, "replaceable": true},
            {"taskType": "TestForwardTask", "role": "forward", "resource": "cpu", "budgetUs": 1000, "deadlineUs": 5000, "replaceable": false},
            {"taskType": "TestSinkTask", "role": "sink", "resource": "cpu", "budgetUs": 5000, "deadlineUs": 5000, "replaceable": false}
          ],
          "topology": {
            "queues": [
              {"name": "source_to_left", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"},
              {"name": "source_to_right", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"},
              {"name": "left_to_sink", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"},
              {"name": "right_to_sink", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"}
            ],
            "tasks": [
              {"name": "sink", "type": "TestSinkTask", "trigger": {"mode": "any_queue_ready", "queues": ["left_to_sink", "right_to_sink"]}, "inputs": {"0": "left_to_sink", "1": "right_to_sink"}},
              {"name": "left", "type": "TestForwardTask", "trigger": {"mode": "any_queue_ready", "queues": ["source_to_left"]}, "inputs": {"0": "source_to_left"}, "outputs": {"0": "left_to_sink"}},
              {"name": "right", "type": "TestForwardTask", "trigger": {"mode": "any_queue_ready", "queues": ["source_to_right"]}, "inputs": {"0": "source_to_right"}, "outputs": {"0": "right_to_sink"}},
              {"name": "source", "type": "TestSourceTask", "trigger": {"mode": "periodic", "interval_ms": 1}, "outputs": {"0": "source_to_left", "1": "source_to_right"}}
            ]
          },
          "diagnostics": {
            "queues": {
              "source_to_left": {"maxDepthObserved": 0, "droppedNewest": 0, "overwrittenOldest": 0, "pushedPerSecond": 0, "poppedPerSecond": 0, "droppedPerSecond": 0},
              "source_to_right": {"maxDepthObserved": 0, "droppedNewest": 0, "overwrittenOldest": 0, "pushedPerSecond": 0, "poppedPerSecond": 0, "droppedPerSecond": 0},
              "left_to_sink": {"maxDepthObserved": 0, "droppedNewest": 0, "overwrittenOldest": 0, "pushedPerSecond": 0, "poppedPerSecond": 0, "droppedPerSecond": 0},
              "right_to_sink": {"maxDepthObserved": 0, "droppedNewest": 0, "overwrittenOldest": 0, "pushedPerSecond": 0, "poppedPerSecond": 0, "droppedPerSecond": 0}
            },
            "tasks": {
              "source": {"maxLoopUs": 1200, "p90LoopUs": 1200, "p99LoopUs": 1200, "averageLoopUs": 1200, "resourceWaitCount": 0, "maxResourceWaitUs": 0, "averageResourceWaitUs": 0, "totalResourceWaitUs": 0, "utilizationPpm": 1200000, "budgetOverrunCount": 1, "deadlineMissCount": 0, "schedulingErrorCount": 0},
              "left": {"maxLoopUs": 2000, "p90LoopUs": 2000, "p99LoopUs": 2000, "averageLoopUs": 2000, "resourceWaitCount": 0, "maxResourceWaitUs": 0, "averageResourceWaitUs": 0, "totalResourceWaitUs": 0, "utilizationPpm": 0, "budgetOverrunCount": 1, "deadlineMissCount": 0, "schedulingErrorCount": 0},
              "right": {"maxLoopUs": 3000, "p90LoopUs": 3000, "p99LoopUs": 3000, "averageLoopUs": 3000, "resourceWaitCount": 0, "maxResourceWaitUs": 0, "averageResourceWaitUs": 0, "totalResourceWaitUs": 0, "utilizationPpm": 0, "budgetOverrunCount": 1, "deadlineMissCount": 0, "schedulingErrorCount": 0},
              "sink": {"maxLoopUs": 400, "p90LoopUs": 400, "p99LoopUs": 400, "averageLoopUs": 400, "resourceWaitCount": 0, "maxResourceWaitUs": 0, "averageResourceWaitUs": 0, "totalResourceWaitUs": 0, "utilizationPpm": 0, "budgetOverrunCount": 0, "deadlineMissCount": 0, "schedulingErrorCount": 0}
            }
          }
        })";

OptimizerArtifactPaths TopologyOptimizerPaths()
{
    return {
        "/tmp/smartdrone_epg_optimizer_topology_profile.json",
        "/tmp/smartdrone_epg_optimizer_topology_optimized.json",
        "/tmp/smartdrone_epg_optimizer_topology_report.json",
    };
}

SmartDrone::Core::Application::EpgTaskManifest MakeTopologyOptimizerManifest(
    const OptimizerArtifactPaths &paths)
{
    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = paths.profilePath;
    manifest.artifactPaths.optimizedConfigPath = paths.outputPath;
    manifest.artifactPaths.solverReportPath = paths.reportPath;
    manifest.catalog[0] =
        {"TestSourceTask", "source", "cpu", 1000, 5000, true};
    manifest.catalog.push_back(
        {"TestForwardTask", "forward", "cpu", 1000, 5000, false});
    manifest.catalog.push_back(
        {"TestSinkTask", "sink", "cpu", 5000, 5000, false});
    return manifest;
}

void ExpectCompactTopologySchedule(const Epg::OptimizedGraph &optimizedGraph)
{
    ASSERT_EQ(optimizedGraph.config.tasks.size(), 4u);
    EXPECT_EQ(optimizedGraph.config.tasks[0].name, "source");
    EXPECT_EQ(optimizedGraph.config.tasks[1].name, "left");
    EXPECT_EQ(optimizedGraph.config.tasks[2].name, "right");
    EXPECT_EQ(optimizedGraph.config.tasks[3].name, "sink");
    EXPECT_EQ(optimizedGraph.config.tasks[1].scheduling.topologyLevel, 1u);
    EXPECT_EQ(optimizedGraph.config.tasks[2].scheduling.topologyLevel, 1u);
    EXPECT_EQ(optimizedGraph.config.tasks[1].scheduling.phaseOffsetMs, 2u);
    EXPECT_EQ(optimizedGraph.config.tasks[2].scheduling.phaseOffsetMs, 2u);
    EXPECT_EQ(optimizedGraph.config.tasks[3].scheduling.phaseOffsetMs, 5u);
}
