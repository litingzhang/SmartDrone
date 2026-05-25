struct OptimizerArtifactPaths {
    std::string profilePath;
    std::string outputPath;
    std::string reportPath;
};

const char *const FRESH_OPTIMIZER_PROFILE_JSON = R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "resourceAlternates": ["cpu_isolated"],
              "budgetUs": 1500,
              "deadlineUs": 3000,
              "replaceable": true
            },
            {
              "taskType": "TestSinkTask",
              "role": "sink",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 3000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
            ],
            "tasks": [
              {
                "name": "source",
                "type": "TestSourceTask",
                "trigger": {"mode": "periodic", "interval_ms": 1},
                "outputs": {"0": "packets"}
              },
              {
                "name": "sink",
                "type": "TestSinkTask",
                "trigger": {
                  "mode": "any_queue_ready",
                  "queues": ["packets"]
                },
                "inputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": {
            "graph": "test_graph",
            "timestampMs": 1000,
            "queues": {
              "packets": {
                "droppedNewest": 0,
                "overwrittenOldest": 0,
                "maxDepthObserved": 5,
                "pushedPerSecond": 100,
                "poppedPerSecond": 70,
                "droppedPerSecond": 0
              }
            },
            "tasks": {
              "source": {
                "maxLoopUs": 2600,
                "p90LoopUs": 2400,
                "p99LoopUs": 2600,
                "averageLoopUs": 2200,
                "resourceWaitCount": 2,
                "maxResourceWaitUs": 1500,
                "totalResourceWaitUs": 2500,
                "averageResourceWaitUs": 1250,
                "utilizationPpm": 900000,
                "budgetOverrunCount": 2,
                "deadlineMissCount": 0,
                "schedulingErrorCount": 0
              },
              "sink": {
                "maxLoopUs": 300,
                "p90LoopUs": 250,
                "p99LoopUs": 300,
                "averageLoopUs": 200,
                "resourceWaitCount": 0,
                "maxResourceWaitUs": 0,
                "totalResourceWaitUs": 0,
                "averageResourceWaitUs": 0,
                "utilizationPpm": 100000,
                "budgetOverrunCount": 0,
                "deadlineMissCount": 0,
                "schedulingErrorCount": 0
              }
            }
          }
        })";

const char *const ACCURACY_PROFILE_PREFIX = R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {"taskType": "TestSourceTask", "role": "source", "resource": "cpu", "resourceAlternates": ["cpu_isolated"], "budgetUs": 1500, "deadlineUs": 3000, "replaceable": true, "preserveAccuracy": true},
            {"taskType": "TestSinkTask", "role": "sink", "resource": "cpu", "budgetUs": 1000, "deadlineUs": 3000, "replaceable": false}
          ],
          "topology": {
            "queues": [{"name": "packets", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"}],
            "tasks": [
              {"name": "source", "type": "TestSourceTask", "trigger": {"mode": "periodic", "interval_ms": 1}, "scheduling": {"resource": "cpu", "cpu_affinity": -1, "budget_us": 1500, "deadline_us": 3000, "backpressure_outputs": [], "realtime": false, "priority": 0}, "outputs": {"0": "packets"}},
              {"name": "sink", "type": "TestSinkTask", "trigger": {"mode": "any_queue_ready", "queues": ["packets"]}, "inputs": {"0": "packets"}}
            ]
          },
          "diagnostics": {
            "graph": "test_graph",
            "timestampMs": 1000,
            "queues": {"packets": {"maxDepthObserved": 5, "droppedNewest": 0, "overwrittenOldest": 0, "pushedPerSecond": 100, "poppedPerSecond": 70, "droppedPerSecond": 0}},
            "tasks": {
              "source": )";

const char *const ACCURACY_PROFILE_SUFFIX = R"(,
              "sink": {"maxLoopUs": 300, "p90LoopUs": 250, "p99LoopUs": 300, "averageLoopUs": 200, "resourceWaitCount": 0, "maxResourceWaitUs": 0, "averageResourceWaitUs": 0, "totalResourceWaitUs": 0, "utilizationPpm": 0, "budgetOverrunCount": 0, "deadlineMissCount": 0, "schedulingErrorCount": 0}
            }
          }
        })";

const char *const OUTSIDE_MANIFEST_PROFILE_PREFIX = R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 2000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
            ],
            "tasks": [
              {
                "name": "source",
                "type": "TestSourceTask",
                "trigger": {"mode": "periodic", "interval_ms": 1},
                "outputs": {"0": "packets"}
              },
              {
                "name": "sink",
                "type": "TestSinkTask",
                "trigger": {
                  "mode": "any_queue_ready",
                  "queues": ["packets"]
                },
                "inputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": )";

const char *const OUTSIDE_MANIFEST_PROFILE_SUFFIX = R"(
        })";

OptimizerArtifactPaths FreshOptimizerPaths()
{
    return {
        "/tmp/smartdrone_epg_optimizer_test_profile.json",
        "/tmp/smartdrone_epg_optimizer_test_optimized.json",
        "/tmp/smartdrone_epg_optimizer_test_optimized_report.json",
    };
}

OptimizerArtifactPaths AccuracyOptimizerPaths()
{
    return {
        "/tmp/smartdrone_epg_optimizer_accuracy_profile.json",
        "/tmp/smartdrone_epg_optimizer_accuracy_optimized.json",
        "/tmp/smartdrone_epg_optimizer_accuracy_report.json",
    };
}

void RemoveOptimizerArtifacts(const OptimizerArtifactPaths &paths)
{
    (void)std::remove(paths.profilePath.c_str());
    (void)std::remove(paths.outputPath.c_str());
    (void)std::remove(paths.reportPath.c_str());
}

SmartDrone::Core::Application::EpgTaskManifest MakeFreshOptimizerManifest(
    const OptimizerArtifactPaths &paths)
{
    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = paths.profilePath;
    manifest.artifactPaths.optimizedConfigPath = paths.outputPath;
    manifest.artifactPaths.solverReportPath = paths.reportPath;
    manifest.catalog[0] =
        {"TestSourceTask", "source", "cpu", 1500, 3000, true,
         {"cpu_isolated"}};
    manifest.catalog.push_back(
        {"TestSinkTask", "sink", "cpu", 1000, 3000, false});
    return manifest;
}

SmartDrone::Core::Application::EpgTaskManifest MakeAccuracyOptimizerManifest(
    const OptimizerArtifactPaths &paths)
{
    auto manifest = MakeFreshOptimizerManifest(paths);
    manifest.catalog[0] =
        {"TestSourceTask", "source", "cpu", 1500, 3000, true,
         {"cpu_isolated"}, true};
    return manifest;
}

std::string AccuracyPreservingProfileJson()
{
    return std::string(ACCURACY_PROFILE_PREFIX) +
           TaskDiagnosticsJson(2600, 2200, 1250, 900000) +
           ACCURACY_PROFILE_SUFFIX;
}

std::string OutsideManifestProfileJson()
{
    return std::string(OUTSIDE_MANIFEST_PROFILE_PREFIX) +
           MinimalProfileDiagnosticsJson(1000) +
           OUTSIDE_MANIFEST_PROFILE_SUFFIX;
}

void ExpectFreshOptimizerResult(
    const SmartDrone::Core::Application::EpgRuntimeOptimizerResult &result,
    const OptimizerArtifactPaths &paths)
{
    EXPECT_TRUE(result.optimized) << result.message;
    EXPECT_TRUE(result.configChanged);
    EXPECT_EQ(result.targetGraph, "test_graph");
    EXPECT_EQ(result.topologyVersion, "test-topology");
    EXPECT_EQ(result.sourceProfile, "test_graph");
    EXPECT_EQ(result.sourceProfilePath, paths.profilePath);
    EXPECT_EQ(result.sourceTimestampMs, 1000u);
    EXPECT_EQ(result.generatedAtMs, 1010u);
    EXPECT_EQ(result.solverVersion, Epg::NATIVE_EXACT_SOLVER_VERSION);
    EXPECT_EQ(result.optimizedConfigPath, paths.outputPath);
    EXPECT_EQ(result.solverReportPath, paths.reportPath);
}

void ExpectFreshOptimizedText(const std::string &optimized)
{
    EXPECT_NE(optimized.find(std::string("\"schema\": \"") +
                             Epg::OPTIMIZED_GRAPH_SCHEMA + "\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"targetGraph\": \"test_graph\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"sourceProfile\": \"test_graph\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"topologyVersion\": \"test-topology\""),
              std::string::npos);
    EXPECT_NE(optimized.find(std::string("\"solverVersion\": \"") +
                             Epg::NATIVE_EXACT_SOLVER_VERSION + "\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"generatedAtMs\": 1010"),
              std::string::npos);
}

void ExpectFreshOptimizedConfig(const Epg::GraphConfig &config)
{
    ASSERT_EQ(config.queues.size(), 1u);
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.queues.front().depth, 4u);
    EXPECT_EQ(config.tasks.front().trigger.interval.count(), 3);
    EXPECT_EQ(config.tasks.front().scheduling.resource, "cpu_isolated");
    EXPECT_EQ(config.tasks.front().scheduling.backpressureOutputs,
              std::vector<Epg::PortId>{0});
    EXPECT_EQ(config.tasks.front().scheduling.cpuAffinity, -1);
    EXPECT_TRUE(config.tasks.front().scheduling.realtime);
    EXPECT_EQ(config.tasks.front().scheduling.priority, 20);
    EXPECT_EQ(config.tasks.front().scheduling.budgetUs, 1500u);
    EXPECT_EQ(config.tasks.front().scheduling.deadlineUs, 3000u);
}

void ExpectFreshSolverReportText(const std::string &report)
{
    EXPECT_NE(report.find(std::string("\"schema\": \"") +
                          Epg::SOLVER_REPORT_SCHEMA + "\""),
              std::string::npos);
    EXPECT_NE(report.find("\"targetGraph\": \"test_graph\""),
              std::string::npos);
    EXPECT_NE(report.find("\"topologyVersion\": \"test-topology\""),
              std::string::npos);
    EXPECT_NE(report.find(std::string("\"solverVersion\": \"") +
                          Epg::NATIVE_EXACT_SOLVER_VERSION + "\""),
              std::string::npos);
    EXPECT_NE(report.find("\"generatedAtMs\": 1010"), std::string::npos);
    EXPECT_NE(report.find("\"reason\": \"keep\""), std::string::npos);
    EXPECT_NE(report.find(
                  "\"reason\": \"global_optimum_interval+global_optimum_backpressure+global_optimum_resource+global_optimum_cpu_binding\""),
              std::string::npos);
    EXPECT_NE(report.find("\"topologyPenalty\": 31"), std::string::npos);
    EXPECT_NE(report.find("\"backpressureBefore\": []"), std::string::npos);
    EXPECT_NE(report.find("\"backpressureAfter\": [0]"), std::string::npos);
    EXPECT_NE(report.find("\"pressureAfter\": 1"), std::string::npos);
    EXPECT_NE(report.find("\"resourceWaitUs\": 0"), std::string::npos);
    EXPECT_NE(report.find("\"totalResourceWaitUs\": 2500"),
              std::string::npos);
    EXPECT_NE(report.find("\"predictedResourceWaitUs\": 0"),
              std::string::npos);
    EXPECT_NE(report.find("\"resourceAfter\": \"cpu_isolated\""),
              std::string::npos);
    EXPECT_NE(report.find("\"cpuBindingAffinity\": 0"), std::string::npos);
    EXPECT_NE(report.find("\"catalogRole\": \"source\""), std::string::npos);
    EXPECT_NE(report.find("\"replaceable\": true"), std::string::npos);
    EXPECT_NE(report.find("\"budgetOverruns\": 2"), std::string::npos);
}

void ExpectFreshParsedSolverReport(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &parsedProfile,
    const Epg::OptimizedGraph &optimizedGraph,
    const Epg::SolverReport &parsedReport,
    const Epg::GraphConfig &config)
{
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, parsedProfile, optimizedGraph, parsedReport));
    EXPECT_EQ(parsedReport.objectiveName,
              SmartDrone::Core::Application::EPG_EXACT_SOLVER_OBJECTIVE);
    EXPECT_EQ(parsedReport.constraints.maxQueueDepth, 16u);
    ASSERT_EQ(parsedReport.decisions.size(), 3u);
    EXPECT_EQ(parsedReport.decisions[0].depthAfter,
              config.queues.front().depth);
    EXPECT_EQ(parsedReport.decisions[1].intervalAfterMs,
              static_cast<std::uint64_t>(
                  config.tasks.front().trigger.interval.count()));
    EXPECT_EQ(parsedReport.decisions[1].backpressureAfter,
              std::vector<Epg::PortId>{0});
    EXPECT_EQ(parsedReport.decisions[1].resourceAfter, "cpu_isolated");
}

void ExpectAccuracyOptimizedGraph(const Epg::OptimizedGraph &optimizedGraph)
{
    ASSERT_EQ(optimizedGraph.config.queues.size(), 1u);
    ASSERT_EQ(optimizedGraph.config.tasks.size(), 2u);
    const auto &task = optimizedGraph.config.tasks.front();
    EXPECT_EQ(optimizedGraph.config.queues.front().depth, 1u);
    EXPECT_EQ(task.trigger.interval.count(), 1);
    EXPECT_EQ(task.scheduling.resource, "cpu");
    EXPECT_EQ(task.scheduling.backpressureOutputs,
              std::vector<Epg::PortId>{});
    EXPECT_EQ(task.scheduling.cpuAffinity, -1);
    EXPECT_FALSE(task.scheduling.realtime);
    EXPECT_EQ(task.scheduling.priority, 0);
}

void ExpectAccuracySolverReport(const Epg::SolverReport &parsedReport)
{
    ASSERT_EQ(parsedReport.decisions.size(), 3u);
    EXPECT_EQ(parsedReport.decisions[0].depthAfter, 1u);
    EXPECT_EQ(parsedReport.decisions[0].reason, "keep");
    EXPECT_EQ(parsedReport.decisions[1].intervalAfterMs, 1u);
    EXPECT_EQ(parsedReport.decisions[1].resourceAfter, "cpu");
    EXPECT_EQ(parsedReport.decisions[1].backpressureAfter,
              std::vector<Epg::PortId>{});
    EXPECT_EQ(parsedReport.decisions[1].predictedResourceWaitUs, 2500u);
    EXPECT_EQ(parsedReport.decisions[1].reason,
              "global_optimum_cpu_binding");
}
