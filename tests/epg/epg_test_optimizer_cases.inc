TEST(EventPipelineGraphOptimizer, WritesOptimizedConfigFromFreshProfile)
{
    const std::string profilePath = "/tmp/smartdrone_epg_optimizer_test_profile.json";
    const std::string outputPath = "/tmp/smartdrone_epg_optimizer_test_optimized.json";
    const std::string reportPath =
        "/tmp/smartdrone_epg_optimizer_test_optimized_report.json";
    const std::uint64_t nowMs = 1000;
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
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
                "type": "TestPacket",
                "size": 4,
                "depth": 4,
                "pushed": 100,
                "popped": 70,
                "droppedNewest": 0,
                "overwrittenOldest": 0,
                "wakeups": 70,
                "maxDepthObserved": 5,
                "firstActivityMs": 1,
                "lastActivityMs": 1000,
                "windowMs": 999,
                "pushedPerSecond": 100,
                "poppedPerSecond": 70,
                "droppedPerSecond": 0
              }
            },
            "tasks": {
              "source": {
                "lastLoopUs": 2200,
                "maxLoopUs": 2600,
                "p50LoopUs": 2000,
                "p90LoopUs": 2400,
                "p99LoopUs": 2600,
              "totalLoopUs": 2600,
              "averageLoopUs": 2200,
              "resourceWaitCount": 2,
              "lastResourceWaitUs": 1300,
              "maxResourceWaitUs": 1500,
              "totalResourceWaitUs": 2500,
              "averageResourceWaitUs": 1250,
              "loopCount": 1,
                "errorCount": 0,
                "idleWakeups": 0,
                "firstLoopMs": 1,
                "lastLoopMs": 1000,
                "windowMs": 999,
                "utilizationPpm": 900000,
                "budgetOverrunCount": 2,
                "deadlineMissCount": 0,
                "schedulingErrorCount": 0,
                "lastSchedulingError": 0
              },
              "sink": {
                "lastLoopUs": 200,
                "maxLoopUs": 300,
                "p50LoopUs": 200,
                "p90LoopUs": 250,
                "p99LoopUs": 300,
              "totalLoopUs": 300,
              "averageLoopUs": 200,
              "resourceWaitCount": 0,
              "lastResourceWaitUs": 0,
              "maxResourceWaitUs": 0,
              "totalResourceWaitUs": 0,
              "averageResourceWaitUs": 0,
              "loopCount": 1,
                "errorCount": 0,
                "idleWakeups": 0,
                "firstLoopMs": 1,
                "lastLoopMs": 1000,
                "windowMs": 999,
                "utilizationPpm": 100000,
                "budgetOverrunCount": 0,
                "deadlineMissCount": 0,
                "schedulingErrorCount": 0,
                "lastSchedulingError": 0
              }
            }
          }
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath = reportPath;
    manifest.catalog[0] =
        {"TestSourceTask", "source", "cpu", 1500, 3000, true};
    manifest.catalog.push_back(
        {"TestSinkTask", "sink", "cpu", 1000, 3000, false});

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 10);
    EXPECT_TRUE(result.optimized) << result.message;
    EXPECT_TRUE(result.configChanged);
    EXPECT_EQ(result.targetGraph, "test_graph");
    EXPECT_EQ(result.topologyVersion, "test-topology");
    EXPECT_EQ(result.sourceProfile, "test_graph");
    EXPECT_EQ(result.sourceProfilePath, profilePath);
    EXPECT_EQ(result.sourceTimestampMs, 1000u);
    EXPECT_EQ(result.generatedAtMs, 1010u);
    EXPECT_EQ(result.solverVersion, Epg::NATIVE_EXACT_SOLVER_VERSION);
    EXPECT_EQ(result.optimizedConfigPath, outputPath);
    EXPECT_EQ(result.solverReportPath, reportPath);

    const std::string optimized = ReadFileText(outputPath);
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

    const auto config = Epg::ParseGraphConfigJson(optimized);
    ASSERT_EQ(config.queues.size(), 1u);
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.queues.front().depth, 5u);
    EXPECT_EQ(config.tasks.front().trigger.interval.count(), 3);
    EXPECT_EQ(config.tasks.front().scheduling.cpuAffinity, 2);
    EXPECT_TRUE(config.tasks.front().scheduling.realtime);
    EXPECT_EQ(config.tasks.front().scheduling.priority, 20);
    EXPECT_EQ(config.tasks.front().scheduling.budgetUs, 1500u);
    EXPECT_EQ(config.tasks.front().scheduling.deadlineUs, 3000u);

    const std::string report = ReadFileText(reportPath);
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
    EXPECT_NE(report.find("\"reason\": \"global_optimum_depth\""),
              std::string::npos);
    EXPECT_NE(report.find("\"reason\": \"global_optimum_interval\""),
              std::string::npos);
    EXPECT_NE(report.find("\"pressureAfter\": 0"), std::string::npos);
    EXPECT_NE(report.find("\"resourceWaitUs\": 2500"),
              std::string::npos);
    EXPECT_NE(report.find("\"totalResourceWaitUs\": 2500"),
              std::string::npos);
    EXPECT_NE(report.find("\"catalogRole\": \"source\""),
              std::string::npos);
    EXPECT_NE(report.find("\"replaceable\": true"), std::string::npos);
    EXPECT_NE(report.find("\"budgetOverruns\": 2"), std::string::npos);
    const auto optimizedGraph = Epg::ParseOptimizedGraphJson(optimized);
    const auto parsedReport = Epg::ParseSolverReportJson(report);
    const auto parsedProfile = Epg::ParseGraphProfileJson(
        ReadFileText(profilePath));
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

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
}

TEST(EventPipelineGraphOptimizer, KeepsNonReplaceableTaskInterval)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_non_replaceable_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_non_replaceable_optimized.json";
    const std::string reportPath =
        "/tmp/smartdrone_epg_optimizer_non_replaceable_report.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath, NonReplaceableTaskProfileJson());

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath = reportPath;

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    const auto config = Epg::ParseGraphConfigJson(ReadFileText(outputPath));
    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.tasks.front().trigger.interval.count(), 1);
    EXPECT_NE(ReadFileText(reportPath).find("not_replaceable+"),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
}

TEST(EventPipelineGraphOptimizer, RejectsProfileTasksOutsideManifest)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_outside_manifest_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_outside_manifest_optimized.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
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
          "diagnostics": )") +
            MinimalProfileDiagnosticsJson(1000) +
            R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("outside manifest"), std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
}

TEST(EventPipelineGraphOptimizer, RejectsProfileCatalogMismatch)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_catalog_mismatch_profile.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "gpu",
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
              }
            ]
          },
          "diagnostics": )") +
            MinimalProfileDiagnosticsJson(1000) +
            R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("catalog mismatch"), std::string::npos);

    (void)std::remove(profilePath.c_str());
}

TEST(EventPipelineGraphOptimizer, RejectsProfileMissingTaskDiagnostics)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_missing_task_diag_profile.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath, MissingTaskDiagnosticsProfileJson());

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("profile diagnostics missing task: source"),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
}

TEST(EventPipelineGraphOptimizer, CreatesArtifactDirectories)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_nested_profile.json";
    const std::string outputDir =
        "/tmp/smartdrone_epg_optimizer_nested_artifacts";
    const std::string outputPath = outputDir + "/optimized/config.json";
    const std::string reportPath = outputDir + "/reports/report.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
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
              }
            ]
          },
          "diagnostics": )") +
            MinimalProfileDiagnosticsJson(1000) +
            R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath = reportPath;

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    EXPECT_NE(ReadFileText(outputPath).find(Epg::OPTIMIZED_GRAPH_SCHEMA),
              std::string::npos);
    EXPECT_NE(ReadFileText(reportPath).find(Epg::SOLVER_REPORT_SCHEMA),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
    (void)std::remove((outputDir + "/optimized").c_str());
    (void)std::remove((outputDir + "/reports").c_str());
    (void)std::remove(outputDir.c_str());
}

TEST(EventPipelineGraphOptimizer, ReportsArtifactWriteFailure)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_write_failure_profile.json";
    const std::string blockedParent =
        "/tmp/smartdrone_epg_optimizer_blocked_parent";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        blockedParent, "{}");
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
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
              }
            ]
          },
          "diagnostics": )") +
            MinimalProfileDiagnosticsJson(1000) +
            R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = blockedParent + "/config.json";
    manifest.artifactPaths.solverReportPath = blockedParent + "/report.json";

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("not a directory"), std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(blockedParent.c_str());
}

TEST(EventPipelineGraphOptimizer, ReportsUnchangedConfigWithoutRedeploy)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_unchanged_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_unchanged_optimized.json";
    const std::uint64_t nowMs = 2000;
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 2000,
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
              }
            ]
          },
          "diagnostics": )") +
            MinimalProfileDiagnosticsJson(2000) +
            R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath =
        "/tmp/smartdrone_epg_optimizer_unchanged_optimized_report.json";

    const auto first =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 10);
    ASSERT_TRUE(first.optimized) << first.message;
    EXPECT_TRUE(first.configChanged);

    const auto second =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 20);
    EXPECT_TRUE(second.optimized) << second.message;
    EXPECT_FALSE(second.configChanged);
    EXPECT_EQ(second.targetGraph, "test_graph");
    EXPECT_EQ(second.sourceProfilePath, profilePath);
    EXPECT_EQ(second.generatedAtMs, nowMs + 20);
    EXPECT_EQ(second.optimizedConfigPath, outputPath);
    EXPECT_EQ(second.solverReportPath,
              manifest.artifactPaths.solverReportPath);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(manifest.artifactPaths.solverReportPath.c_str());
}
