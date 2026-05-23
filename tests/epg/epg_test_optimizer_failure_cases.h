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
