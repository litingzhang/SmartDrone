const char *const OPTIMIZER_SINGLE_TASK_PROFILE_PREFIX = R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": )";

const char *const OPTIMIZER_SINGLE_TASK_PROFILE_RESOURCE_PREFIX = R"(,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": ")";

const char *const OPTIMIZER_SINGLE_TASK_PROFILE_RESOURCE_SUFFIX = R"(",
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
          "diagnostics": )";

const char *const OPTIMIZER_SINGLE_TASK_PROFILE_SUFFIX = R"(
        })";

std::string OptimizerSingleTaskProfileJson(const std::string &resource,
                                           std::uint64_t timestampMs)
{
    return std::string(OPTIMIZER_SINGLE_TASK_PROFILE_PREFIX) +
           std::to_string(timestampMs) +
           OPTIMIZER_SINGLE_TASK_PROFILE_RESOURCE_PREFIX + resource +
           OPTIMIZER_SINGLE_TASK_PROFILE_RESOURCE_SUFFIX +
           MinimalProfileDiagnosticsJson(timestampMs) +
           OPTIMIZER_SINGLE_TASK_PROFILE_SUFFIX;
}

SmartDrone::Core::Application::EpgTaskManifest MakeOptimizerProfileManifest(
    const std::string &profilePath)
{
    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    return manifest;
}

SmartDrone::Core::Application::EpgTaskManifest MakeOptimizerArtifactManifest(
    const OptimizerArtifactPaths &paths)
{
    auto manifest = MakeOptimizerProfileManifest(paths.profilePath);
    manifest.artifactPaths.optimizedConfigPath = paths.outputPath;
    manifest.artifactPaths.solverReportPath = paths.reportPath;
    return manifest;
}

OptimizerArtifactPaths NestedOptimizerPaths(const std::string &outputDir)
{
    return {
        "/tmp/smartdrone_epg_optimizer_nested_profile.json",
        outputDir + "/optimized/config.json",
        outputDir + "/reports/report.json",
    };
}

OptimizerArtifactPaths BlockedOptimizerPaths(const std::string &blockedParent)
{
    return {
        "/tmp/smartdrone_epg_optimizer_write_failure_profile.json",
        blockedParent + "/config.json",
        blockedParent + "/report.json",
    };
}

OptimizerArtifactPaths UnchangedOptimizerPaths()
{
    return {
        "/tmp/smartdrone_epg_optimizer_unchanged_profile.json",
        "/tmp/smartdrone_epg_optimizer_unchanged_optimized.json",
        "/tmp/smartdrone_epg_optimizer_unchanged_optimized_report.json",
    };
}

void ExpectOptimizerArtifactSchemas(const OptimizerArtifactPaths &paths)
{
    EXPECT_NE(ReadFileText(paths.outputPath).find(Epg::OPTIMIZED_GRAPH_SCHEMA),
              std::string::npos);
    EXPECT_NE(ReadFileText(paths.reportPath).find(Epg::SOLVER_REPORT_SCHEMA),
              std::string::npos);
}

void RemoveNestedOptimizerArtifacts(const OptimizerArtifactPaths &paths,
                                    const std::string &outputDir)
{
    RemoveOptimizerArtifacts(paths);
    (void)std::remove((outputDir + "/optimized").c_str());
    (void)std::remove((outputDir + "/reports").c_str());
    (void)std::remove(outputDir.c_str());
}

void ExpectUnchangedOptimizerResult(
    const SmartDrone::Core::Application::EpgRuntimeOptimizerResult &result,
    const OptimizerArtifactPaths &paths,
    std::uint64_t generatedAtMs)
{
    EXPECT_TRUE(result.optimized) << result.message;
    EXPECT_FALSE(result.configChanged);
    EXPECT_EQ(result.targetGraph, "test_graph");
    EXPECT_EQ(result.sourceProfilePath, paths.profilePath);
    EXPECT_EQ(result.generatedAtMs, generatedAtMs);
    EXPECT_EQ(result.optimizedConfigPath, paths.outputPath);
    EXPECT_EQ(result.solverReportPath, paths.reportPath);
}
