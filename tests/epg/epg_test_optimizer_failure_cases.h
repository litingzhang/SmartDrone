TEST(EventPipelineGraphOptimizer, RejectsProfileCatalogMismatch)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_catalog_mismatch_profile.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        OptimizerSingleTaskProfileJson("gpu", 1000));

    const auto manifest = MakeOptimizerProfileManifest(profilePath);

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
    const std::string outputDir =
        "/tmp/smartdrone_epg_optimizer_nested_artifacts";
    const auto paths = NestedOptimizerPaths(outputDir);
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        paths.profilePath,
        OptimizerSingleTaskProfileJson("cpu", 1000));

    const auto manifest = MakeOptimizerArtifactManifest(paths);

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    ExpectOptimizerArtifactSchemas(paths);

    RemoveNestedOptimizerArtifacts(paths, outputDir);
}

TEST(EventPipelineGraphOptimizer, ReportsArtifactWriteFailure)
{
    const std::string blockedParent =
        "/tmp/smartdrone_epg_optimizer_blocked_parent";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        blockedParent, "{}");
    const auto paths = BlockedOptimizerPaths(blockedParent);
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        paths.profilePath,
        OptimizerSingleTaskProfileJson("cpu", 1000));

    const auto manifest = MakeOptimizerArtifactManifest(paths);

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("not a directory"), std::string::npos);

    (void)std::remove(paths.profilePath.c_str());
    (void)std::remove(blockedParent.c_str());
}

TEST(EventPipelineGraphOptimizer, ReportsUnchangedConfigWithoutRedeploy)
{
    const auto paths = UnchangedOptimizerPaths();
    const std::uint64_t nowMs = 2000;
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        paths.profilePath,
        OptimizerSingleTaskProfileJson("cpu", nowMs));

    const auto manifest = MakeOptimizerArtifactManifest(paths);

    const auto first =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 10);
    ASSERT_TRUE(first.optimized) << first.message;
    EXPECT_TRUE(first.configChanged);

    const auto second =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 20);
    ExpectUnchangedOptimizerResult(second, paths, nowMs + 20);

    RemoveOptimizerArtifacts(paths);
}
