TEST(EventPipelineGraphOptimizer, WritesOptimizedConfigFromFreshProfile)
{
    const auto paths = FreshOptimizerPaths();
    const std::uint64_t nowMs = 1000;
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        paths.profilePath,
        FRESH_OPTIMIZER_PROFILE_JSON);

    const auto manifest = MakeFreshOptimizerManifest(paths);

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 10);
    ExpectFreshOptimizerResult(result, paths);

    const std::string optimized = ReadFileText(paths.outputPath);
    ExpectFreshOptimizedText(optimized);

    const auto config = Epg::ParseGraphConfigJson(optimized);
    ExpectFreshOptimizedConfig(config);

    const std::string report = ReadFileText(paths.reportPath);
    ExpectFreshSolverReportText(report);
    const auto optimizedGraph = Epg::ParseOptimizedGraphJson(optimized);
    const auto parsedReport = Epg::ParseSolverReportJson(report);
    const auto parsedProfile = Epg::ParseGraphProfileJson(
        ReadFileText(paths.profilePath));
    ExpectFreshParsedSolverReport(
        manifest, parsedProfile, optimizedGraph, parsedReport, config);

    RemoveOptimizerArtifacts(paths);
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
    EXPECT_NE(ReadFileText(reportPath).find("global_optimum_cpu_binding"),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
}

TEST(EventPipelineGraphOptimizer,
     AllowsAccuracyPreservingTaskResourceIsolationOnly)
{
    const auto paths = AccuracyOptimizerPaths();
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        paths.profilePath, AccuracyPreservingProfileJson());

    const auto manifest = MakeAccuracyOptimizerManifest(paths);

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    const auto optimizedGraph = Epg::ParseOptimizedGraphJson(
        ReadFileText(paths.outputPath));
    ExpectAccuracyOptimizedGraph(optimizedGraph);

    const auto parsedReport = Epg::ParseSolverReportJson(
        ReadFileText(paths.reportPath));
    ExpectAccuracySolverReport(parsedReport);
    const auto parsedProfile = Epg::ParseGraphProfileJson(
        ReadFileText(paths.profilePath));
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, parsedProfile, optimizedGraph, parsedReport));

    RemoveOptimizerArtifacts(paths);
}

TEST(EventPipelineGraphOptimizer, RejectsProfileTasksOutsideManifest)
{
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_outside_manifest_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_outside_manifest_optimized.json";
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        profilePath,
        OutsideManifestProfileJson());

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
