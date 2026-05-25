TEST(EventPipelineGraphOptimizer, WritesCompactTopologySchedule)
{
    const auto paths = TopologyOptimizerPaths();
    SmartDrone::Core::Application::WriteEpgDfxSnapshotFile(
        paths.profilePath,
        TOPOLOGY_OPTIMIZER_PROFILE_JSON);

    const auto manifest = MakeTopologyOptimizerManifest(paths);

    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    const auto optimizedGraph = Epg::ParseOptimizedGraphJson(
        ReadFileText(paths.outputPath));
    ExpectCompactTopologySchedule(optimizedGraph);

    const auto report = Epg::ParseSolverReportJson(ReadFileText(paths.reportPath));
    const auto profile = Epg::ParseGraphProfileJson(ReadFileText(paths.profilePath));
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, profile, optimizedGraph, report));

    RemoveOptimizerArtifacts(paths);
}
