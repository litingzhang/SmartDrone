TEST(EventPipelineGraphManifest, BuildsCentralizedArtifactPaths)
{
    using SmartDrone::Core::Application::BuildEpgTaskArtifactPaths;

    const auto paths = BuildEpgTaskArtifactPaths({
        "smartdrone_epg_unit",
        "optimized_unit_graph",
    });

    EXPECT_EQ(paths.dfxSnapshotPath, "/tmp/smartdrone_epg_unit.json");
    EXPECT_EQ(paths.profilePath, "/tmp/smartdrone_epg_unit_profile.json");
    EXPECT_EQ(paths.optimizedConfigPath,
              "output/epg/optimized_unit_graph.json");
    EXPECT_EQ(paths.solverReportPath,
              "output/epg/optimized_unit_graph_report.json");

    EXPECT_EQ(BuildEpgTaskTopologyVersion({"config/epg/unit.dot", "v7"}),
              "config/epg/unit.dot:v7");
}

TEST(EventPipelineGraphManifest, RuntimeManifestsUseCentralizedArtifacts)
{
    const auto &system = EpgManifestForDomain(EpgDomain::SystemRuntime);
    const auto systemTaskTypes = EpgTaskCatalogTypes(system);
    EXPECT_EQ(systemTaskTypes.size(), 14u);
    EXPECT_EQ(systemTaskTypes.front(), "VehicleTelemetryRxTask");
    EXPECT_NE(std::find(systemTaskTypes.begin(), systemTaskTypes.end(),
                        "EpgRedeployTask"),
              systemTaskTypes.end());
    EXPECT_EQ(system.topologyPath, "config/epg/epg_topology.dot");
    EXPECT_EQ(
        system.topologyVersion,
        BuildEpgTaskTopologyVersion({system.topologyPath, "v4"}));
    EXPECT_EQ(system.artifactPaths.dfxSnapshotPath,
              "/tmp/smartdrone_epg_system.json");
    EXPECT_EQ(system.artifactPaths.profilePath,
              "/tmp/smartdrone_epg_system_profile.json");
    EXPECT_EQ(system.artifactPaths.optimizedConfigPath,
              "output/epg/optimized_system_runtime_graph.json");
    EXPECT_EQ(system.artifactPaths.solverReportPath,
              "output/epg/optimized_system_runtime_graph_report.json");

    const auto &slam = EpgManifestForDomain(EpgDomain::SlamSession);
    const auto slamTaskTypes = EpgTaskCatalogTypes(slam);
    EXPECT_EQ(slamTaskTypes.size(), 20u);
    EXPECT_EQ(slamTaskTypes.back(), "EpgDfxSnapshotTask");
    EXPECT_EQ(slam.topologyPath, system.topologyPath);
    EXPECT_EQ(slam.topologyVersion, system.topologyVersion);
    EXPECT_EQ(slam.artifactPaths.dfxSnapshotPath,
              "/tmp/smartdrone_epg_slam.json");
    EXPECT_EQ(slam.artifactPaths.profilePath,
              "/tmp/smartdrone_epg_slam_profile.json");
    EXPECT_EQ(slam.artifactPaths.optimizedConfigPath,
              "output/epg/optimized_slam_session_graph.json");
    EXPECT_EQ(slam.artifactPaths.solverReportPath,
              "output/epg/optimized_slam_session_graph_report.json");
    ASSERT_EQ(slam.runtimeTuning.size(), 3u);
    EXPECT_EQ(slam.runtimeTuning[0].taskName, "SlamResourceTask");
    EXPECT_TRUE(slam.runtimeTuning[0].interval);
    EXPECT_EQ(slam.runtimeTuning[1].taskName, "SlamClockTask");
    EXPECT_TRUE(slam.runtimeTuning[1].interval);
    EXPECT_EQ(slam.runtimeTuning[2].taskName, "SlamImuPollTask");
    EXPECT_TRUE(slam.runtimeTuning[2].realtime);
    EXPECT_TRUE(slam.runtimeTuning[2].priority);

    const auto &calib = EpgManifestForDomain(EpgDomain::CalibSession);
    const auto calibTaskTypes = EpgTaskCatalogTypes(calib);
    EXPECT_EQ(calibTaskTypes.size(), 11u);
    EXPECT_EQ(calibTaskTypes.back(), "EpgDfxSnapshotTask");
    EXPECT_EQ(calib.topologyPath, system.topologyPath);
    EXPECT_EQ(calib.topologyVersion, system.topologyVersion);
    EXPECT_EQ(calib.artifactPaths.dfxSnapshotPath,
              "/tmp/smartdrone_epg_calib.json");
    EXPECT_EQ(calib.artifactPaths.profilePath,
              "/tmp/smartdrone_epg_calib_profile.json");
    EXPECT_EQ(calib.artifactPaths.optimizedConfigPath,
              "output/epg/optimized_calib_session_graph.json");
    EXPECT_EQ(calib.artifactPaths.solverReportPath,
              "output/epg/optimized_calib_session_graph_report.json");
}

TEST(EventPipelineGraphManifest, ValidatesRuntimeTuningManifest)
{
    auto manifest = MakeValidTestManifest();
    manifest.runtimeTuning.push_back({"source", true, false, false});
    Epg::GraphConfig config;
    config.tasks.push_back({"source", "TestSourceTask"});

    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"source", true, false, false}}));
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"source", false, true, false}}),
        std::runtime_error);
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"missing", true, false, false}}),
        std::runtime_error);

    auto duplicate = manifest;
    duplicate.runtimeTuning.push_back({"source", false, true, false});
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskManifest(duplicate),
        std::runtime_error);

    auto incomplete = manifest;
    incomplete.runtimeTuning.push_back({"", false, false, false});
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskManifest(incomplete),
        std::runtime_error);

    auto missingGraphTask = manifest;
    missingGraphTask.runtimeTuning.push_back({"missing", true, false, false});
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskGraphManifest(
            missingGraphTask, config),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsDuplicateCatalogTaskTypes)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog.push_back(
        {"TestSourceTask", "duplicate", "cpu", 1000, 2000, false});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsAliasTargetOutsideCatalog)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"LegacySourceTask", "MissingTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsDuplicateAliases)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"LegacySourceTask", "TestSourceTask"});
    manifest.aliases.push_back({"LegacySourceTask", "TestSourceTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsAliasShadowingCatalogType)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"TestSourceTask", "TestSourceTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteAliasMetadata)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"", "TestSourceTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsEmptyCatalog)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog.clear();

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteCatalogMetadata)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog[0].role.clear();

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsInvalidCatalogTiming)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog[0].budgetUs = 2000;
    manifest.catalog[0].deadlineUs = 1000;

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteManifestMetadata)
{
    {
        auto manifest = MakeValidTestManifest();
        manifest.subgraphName.clear();

        EXPECT_THROW(
            SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
    {
        auto manifest = MakeValidTestManifest();
        manifest.topologyVersion.clear();

        EXPECT_THROW(
            SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
    {
        auto manifest = MakeValidTestManifest();
        manifest.artifactPaths.profilePath.clear();

        EXPECT_THROW(
            SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
}
