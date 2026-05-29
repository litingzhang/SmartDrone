void ExpectSystemRuntimeManifestArtifacts(
    const SmartDrone::Core::Application::EpgTaskManifest &system)
{
    const auto systemTaskTypes = EpgTaskCatalogTypes(system);
    EXPECT_EQ(systemTaskTypes.size(), 14u);
    EXPECT_EQ(systemTaskTypes.front(), "VehicleTelemetryRxTask");
    EXPECT_NE(std::find(systemTaskTypes.begin(), systemTaskTypes.end(),
                        "EpgRedeployTask"),
              systemTaskTypes.end());
    EXPECT_EQ(system.topologyPath, "config/epg/epg_topology.dot");
    EXPECT_EQ(
        system.topologyVersion,
        BuildEpgTaskTopologyVersion({system.topologyPath, "v9"}));
    EXPECT_EQ(system.artifactPaths.dfxSnapshotPath,
              "/tmp/smartdrone_epg_system.json");
    EXPECT_EQ(system.artifactPaths.profilePath,
              "/tmp/smartdrone_epg_system_profile.json");
    EXPECT_EQ(system.artifactPaths.optimizedConfigPath,
              "output/epg/optimized_system_runtime_graph.json");
    EXPECT_EQ(system.artifactPaths.solverReportPath,
              "output/epg/optimized_system_runtime_graph_report.json");
}

void ExpectSlamSessionManifestArtifacts(
    const SmartDrone::Core::Application::EpgTaskManifest &system,
    const SmartDrone::Core::Application::EpgTaskManifest &slam)
{
    const auto slamTaskTypes = EpgTaskCatalogTypes(slam);
    EXPECT_EQ(slamTaskTypes.size(), 16u);
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
}

void ExpectCalibSessionManifestArtifacts(
    const SmartDrone::Core::Application::EpgTaskManifest &system,
    const SmartDrone::Core::Application::EpgTaskManifest &calib)
{
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
