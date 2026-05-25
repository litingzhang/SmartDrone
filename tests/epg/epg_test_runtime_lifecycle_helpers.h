void RequestTestSystemRedeploy(
    SmartDrone::Core::Application::EpgRedeployCoordinator &redeploy)
{
    redeploy.RequestSystemRedeploy({
        "cluster_system_runtime_graph",
        "optimized config changed",
        "config/epg/epg_topology.dot:v6",
        "cluster_system_runtime_graph",
        "/tmp/smartdrone_epg_system_profile.json",
        1000,
        1010,
        "native-exact-v1",
        "output/epg/optimized_system_runtime_graph.json",
        "output/epg/optimized_system_runtime_graph_report.json",
    });
}

void ExpectTestSystemRedeployRequest(
    const SmartDrone::Core::Application::EpgRedeployRequest &request)
{
    EXPECT_EQ(request.graphName, "cluster_system_runtime_graph");
    EXPECT_EQ(request.reason, "optimized config changed");
    EXPECT_EQ(request.topologyVersion, "config/epg/epg_topology.dot:v6");
    EXPECT_EQ(request.sourceProfile, "cluster_system_runtime_graph");
    EXPECT_EQ(request.sourceProfilePath,
              "/tmp/smartdrone_epg_system_profile.json");
    EXPECT_EQ(request.sourceTimestampMs, 1000u);
    EXPECT_EQ(request.generatedAtMs, 1010u);
    EXPECT_EQ(request.solverVersion, "native-exact-v1");
    EXPECT_EQ(request.optimizedConfigPath,
              "output/epg/optimized_system_runtime_graph.json");
    EXPECT_EQ(request.solverReportPath,
              "output/epg/optimized_system_runtime_graph_report.json");
}

void ExpectTestSystemRedeployDescription(
    const SmartDrone::Core::Application::EpgRedeployRequest &request)
{
    const auto description =
        SmartDrone::Core::Application::DescribeEpgRedeployRequest(request);
    EXPECT_NE(description.find("graph=cluster_system_runtime_graph"),
              std::string::npos);
    EXPECT_NE(description.find("source_ts_ms=1000"), std::string::npos);
    EXPECT_NE(
        description.find(
            "optimized=output/epg/optimized_system_runtime_graph.json"),
        std::string::npos);
}

void RequestTestSessionRedeploy(
    SmartDrone::Core::Application::EpgRedeployCoordinator &redeploy)
{
    redeploy.RequestSessionRedeploy({
        "cluster_slam_session_graph",
        "optimized config changed",
        "config/epg/epg_topology.dot:v6",
        "cluster_slam_session_graph",
        "/tmp/smartdrone_epg_slam_profile.json",
        2000,
        2010,
        "native-exact-v1",
        "output/epg/optimized_slam_session_graph.json",
        "output/epg/optimized_slam_session_graph_report.json",
    });
}

void ExpectTestSessionRedeployRequest(
    const SmartDrone::Core::Application::EpgRedeployRequest &request)
{
    EXPECT_EQ(request.graphName, "cluster_slam_session_graph");
    EXPECT_EQ(request.reason, "optimized config changed");
    EXPECT_EQ(request.sourceTimestampMs, 2000u);
    EXPECT_EQ(request.generatedAtMs, 2010u);
    EXPECT_EQ(request.optimizedConfigPath,
              "output/epg/optimized_slam_session_graph.json");
    EXPECT_EQ(request.solverReportPath,
              "output/epg/optimized_slam_session_graph_report.json");
}
