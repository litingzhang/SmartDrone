TEST(EventPipelineGraphLifecycle, ResetForStartClearsStopRequest)
{
    std::atomic<bool> stop{true};
    SmartDrone::Core::Application::EpgGraphLifecycle lifecycle({
        stop,
        []() { return true; },
        []() {},
        []() {},
    });

    lifecycle.ResetForStart();

    EXPECT_FALSE(stop.load());
    EXPECT_FALSE(lifecycle.Done());
    EXPECT_FALSE(lifecycle.StopRequested());
}

TEST(EventPipelineGraphRedeploy, TracksSystemAndSessionRequestsIndependently)
{
    SmartDrone::Core::Application::EpgRedeployCoordinator redeploy;

    redeploy.RequestSystemRedeploy({
        "cluster_system_runtime_graph",
        "optimized config changed",
        "config/epg/epg_topology.dot:v4",
        "cluster_system_runtime_graph",
        "/tmp/smartdrone_epg_system_profile.json",
        1000,
        1010,
        "native-exact-v1",
        "output/epg/optimized_system_runtime_graph.json",
        "output/epg/optimized_system_runtime_graph_report.json",
    });
    EXPECT_TRUE(redeploy.SystemRedeployRequested());
    EXPECT_FALSE(redeploy.SessionRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest systemRequest;
    EXPECT_TRUE(redeploy.TakeSystemRedeployRequest(systemRequest));
    EXPECT_EQ(systemRequest.graphName, "cluster_system_runtime_graph");
    EXPECT_EQ(systemRequest.reason, "optimized config changed");
    EXPECT_EQ(systemRequest.topologyVersion, "config/epg/epg_topology.dot:v4");
    EXPECT_EQ(systemRequest.sourceProfile, "cluster_system_runtime_graph");
    EXPECT_EQ(systemRequest.sourceProfilePath,
              "/tmp/smartdrone_epg_system_profile.json");
    EXPECT_EQ(systemRequest.sourceTimestampMs, 1000u);
    EXPECT_EQ(systemRequest.generatedAtMs, 1010u);
    EXPECT_EQ(systemRequest.solverVersion, "native-exact-v1");
    EXPECT_EQ(systemRequest.optimizedConfigPath,
              "output/epg/optimized_system_runtime_graph.json");
    EXPECT_EQ(systemRequest.solverReportPath,
              "output/epg/optimized_system_runtime_graph_report.json");
    const auto description =
        SmartDrone::Core::Application::DescribeEpgRedeployRequest(
            systemRequest);
    EXPECT_NE(description.find("graph=cluster_system_runtime_graph"),
              std::string::npos);
    EXPECT_NE(description.find("source_ts_ms=1000"), std::string::npos);
    EXPECT_NE(
        description.find(
            "optimized=output/epg/optimized_system_runtime_graph.json"),
        std::string::npos);
    EXPECT_FALSE(redeploy.TakeSystemRedeployRequest(systemRequest));

    redeploy.RequestSessionRedeploy({
        "cluster_slam_session_graph",
        "optimized config changed",
        "config/epg/epg_topology.dot:v4",
        "cluster_slam_session_graph",
        "/tmp/smartdrone_epg_slam_profile.json",
        2000,
        2010,
        "native-exact-v1",
        "output/epg/optimized_slam_session_graph.json",
        "output/epg/optimized_slam_session_graph_report.json",
    });
    EXPECT_TRUE(redeploy.SessionRedeployRequested());
    EXPECT_FALSE(redeploy.SystemRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest sessionRequest;
    EXPECT_TRUE(redeploy.TakeSessionRedeployRequest(sessionRequest));
    EXPECT_EQ(sessionRequest.graphName, "cluster_slam_session_graph");
    EXPECT_EQ(sessionRequest.reason, "optimized config changed");
    EXPECT_EQ(sessionRequest.sourceTimestampMs, 2000u);
    EXPECT_EQ(sessionRequest.generatedAtMs, 2010u);
    EXPECT_EQ(sessionRequest.optimizedConfigPath,
              "output/epg/optimized_slam_session_graph.json");
    EXPECT_EQ(sessionRequest.solverReportPath,
              "output/epg/optimized_slam_session_graph_report.json");
    EXPECT_FALSE(redeploy.TakeSessionRedeployRequest(sessionRequest));
}
