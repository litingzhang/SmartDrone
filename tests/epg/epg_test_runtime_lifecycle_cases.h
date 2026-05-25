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

    RequestTestSystemRedeploy(redeploy);
    EXPECT_TRUE(redeploy.SystemRedeployRequested());
    EXPECT_FALSE(redeploy.SessionRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest systemRequest;
    EXPECT_TRUE(redeploy.TakeSystemRedeployRequest(systemRequest));
    ExpectTestSystemRedeployRequest(systemRequest);
    ExpectTestSystemRedeployDescription(systemRequest);
    EXPECT_FALSE(redeploy.TakeSystemRedeployRequest(systemRequest));

    RequestTestSessionRedeploy(redeploy);
    EXPECT_TRUE(redeploy.SessionRedeployRequested());
    EXPECT_FALSE(redeploy.SystemRedeployRequested());
    SmartDrone::Core::Application::EpgRedeployRequest sessionRequest;
    EXPECT_TRUE(redeploy.TakeSessionRedeployRequest(sessionRequest));
    ExpectTestSessionRedeployRequest(sessionRequest);
    EXPECT_FALSE(redeploy.TakeSessionRedeployRequest(sessionRequest));
}
