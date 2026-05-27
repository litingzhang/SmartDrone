TEST(EventPipelineGraphTopology, RunsLinearChainTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "chain_pipeline.json",
        {"source_to_forward", "forward_to_sink"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphTopology, RunsParallelIndependentTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "parallel_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(EventPipelineGraphTopology, RunsFanInJoinTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "fanin_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "right_source", "join_sink"});
}

TEST(EventPipelineGraphTopology, RunsDiamondTopologyFromJson)
{
    RunTopologyFromJsonFile(
        "diamond_pipeline.json",
        {"left_packets", "right_packets", "forwarded_left_packets"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(EventPipelineGraphMermaid, ConvertsMermaidTopologyToRuntimeConfig)
{
    auto registry = MakeRegistry();
    const auto config = Epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1; resource=cpu; cpu_affinity=-1; budget_us=500; deadline_us=900; backpressure_outputs=0; realtime=true; priority=42"]
        forward["type=TestForwardTask; trigger=any_queue_ready"]
        sink["type=TestSinkTask; trigger=any_queue_ready"]

        source -->|"type=TestPacket; depth=8; overflow=drop_newest"| forward
        forward -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
    )",
                                                     registry);

    ASSERT_EQ(config.queues.size(), 2u);
    EXPECT_EQ(config.queues[0].name, "source_0_to_forward_0");
    EXPECT_EQ(config.queues[0].type, "TestPacket");
    EXPECT_EQ(config.queues[0].depth, 8u);
    EXPECT_EQ(config.queues[1].name, "forward_0_to_sink_0");

    ASSERT_EQ(config.tasks.size(), 3u);
    EXPECT_EQ(config.tasks[0].name, "source");
    EXPECT_EQ(config.tasks[0].scheduling.resource, "cpu");
    EXPECT_EQ(config.tasks[0].scheduling.cpuAffinity, -1);
    EXPECT_EQ(config.tasks[0].scheduling.budgetUs, 500u);
    EXPECT_EQ(config.tasks[0].scheduling.deadlineUs, 900u);
    EXPECT_EQ(config.tasks[0].scheduling.backpressureOutputs,
              std::vector<Epg::PortId>{0});
    EXPECT_TRUE(config.tasks[0].scheduling.realtime);
    EXPECT_EQ(config.tasks[0].scheduling.priority, 42);
    EXPECT_EQ(config.tasks[0].outputs.at(0), "source_0_to_forward_0");
    EXPECT_EQ(config.tasks[1].inputs.at(0), "source_0_to_forward_0");
    EXPECT_EQ(config.tasks[1].outputs.at(0), "forward_0_to_sink_0");
    EXPECT_EQ(config.tasks[1].trigger.queues, std::vector<std::string>{"source_0_to_forward_0"});
    EXPECT_EQ(config.tasks[2].inputs.at(0), "forward_0_to_sink_0");
    EXPECT_EQ(config.tasks[2].trigger.queues, std::vector<std::string>{"forward_0_to_sink_0"});
}

TEST(EventPipelineGraphDot, RunsTopologyCompiledFromDotFile)
{
    RunTopologyFromDotFile(
        "chain_pipeline.dot",
        {"source_0_to_forward_0", "forward_0_to_sink_0"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphMermaid, ConvertsMarkdownMermaidBlockToRuntimeConfig)
{
    auto registry = MakeRegistry();
    const auto config = Epg::ParseGraphConfigMermaid(R"(
# EventPipelineGraph

```mermaid
flowchart LR
  source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
  sink["type=TestSinkTask; trigger=any_queue_ready"]
  source -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
```
    )",
                                                     registry);

    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues[0].name, "source_0_to_sink_0");
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks[0].outputs.at(0), "source_0_to_sink_0");
    EXPECT_EQ(config.tasks[1].inputs.at(0), "source_0_to_sink_0");
}

TEST(EventPipelineGraphDot, CompilesSlamSubgraphFromMaintainedTopology)
{
    auto registry = MakeSlamShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_slam_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 21u);
    ASSERT_EQ(config.queues.size(), 35u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    ExpectSlamTimingTopology(config);
    ExpectSlamIngestTopology(config);
    ExpectSlamTrackingTopology(config);
    ExpectSlamOutputTopology(config);
    ExpectSlamPublishTopology(config);
    ExpectSlamMonitorTopology(config);
}

TEST(EventPipelineGraphDot, CompilesSystemRuntimeSubgraphFromMaintainedTopology)
{
    auto registry = MakeSystemShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_system_runtime_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 14u);
    ASSERT_EQ(config.queues.size(), 13u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    ExpectSystemIngressTopology(config);
    ExpectSystemPeriodicTopology(config);
    ExpectSystemControlTopology(config);
    ExpectSystemRedeployTopology(config);
}

TEST(EventPipelineGraphDot, CompilesCalibSubgraphFromMaintainedTopology)
{
    auto registry = MakeCalibShapeRegistry();
    const auto config = Epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_calib_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 11u);
    ASSERT_EQ(config.queues.size(), 13u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    ExpectCalibCaptureTopology(config);
    ExpectCalibStorageTopology(config);
    ExpectCalibCompletionTopology(config);
}
