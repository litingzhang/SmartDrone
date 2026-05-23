TEST(EventPipelineGraphQueue, DropNewestKeepsOldItemsAndCountsDrops)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 2, OverflowPolicy::DropNewest);

    EXPECT_EQ(queue.Name(), "packets");
    EXPECT_EQ(queue.TypeName(), "TestPacket");
    EXPECT_EQ(queue.Depth(), 2u);
    EXPECT_TRUE(queue.Empty());

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));
    EXPECT_FALSE(queue.Push(std::make_shared<TestPacket>(TestPacket{3})));

    EXPECT_EQ(queue.Size(), 2u);
    EXPECT_EQ(queue.TryPop()->sequence, 1);
    EXPECT_EQ(queue.TryPop()->sequence, 2);
    EXPECT_FALSE(queue.TryPop());

    const auto diag = queue.Diagnostics();
    EXPECT_EQ(diag.pushed, 2u);
    EXPECT_EQ(diag.popped, 2u);
    EXPECT_EQ(diag.droppedNewest, 1u);
    EXPECT_EQ(diag.overwrittenOldest, 0u);
    EXPECT_EQ(diag.maxDepthObserved, 2u);
}

TEST(EventPipelineGraphQueue, OverwriteOldestKeepsNewestItemsAndCountsOverwrites)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 2, OverflowPolicy::OverwriteOldest);

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{3})));

    EXPECT_EQ(queue.TryPop()->sequence, 2);
    EXPECT_EQ(queue.TryPop()->sequence, 3);
    EXPECT_FALSE(queue.TryPop());

    const auto diag = queue.Diagnostics();
    EXPECT_EQ(diag.pushed, 3u);
    EXPECT_EQ(diag.popped, 2u);
    EXPECT_EQ(diag.droppedNewest, 0u);
    EXPECT_EQ(diag.overwrittenOldest, 1u);
}

TEST(EventPipelineGraphQueue, TryPopLatestDrainsQueueAndReturnsNewestItem)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 4, OverflowPolicy::DropNewest);

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));
    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{3})));

    const auto latest = queue.TryPopLatest();
    ASSERT_TRUE(latest);
    EXPECT_EQ(latest->sequence, 3);
    EXPECT_TRUE(queue.Empty());
    EXPECT_EQ(queue.Diagnostics().popped, 3u);
}

TEST(EventPipelineGraphQueue, NotifierRunsForAcceptedPushesOnly)
{
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 1, OverflowPolicy::DropNewest);
    int notifications = 0;
    queue.SetNotifier([&notifications]() { ++notifications; });

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_FALSE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));

    EXPECT_EQ(notifications, 1);
    EXPECT_EQ(queue.Diagnostics().wakeups, 1u);
}

TEST(EventPipelineGraphIngress, ExternalInterruptEventWakesConsumerTask)
{
    Registry registry;
    registry.RegisterMessageType<TestPacket>("TestPacket");
    int packets = 0;
    registry.RegisterTaskFactory(
        "TestCountingSinkTask",
        {PortSpec{0, "TestPacket"}},
        {},
        [&packets]() {
            return std::unique_ptr<ITask>(new TestCountingSinkTask(packets));
        });

    EventPipelineGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [
        {"name": "irq_events", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "interrupt_consumer",
          "type": "TestCountingSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["irq_events"]},
          "inputs": {"0": "irq_events"}
        }
      ]
    })");

    auto ingress = graph.CreateExternalIngress<TestPacket>("irq_events");
    ASSERT_TRUE(ingress.Valid());
    EXPECT_EQ(ingress.QueueName(), "irq_events");

    graph.Start();
    EXPECT_TRUE(ingress.Emplace(TestPacket{1}));
    EXPECT_TRUE(ingress.Emplace(TestPacket{2}));

    for (int i = 0; i < 50 && packets < 2; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    graph.Stop();

    EXPECT_EQ(packets, 2);
    const auto queueDiagnostics = graph.QueueDiagnostics().at("irq_events");
    EXPECT_EQ(queueDiagnostics.pushed, 2u);
    EXPECT_EQ(queueDiagnostics.popped, 2u);
    EXPECT_GE(queueDiagnostics.wakeups, 1u);
    EXPECT_GT(graph.TaskDiagnostics().at("interrupt_consumer").loopCount, 0u);
}

TEST(EventPipelineGraphIngress, RejectsTypeMismatchDuplicateIngressAndTaskProducerConflict)
{
    auto registry = MakeRegistry();

    {
        EventPipelineGraph graph(registry);
        graph.ConfigureJson(R"({
          "queues": [
            {"name": "irq_events", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
          ],
          "tasks": [
            {
              "name": "sink",
              "type": "TestSinkTask",
              "trigger": {"mode": "any_queue_ready", "queues": ["irq_events"]},
              "inputs": {"0": "irq_events"}
            }
          ]
        })");

        EXPECT_THROW(graph.CreateExternalIngress<OtherPacket>("irq_events"), std::runtime_error);
        auto ingress = graph.CreateExternalIngress<TestPacket>("irq_events");
        EXPECT_TRUE(ingress.Valid());
        EXPECT_THROW(graph.CreateExternalIngress<TestPacket>("irq_events"), std::runtime_error);
    }

    {
        EventPipelineGraph graph(registry);
        graph.ConfigureJson(MinimalValidJson());
        EXPECT_THROW(graph.CreateExternalIngress<TestPacket>("packets"), std::runtime_error);
    }
}

TEST(EventPipelineGraph, RunsPipelineConfiguredFromJsonFile)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(Epg::ParseGraphConfigJsonFile(
        std::string(TEST_EPG_DIR) + "/basic_pipeline.json"));

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    graph.Stop();

    const auto queueDiagnostics = graph.QueueDiagnostics();
    const auto packets = queueDiagnostics.at("packets");
    EXPECT_GT(packets.pushed, 0u);
    EXPECT_GT(packets.popped, 0u);
    EXPECT_GT(packets.wakeups, 0u);
    EXPECT_LE(packets.maxDepthObserved, 8u);

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("source").loopCount, 0u);
    EXPECT_GT(taskDiagnostics.at("sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("source").errorCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("sink").errorCount, 0u);
}

TEST(EventPipelineGraph, TaskCanPublishToMultipleQueuesForDifferentConsumers)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(Epg::ParseGraphConfigJsonFile(
        std::string(TEST_EPG_DIR) + "/fanout_pipeline.json"));

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    graph.Stop();

    const auto queueDiagnostics = graph.QueueDiagnostics();
    const auto left = queueDiagnostics.at("left_packets");
    const auto right = queueDiagnostics.at("right_packets");

    EXPECT_GT(left.pushed, 0u);
    EXPECT_GT(right.pushed, 0u);
    EXPECT_GT(left.popped, 0u);
    EXPECT_GT(right.popped, 0u);
    EXPECT_GT(left.wakeups, 0u);
    EXPECT_GT(right.wakeups, 0u);

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("fanout_source").loopCount, 0u);
    EXPECT_GT(taskDiagnostics.at("left_sink").loopCount, 0u);
    EXPECT_GT(taskDiagnostics.at("right_sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("fanout_source").errorCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("left_sink").errorCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("right_sink").errorCount, 0u);
}
