#include "common/epg/epg.h"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using epg::ITask;
using epg::OverflowPolicy;
using epg::PortSpec;
using epg::Registry;
using epg::EventPipelineGraph;
using epg::SpscSharedPtrQueue;
using epg::TaskContext;

struct TestPacket {
    int sequence{};
};

struct OtherPacket {
    int value{};
};

struct ReflectedPacket {
    int sequence{};
};
EPG_REGISTER_MESSAGE(ReflectedPacket, "ReflectedPacket")

struct NativeSlamResourceReady {};
struct NativeSlamTick {};
struct NativeSlamFrameReady {};
struct NativeSlamPreparedFrame {};
struct NativeSlamTrackedFrame {};
struct NativeSlamPublishedFrame {};
struct NativeSlamStatus {};
struct CalibResourceReady {};
struct NativeCalibTick {};
struct CalibStereoFrame {};
struct CalibSavePair {};
struct CalibCaptureDone {};
struct CalibStorageStatus {};
struct CalibImuStatus {};
struct CalibPreviewStatus {};
struct CalibFlushRequest {};
struct NativeCalibStatus {};

class TestSourceTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        auto packet = context.Make<TestPacket>();
        packet->sequence = ++m_sequence;
        context.Push<TestPacket>(0, std::move(packet));
    }

private:
    int m_sequence{};
};

class TestSecondSourceTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        auto packet = context.Make<TestPacket>();
        packet->sequence = ++m_sequence;
        context.Push<TestPacket>(0, std::move(packet));
    }

private:
    int m_sequence{};
};

class TestFanoutSourceTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        ++m_sequence;

        auto left = context.Make<TestPacket>();
        left->sequence = m_sequence;
        context.Push<TestPacket>(0, std::move(left));

        auto right = context.Make<TestPacket>();
        right->sequence = m_sequence;
        context.Push<TestPacket>(1, std::move(right));
    }

private:
    int m_sequence{};
};

class TestSinkTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<TestPacket>(0)) {
            (void)packet;
        }
    }
};

class TestForwardTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<TestPacket>(0)) {
            auto forwarded = context.Make<TestPacket>();
            forwarded->sequence = packet->sequence;
            context.Push<TestPacket>(0, std::move(forwarded));
        }
    }
};

class TestAllInputsSinkTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        auto left = context.TryPop<TestPacket>(0);
        auto right = context.TryPop<TestPacket>(1);
        if (!left || !right) {
            throw std::runtime_error("all-input sink woke before both queues were ready");
        }
    }
};

class TestHeartbeatTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        (void)context;
    }
};

class TestThrowingTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        (void)context;
        throw std::runtime_error("expected test exception");
    }
};

class TestBadContextTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        (void)context.TryPop<TestPacket>(999);
    }
};

class TestCountingTask final : public ITask {
public:
    explicit TestCountingTask(int& ticks) : m_ticks(ticks) {}

    void OnTick(TaskContext& context) override {
        (void)context;
        ++m_ticks;
    }

private:
    int& m_ticks;
};

class TestCountingSinkTask final : public ITask {
public:
    explicit TestCountingSinkTask(int& packets) : m_packets(packets) {}

    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<TestPacket>(0)) {
            (void)packet;
            ++m_packets;
        }
    }

private:
    int& m_packets;
};

class ReflectedSourceTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        auto packet = context.Make<ReflectedPacket>();
        packet->sequence = ++m_sequence;
        context.Push(0, std::move(packet));
    }

private:
    int m_sequence{};
};
EPG_REGISTER_TASK(
    ReflectedSourceTask, "ReflectedSourceTask",
    std::vector<epg::PortSpec>{},
    std::vector<epg::PortSpec>{
        EPG_PORT(0, "ReflectedPacket")})

class ReflectedSinkTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<ReflectedPacket>(0)) {
            (void)packet;
        }
    }
};
EPG_REGISTER_TASK(
    ReflectedSinkTask, "ReflectedSinkTask",
    std::vector<epg::PortSpec>{
        EPG_PORT(0, "ReflectedPacket")},
    std::vector<epg::PortSpec>{})

Registry MakeRegistry() {
    Registry registry;
    registry.RegisterMessageType<TestPacket>("TestPacket");
    registry.RegisterMessageType<OtherPacket>("OtherPacket");
    registry.RegisterTaskType<TestSourceTask>(
        "TestSourceTask",
        {},
        {PortSpec{0, "TestPacket"}});
    registry.RegisterTaskType<TestSecondSourceTask>(
        "TestSecondSourceTask",
        {},
        {PortSpec{0, "TestPacket"}});
    registry.RegisterTaskType<TestFanoutSourceTask>(
        "TestFanoutSourceTask",
        {},
        {PortSpec{0, "TestPacket"}, PortSpec{1, "TestPacket"}});
    registry.RegisterTaskType<TestSinkTask>(
        "TestSinkTask",
        {PortSpec{0, "TestPacket"}},
        {});
    registry.RegisterTaskType<TestForwardTask>(
        "TestForwardTask",
        {PortSpec{0, "TestPacket"}},
        {PortSpec{0, "TestPacket"}});
    registry.RegisterTaskType<TestAllInputsSinkTask>(
        "TestAllInputsSinkTask",
        {PortSpec{0, "TestPacket"}, PortSpec{1, "TestPacket"}},
        {});
    registry.RegisterTaskType<TestHeartbeatTask>(
        "TestHeartbeatTask",
        {},
        {});
    registry.RegisterTaskType<TestThrowingTask>(
        "TestThrowingTask",
        {},
        {});
    registry.RegisterTaskType<TestBadContextTask>(
        "TestBadContextTask",
        {},
        {});
    return registry;
}

Registry MakeNativeSlamShapeRegistry() {
    Registry registry;
    registry.RegisterMessageType<NativeSlamResourceReady>("NativeSlamResourceReady");
    registry.RegisterMessageType<NativeSlamTick>("NativeSlamTick");
    registry.RegisterMessageType<NativeSlamFrameReady>("NativeSlamFrameReady");
    registry.RegisterMessageType<NativeSlamPreparedFrame>("NativeSlamPreparedFrame");
    registry.RegisterMessageType<NativeSlamTrackedFrame>("NativeSlamTrackedFrame");
    registry.RegisterMessageType<NativeSlamPublishedFrame>("NativeSlamPublishedFrame");
    registry.RegisterMessageType<NativeSlamStatus>("NativeSlamStatus");

    const auto factory = []() {
        return std::unique_ptr<ITask>(new TestHeartbeatTask());
    };
    registry.RegisterTaskFactory("NativeSlamResourceTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamClockTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamImuGateTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamAcquireTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamTrackingTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamPosePostprocessTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamPointCloudTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamLivePoseTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamMavlinkTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamUdpTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamDfxTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeSlamMonitorTask", {}, {}, factory);
    return registry;
}

Registry MakeNativeCalibShapeRegistry() {
    Registry registry;
    registry.RegisterMessageType<CalibResourceReady>("CalibResourceReady");
    registry.RegisterMessageType<NativeCalibTick>("NativeCalibTick");
    registry.RegisterMessageType<CalibStereoFrame>("CalibStereoFrame");
    registry.RegisterMessageType<CalibSavePair>("CalibSavePair");
    registry.RegisterMessageType<CalibCaptureDone>("CalibCaptureDone");
    registry.RegisterMessageType<CalibStorageStatus>("CalibStorageStatus");
    registry.RegisterMessageType<CalibImuStatus>("CalibImuStatus");
    registry.RegisterMessageType<CalibPreviewStatus>("CalibPreviewStatus");
    registry.RegisterMessageType<CalibFlushRequest>("CalibFlushRequest");
    registry.RegisterMessageType<NativeCalibStatus>("NativeCalibStatus");

    const auto factory = []() {
        return std::unique_ptr<ITask>(new TestHeartbeatTask());
    };
    registry.RegisterTaskFactory("CalibResourceTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeCalibClockTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibCameraAcquireTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibPacingFilterTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibStorageWriteTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibImuWriterTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibUdpPreviewTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibCompletionTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibFlushSyncTask", {}, {}, factory);
    registry.RegisterTaskFactory("NativeCalibMonitorTask", {}, {}, factory);
    return registry;
}

std::string MinimalValidJson() {
    return R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "packets"}
        },
        {
          "name": "sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })";
}

void ExpectConfigureThrows(const std::string& json) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);
    EXPECT_THROW(graph.ConfigureJson(json), std::runtime_error);
}

void RunTopology(const epg::GraphConfig& config,
                 const std::vector<std::string>& queues,
                 const std::vector<std::string>& tasks) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(config);

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    graph.Stop();

    const auto queueDiagnostics = graph.QueueDiagnostics();
    for (const auto& queueName : queues) {
        const auto queueIt = queueDiagnostics.find(queueName);
        ASSERT_NE(queueIt, queueDiagnostics.end()) << queueName;
        EXPECT_GT(queueIt->second.pushed, 0u) << queueName;
        EXPECT_GT(queueIt->second.popped, 0u) << queueName;
        EXPECT_GT(queueIt->second.wakeups, 0u) << queueName;
    }

    const auto taskDiagnostics = graph.TaskDiagnostics();
    for (const auto& taskName : tasks) {
        const auto taskIt = taskDiagnostics.find(taskName);
        ASSERT_NE(taskIt, taskDiagnostics.end()) << taskName;
        EXPECT_GT(taskIt->second.loopCount, 0u) << taskName;
        EXPECT_EQ(taskIt->second.errorCount, 0u) << taskName;
    }
}

void RunTopologyFromJsonFile(const std::string& jsonFile,
                             const std::vector<std::string>& queues,
                             const std::vector<std::string>& tasks) {
    RunTopology(
        epg::ParseGraphConfigJsonFile(
            std::string(TEST_EPG_DIR) + "/" + jsonFile),
        queues,
        tasks);
}

void RunTopologyFromDotFile(const std::string& dotFile,
                            const std::vector<std::string>& queues,
                            const std::vector<std::string>& tasks) {
    auto registry = MakeRegistry();
    RunTopology(
        epg::ParseGraphConfigDotFile(
            std::string(TEST_EPG_DIR) + "/" + dotFile,
            "cluster_test_graph",
            registry),
        queues,
        tasks);
}

} // namespace

TEST(EventPipelineGraphQueue, DropNewestKeepsOldItemsAndCountsDrops) {
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

TEST(EventPipelineGraphQueue, OverwriteOldestKeepsNewestItemsAndCountsOverwrites) {
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

TEST(EventPipelineGraphQueue, TryPopLatestDrainsQueueAndReturnsNewestItem) {
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

TEST(EventPipelineGraphQueue, NotifierRunsForAcceptedPushesOnly) {
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 1, OverflowPolicy::DropNewest);
    int notifications = 0;
    queue.SetNotifier([&notifications]() { ++notifications; });

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_FALSE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));

    EXPECT_EQ(notifications, 1);
    EXPECT_EQ(queue.Diagnostics().wakeups, 1u);
}

TEST(EventPipelineGraphIngress, ExternalInterruptEventWakesConsumerTask) {
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

TEST(EventPipelineGraphIngress, RejectsTypeMismatchDuplicateIngressAndTaskProducerConflict) {
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

TEST(EventPipelineGraph, RunsPipelineConfiguredFromJsonFile) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(epg::ParseGraphConfigJsonFile(
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

TEST(EventPipelineGraph, TaskCanPublishToMultipleQueuesForDifferentConsumers) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(epg::ParseGraphConfigJsonFile(
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

TEST(EventPipelineGraphTopology, RunsLinearChainTopologyFromJson) {
    RunTopologyFromJsonFile(
        "chain_pipeline.json",
        {"source_to_forward", "forward_to_sink"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphTopology, RunsParallelIndependentTopologyFromJson) {
    RunTopologyFromJsonFile(
        "parallel_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(EventPipelineGraphTopology, RunsFanInJoinTopologyFromJson) {
    RunTopologyFromJsonFile(
        "fanin_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "right_source", "join_sink"});
}

TEST(EventPipelineGraphTopology, RunsDiamondTopologyFromJson) {
    RunTopologyFromJsonFile(
        "diamond_pipeline.json",
        {"left_packets", "right_packets", "forwarded_left_packets"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(EventPipelineGraphMermaid, ConvertsMermaidTopologyToRuntimeConfig) {
    auto registry = MakeRegistry();
    const auto config = epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
        forward["type=TestForwardTask; trigger=any_queue_ready"]
        sink["type=TestSinkTask; trigger=any_queue_ready"]

        source -->|"type=TestPacket; depth=8; overflow=drop_newest"| forward
        forward -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
    )", registry);

    ASSERT_EQ(config.queues.size(), 2u);
    EXPECT_EQ(config.queues[0].name, "source_0_to_forward_0");
    EXPECT_EQ(config.queues[0].type, "TestPacket");
    EXPECT_EQ(config.queues[0].depth, 8u);
    EXPECT_EQ(config.queues[1].name, "forward_0_to_sink_0");

    ASSERT_EQ(config.tasks.size(), 3u);
    EXPECT_EQ(config.tasks[0].name, "source");
    EXPECT_EQ(config.tasks[0].outputs.at(0), "source_0_to_forward_0");
    EXPECT_EQ(config.tasks[1].inputs.at(0), "source_0_to_forward_0");
    EXPECT_EQ(config.tasks[1].outputs.at(0), "forward_0_to_sink_0");
    EXPECT_EQ(config.tasks[1].trigger.queues, std::vector<std::string>{"source_0_to_forward_0"});
    EXPECT_EQ(config.tasks[2].inputs.at(0), "forward_0_to_sink_0");
    EXPECT_EQ(config.tasks[2].trigger.queues, std::vector<std::string>{"forward_0_to_sink_0"});
}

TEST(EventPipelineGraphDot, RunsTopologyCompiledFromDotFile) {
    RunTopologyFromDotFile(
        "chain_pipeline.dot",
        {"source_0_to_forward_0", "forward_0_to_sink_0"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphMermaid, ConvertsMarkdownMermaidBlockToRuntimeConfig) {
    auto registry = MakeRegistry();
    const auto config = epg::ParseGraphConfigMermaid(R"(
# EventPipelineGraph

```mermaid
flowchart LR
  source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
  sink["type=TestSinkTask; trigger=any_queue_ready"]
  source -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
```
    )", registry);

    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues[0].name, "source_0_to_sink_0");
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks[0].outputs.at(0), "source_0_to_sink_0");
    EXPECT_EQ(config.tasks[1].inputs.at(0), "source_0_to_sink_0");
}

TEST(EventPipelineGraphDot, CompilesNativeSlamSubgraphFromMaintainedTopology) {
    auto registry = MakeNativeSlamShapeRegistry();
    const auto config = epg::ParseGraphConfigDotFile(
        std::string(TEST_EPG_DIR) + "/../../config/epg/native_epg_topology.dot",
        "cluster_slam_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 12u);
    ASSERT_EQ(config.queues.size(), 11u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string& name) -> const epg::TaskConfig* {
        for (const auto& task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto* resource = findTask("NativeSlamResourceTask");
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->trigger.mode, epg::TriggerMode::Periodic);
    EXPECT_EQ(resource->trigger.interval, std::chrono::milliseconds(1));

    const auto* clock = findTask("NativeSlamClockTask");
    ASSERT_NE(clock, nullptr);
    EXPECT_EQ(clock->trigger.mode, epg::TriggerMode::Periodic);
    EXPECT_EQ(clock->trigger.interval, std::chrono::milliseconds(1));

    const auto* imuGate = findTask("NativeSlamImuGateTask");
    ASSERT_NE(imuGate, nullptr);
    EXPECT_EQ(imuGate->inputs.at(0), "NativeSlamResourceTask_0_to_NativeSlamImuGateTask_0");
    EXPECT_EQ(imuGate->inputs.at(1), "NativeSlamClockTask_0_to_NativeSlamImuGateTask_1");
    EXPECT_EQ(imuGate->trigger.queues,
              (std::vector<std::string>{"NativeSlamResourceTask_0_to_NativeSlamImuGateTask_0",
                                         "NativeSlamClockTask_0_to_NativeSlamImuGateTask_1"}));

    const auto* acquire = findTask("NativeSlamAcquireTask");
    ASSERT_NE(acquire, nullptr);
    EXPECT_EQ(acquire->inputs.at(0),
              "NativeSlamImuGateTask_0_to_NativeSlamAcquireTask_0");
    EXPECT_EQ(acquire->trigger.queues,
              (std::vector<std::string>{"NativeSlamImuGateTask_0_to_NativeSlamAcquireTask_0"}));

    const auto* tracking = findTask("NativeSlamTrackingTask");
    ASSERT_NE(tracking, nullptr);
    EXPECT_EQ(tracking->inputs.at(0),
              "NativeSlamAcquireTask_0_to_NativeSlamTrackingTask_0");
    EXPECT_EQ(tracking->trigger.queues,
              (std::vector<std::string>{"NativeSlamAcquireTask_0_to_NativeSlamTrackingTask_0"}));
}

TEST(EventPipelineGraphDot, CompilesNativeCalibSubgraphFromMaintainedTopology) {
    auto registry = MakeNativeCalibShapeRegistry();
    const auto config = epg::ParseGraphConfigDotFile(
        std::string(TEST_EPG_DIR) + "/../../config/epg/native_epg_topology.dot",
        "cluster_calib_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 10u);
    ASSERT_EQ(config.queues.size(), 12u);

    EventPipelineGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string& name) -> const epg::TaskConfig* {
        for (const auto& task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto* camera = findTask("CalibCameraAcquireTask");
    ASSERT_NE(camera, nullptr);
    EXPECT_EQ(camera->inputs.at(0), "CalibResourceTask_0_to_CalibCameraAcquireTask_0");
    EXPECT_EQ(camera->inputs.at(1), "NativeCalibClockTask_0_to_CalibCameraAcquireTask_1");

    const auto* pace = findTask("CalibPacingFilterTask");
    ASSERT_NE(pace, nullptr);
    EXPECT_EQ(pace->inputs.at(0), "CalibCameraAcquireTask_0_to_CalibPacingFilterTask_0");

    const auto* preview = findTask("CalibUdpPreviewTask");
    ASSERT_NE(preview, nullptr);
    EXPECT_EQ(preview->inputs.at(0), "CalibCameraAcquireTask_1_to_CalibUdpPreviewTask_0");
}

TEST(EventPipelineGraphDotTopology, RunsBasicTopologyFromDot) {
    RunTopologyFromDotFile(
        "basic_pipeline.dot",
        {"source_0_to_sink_0"},
        {"source", "sink"});
}

TEST(EventPipelineGraphDotTopology, RunsFanoutTopologyFromDot) {
    RunTopologyFromDotFile(
        "fanout_pipeline.dot",
        {"fanout_source_0_to_left_sink_0", "fanout_source_1_to_right_sink_0"},
        {"fanout_source", "left_sink", "right_sink"});
}

TEST(EventPipelineGraphDotTopology, RunsLinearChainTopologyFromDot) {
    RunTopologyFromDotFile(
        "chain_pipeline.dot",
        {"source_0_to_forward_0", "forward_0_to_sink_0"},
        {"source", "forward", "sink"});
}

TEST(EventPipelineGraphDotTopology, RunsParallelIndependentTopologyFromDot) {
    RunTopologyFromDotFile(
        "parallel_pipeline.dot",
        {"left_source_0_to_left_sink_0", "right_source_0_to_right_sink_0"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(EventPipelineGraphDotTopology, RunsFanInJoinTopologyFromDot) {
    RunTopologyFromDotFile(
        "fanin_pipeline.dot",
        {"left_source_0_to_join_sink_0", "right_source_0_to_join_sink_1"},
        {"left_source", "right_source", "join_sink"});
}

TEST(EventPipelineGraphDotTopology, RunsDiamondTopologyFromDot) {
    RunTopologyFromDotFile(
        "diamond_pipeline.dot",
        {"fanout_source_0_to_left_forward_0",
         "fanout_source_1_to_join_sink_1",
         "left_forward_0_to_join_sink_0"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(EventPipelineGraphMermaid, RejectsInvalidMermaidTopology) {
    EXPECT_THROW(epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source[type=TestSourceTask; trigger=periodic; interval_ms=1]
        source.out -->|type=TestPacket; depth=8; overflow=drop_newest| missing.in
    )"), std::runtime_error);

    EXPECT_THROW(epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source[type=TestSourceTask; trigger=periodic; interval_ms=1]
        sink[type=TestSinkTask; trigger=any_queue_ready]
        source.out -->|type=TestPacket; overflow=drop_newest| sink.in
    )"), std::runtime_error);

    auto registry = MakeRegistry();
    EXPECT_THROW(epg::ParseGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
        sink["type=TestSinkTask; trigger=any_queue_ready; trigger_queues=OtherPacket"]
        source -->|"type=TestPacket; depth=1; overflow=drop_newest"| sink
    )", registry), std::runtime_error);
}

TEST(EventPipelineGraph, RejectsMultipleConsumersForSpscQueue) {
    const char* json = R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "packets"}
        },
        {
          "name": "sink_a",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        },
        {
          "name": "sink_b",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })";

    ExpectConfigureThrows(json);
}

TEST(EventPipelineGraph, AllQueueReadyWaitsForAllInputs) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "left_packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"},
        {"name": "right_packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "left_source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "left_packets"}
        },
        {
          "name": "right_source",
          "type": "TestSecondSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 2},
          "outputs": {"0": "right_packets"}
        },
        {
          "name": "all_sink",
          "type": "TestAllInputsSinkTask",
          "trigger": {"mode": "all_queue_ready", "queues": ["left_packets", "right_packets"]},
          "inputs": {"0": "left_packets", "1": "right_packets"}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    graph.Stop();

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("all_sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("all_sink").errorCount, 0u);
}

TEST(EventPipelineGraph, PeriodicOrAnyQueueReadyRunsWithQueueTrigger) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "packets"}
        },
        {
          "name": "hybrid_sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "periodic_or_any_queue_ready", "interval_ms": 50, "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    graph.Stop();

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("hybrid_sink").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("hybrid_sink").errorCount, 0u);
}

TEST(EventPipelineGraph, PeriodicTaskCanRunWithoutQueues) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "heartbeat",
          "type": "TestHeartbeatTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const auto taskDiagnostics = graph.TaskDiagnostics();
    EXPECT_GT(taskDiagnostics.at("heartbeat").loopCount, 0u);
    EXPECT_EQ(taskDiagnostics.at("heartbeat").errorCount, 0u);
}

TEST(EventPipelineGraph, SupportsCapturedTaskFactoryForNativeAdapters) {
    int ticks = 0;
    Registry registry;
    registry.RegisterTaskFactory(
        "TestCountingTask",
        {},
        {},
        [&ticks]() {
            return std::unique_ptr<ITask>(new TestCountingTask(ticks));
        });

    EventPipelineGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "counter",
          "type": "TestCountingTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    EXPECT_GT(ticks, 0);
    EXPECT_GT(graph.TaskDiagnostics().at("counter").loopCount, 0u);
}

TEST(EventPipelineGraph, TaskExceptionsAreCountedAndRunnerKeepsAlive) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "thrower",
          "type": "TestThrowingTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const auto diag = graph.TaskDiagnostics().at("thrower");
    EXPECT_GT(diag.loopCount, 0u);
    EXPECT_EQ(diag.errorCount, diag.loopCount);
}

TEST(EventPipelineGraph, ContextSlotErrorsAreCountedAsTaskErrors) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "bad_context",
          "type": "TestBadContextTask",
          "trigger": {"mode": "periodic", "interval_ms": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const auto diag = graph.TaskDiagnostics().at("bad_context");
    EXPECT_GT(diag.errorCount, 0u);
}

TEST(EventPipelineGraph, LifecycleStateAndAccessorsBehaveAsExpected) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    EXPECT_FALSE(graph.Running());
    EXPECT_THROW(graph.Start(), std::runtime_error);

    graph.ConfigureJson(MinimalValidJson());
    ASSERT_NE(graph.Queue("packets"), nullptr);
    EXPECT_EQ(graph.Queue("missing"), nullptr);
    EXPECT_FALSE(graph.Running());

    graph.Start();
    EXPECT_TRUE(graph.Running());
    EXPECT_THROW(graph.ConfigureJson(MinimalValidJson()), std::runtime_error);
    graph.Stop();
    EXPECT_FALSE(graph.Running());

    graph.Stop();
    EXPECT_FALSE(graph.Running());
}

TEST(GraphConfig, ParsesEscapesAndRejectsMissingFile) {
    const auto config = epg::ParseGraphConfigJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "tail_drop"}
      ],
      "tasks": [
        {
          "name": "source\nname",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "packets"}
        }
      ]
    })");

    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.tasks.front().name, "source\nname");
    EXPECT_THROW(
        epg::ParseGraphConfigJsonFile("/tmp/smart_drone_missing_epg.json"),
        std::runtime_error);
}

TEST(GraphConfig, RejectsInvalidJsonAndUnsupportedEnumValues) {
    EXPECT_THROW(epg::ParseGraphConfigJson("{"), std::runtime_error);
    EXPECT_THROW(epg::ParseGraphConfigJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "bad_policy"}
      ],
      "tasks": []
    })"), std::runtime_error);
    EXPECT_THROW(epg::ParseGraphConfigJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "heartbeat",
          "type": "TestHeartbeatTask",
          "trigger": {"mode": "bad_trigger"}
        }
      ]
    })"), std::runtime_error);
}

TEST(EventPipelineGraphReflection, RegistersMessagesAndTaskPortsFromCatalog) {
    Registry registry;
    epg::TypeCatalog::Global().RegisterReflectedMessageTypes(registry);
    epg::TypeCatalog::Global().RegisterReflectedTaskTypes(
        registry,
        {"ReflectedSourceTask", "ReflectedSinkTask"},
        [](const std::string& type) {
            if (type == "ReflectedSourceTask") {
                return Registry::TaskFactory([]() {
                    return std::unique_ptr<ITask>(new ReflectedSourceTask());
                });
            }
            if (type == "ReflectedSinkTask") {
                return Registry::TaskFactory([]() {
                    return std::unique_ptr<ITask>(new ReflectedSinkTask());
                });
            }
            return Registry::TaskFactory{};
        });

    EventPipelineGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [
        {"name": "reflected_packets", "type": "ReflectedPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "ReflectedSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"0": "reflected_packets"}
        },
        {
          "name": "sink",
          "type": "ReflectedSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["reflected_packets"]},
          "inputs": {"0": "reflected_packets"}
        }
      ]
    })");
    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    graph.Stop();

    const auto taskDiag = graph.TaskDiagnostics();
    EXPECT_GT(taskDiag.at("source").loopCount, 0u);
    EXPECT_GT(taskDiag.at("sink").loopCount, 0u);
}

TEST(EventPipelineGraphValidation, RejectsInvalidTopologyConfigurations) {
    const std::vector<std::string> invalidJsons = {
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"},{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[{"name":"packets","type":"MissingPacket","depth":4,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":0,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1}},{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1}}]})",
        R"({"queues":[],"tasks":[{"name":"missing","type":"MissingTask","trigger":{"mode":"periodic","interval_ms":1}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":0}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":[]},"inputs": {"0":"packets"}}]})",
        R"({"queues":[],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["missing"]},"inputs": {"0":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs": {"999":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"999":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs": {"0":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"OtherPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs": {"0":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"OtherPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source_a","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"packets"}},{"name":"source_b","type":"TestSecondSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs": {"0":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"},{"name":"other","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["other"]},"inputs": {"0":"packets"}}]})"
    };

    for (const auto& json : invalidJsons) {
        ExpectConfigureThrows(json);
    }
}
