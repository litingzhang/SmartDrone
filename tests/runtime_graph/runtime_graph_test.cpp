#include "common/runtime_graph/runtime_graph.h"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using smartdrone::runtime_graph::ITask;
using smartdrone::runtime_graph::OverflowPolicy;
using smartdrone::runtime_graph::PortSpec;
using smartdrone::runtime_graph::Registry;
using smartdrone::runtime_graph::RuntimeGraph;
using smartdrone::runtime_graph::SpscSharedPtrQueue;
using smartdrone::runtime_graph::TaskContext;

struct TestPacket {
    int sequence{};
};

struct OtherPacket {
    int value{};
};

struct ReflectedPacket {
    int sequence{};
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_MESSAGE(ReflectedPacket, "ReflectedPacket")

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
        context.Push<TestPacket>("out", std::move(packet));
    }

private:
    int m_sequence{};
};

class TestSecondSourceTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        auto packet = context.Make<TestPacket>();
        packet->sequence = ++m_sequence;
        context.Push<TestPacket>("out", std::move(packet));
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
        context.Push<TestPacket>("left", std::move(left));

        auto right = context.Make<TestPacket>();
        right->sequence = m_sequence;
        context.Push<TestPacket>("right", std::move(right));
    }

private:
    int m_sequence{};
};

class TestSinkTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<TestPacket>("in")) {
            (void)packet;
        }
    }
};

class TestForwardTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<TestPacket>("in")) {
            auto forwarded = context.Make<TestPacket>();
            forwarded->sequence = packet->sequence;
            context.Push<TestPacket>("out", std::move(forwarded));
        }
    }
};

class TestAllInputsSinkTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        auto left = context.TryPop<TestPacket>("left");
        auto right = context.TryPop<TestPacket>("right");
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
        (void)context.TryPop<TestPacket>("missing");
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
        while (auto packet = context.TryPop<TestPacket>("in")) {
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
        context.Push("out", std::move(packet));
    }

private:
    int m_sequence{};
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    ReflectedSourceTask, "ReflectedSourceTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("out", "ReflectedPacket")})

class ReflectedSinkTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        while (auto packet = context.TryPop<ReflectedPacket>("in")) {
            (void)packet;
        }
    }
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    ReflectedSinkTask, "ReflectedSinkTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("in", "ReflectedPacket")},
    std::vector<smartdrone::runtime_graph::PortSpec>{})

Registry MakeRegistry() {
    Registry registry;
    registry.RegisterMessageType<TestPacket>("TestPacket");
    registry.RegisterMessageType<OtherPacket>("OtherPacket");
    registry.RegisterTaskType<TestSourceTask>(
        "TestSourceTask",
        {},
        {PortSpec{"out", "TestPacket"}});
    registry.RegisterTaskType<TestSecondSourceTask>(
        "TestSecondSourceTask",
        {},
        {PortSpec{"out", "TestPacket"}});
    registry.RegisterTaskType<TestFanoutSourceTask>(
        "TestFanoutSourceTask",
        {},
        {PortSpec{"left", "TestPacket"}, PortSpec{"right", "TestPacket"}});
    registry.RegisterTaskType<TestSinkTask>(
        "TestSinkTask",
        {PortSpec{"in", "TestPacket"}},
        {});
    registry.RegisterTaskType<TestForwardTask>(
        "TestForwardTask",
        {PortSpec{"in", "TestPacket"}},
        {PortSpec{"out", "TestPacket"}});
    registry.RegisterTaskType<TestAllInputsSinkTask>(
        "TestAllInputsSinkTask",
        {PortSpec{"left", "TestPacket"}, PortSpec{"right", "TestPacket"}},
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
    registry.RegisterTaskFactory("NativeSlamResourceTask", {}, {{"ready", "NativeSlamResourceReady"}}, factory);
    registry.RegisterTaskFactory("NativeSlamClockTask", {}, {{"tick", "NativeSlamTick"}}, factory);
    registry.RegisterTaskFactory(
        "NativeSlamImuGateTask",
        {{"ready", "NativeSlamResourceReady"}, {"tick", "NativeSlamTick"}},
        {{"frame_ready", "NativeSlamFrameReady"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamAcquireTask",
        {{"frame_ready", "NativeSlamFrameReady"}},
        {{"prepared", "NativeSlamPreparedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamTrackingTask",
        {{"prepared", "NativeSlamPreparedFrame"}},
        {{"tracked", "NativeSlamTrackedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamPosePostprocessTask",
        {{"tracked", "NativeSlamTrackedFrame"}},
        {{"published", "NativeSlamPublishedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamPointCloudTask",
        {{"published", "NativeSlamPublishedFrame"}},
        {{"published", "NativeSlamPublishedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamLivePoseTask",
        {{"published", "NativeSlamPublishedFrame"}},
        {{"published", "NativeSlamPublishedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamMavlinkTask",
        {{"published", "NativeSlamPublishedFrame"}},
        {{"published", "NativeSlamPublishedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamUdpTask",
        {{"published", "NativeSlamPublishedFrame"}},
        {{"published", "NativeSlamPublishedFrame"}, {"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "NativeSlamDfxTask",
        {{"published", "NativeSlamPublishedFrame"}},
        {{"status", "NativeSlamStatus"}},
        factory);
    registry.RegisterTaskFactory("NativeSlamMonitorTask", {{"status", "NativeSlamStatus"}}, {}, factory);
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
    registry.RegisterTaskFactory(
        "CalibResourceTask",
        {},
        {{"camera", "CalibResourceReady"}, {"imu", "CalibResourceReady"}, {"done", "CalibCaptureDone"}},
        factory);
    registry.RegisterTaskFactory("NativeCalibClockTask", {}, {{"tick", "NativeCalibTick"}}, factory);
    registry.RegisterTaskFactory(
        "CalibCameraAcquireTask",
        {{"ready", "CalibResourceReady"}, {"tick", "NativeCalibTick"}},
        {{"pace", "CalibStereoFrame"}, {"preview", "CalibStereoFrame"}, {"done", "CalibCaptureDone"}},
        factory);
    registry.RegisterTaskFactory(
        "CalibPacingFilterTask",
        {{"stereo", "CalibStereoFrame"}},
        {{"save", "CalibSavePair"}},
        factory);
    registry.RegisterTaskFactory(
        "CalibStorageWriteTask",
        {{"save", "CalibSavePair"}},
        {{"status", "CalibStorageStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "CalibImuWriterTask",
        {{"ready", "CalibResourceReady"}},
        {{"status", "CalibImuStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "CalibUdpPreviewTask",
        {{"stereo", "CalibStereoFrame"}},
        {{"status", "CalibPreviewStatus"}},
        factory);
    registry.RegisterTaskFactory(
        "CalibCompletionTask",
        {{"capture", "CalibCaptureDone"},
         {"storage", "CalibStorageStatus"},
         {"imu", "CalibImuStatus"},
         {"preview", "CalibPreviewStatus"}},
        {{"flush", "CalibFlushRequest"}},
        factory);
    registry.RegisterTaskFactory(
        "CalibFlushSyncTask",
        {{"flush", "CalibFlushRequest"}},
        {{"status", "NativeCalibStatus"}},
        factory);
    registry.RegisterTaskFactory("NativeCalibMonitorTask", {{"status", "NativeCalibStatus"}}, {}, factory);
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
          "outputs": {"out": "packets"}
        },
        {
          "name": "sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"in": "packets"}
        }
      ]
    })";
}

void ExpectConfigureThrows(const std::string& json) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);
    EXPECT_THROW(graph.ConfigureJson(json), std::runtime_error);
}

void RunTopology(const smartdrone::runtime_graph::RuntimeGraphConfig& config,
                 const std::vector<std::string>& queues,
                 const std::vector<std::string>& tasks) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

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
        smartdrone::runtime_graph::ParseRuntimeGraphConfigJsonFile(
            std::string(TEST_RUNTIME_GRAPH_DIR) + "/" + jsonFile),
        queues,
        tasks);
}

void RunTopologyFromMermaidFile(const std::string& mermaidFile,
                                const std::vector<std::string>& queues,
                                const std::vector<std::string>& tasks) {
    auto registry = MakeRegistry();
    RunTopology(
        smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaidFile(
            std::string(TEST_RUNTIME_GRAPH_DIR) + "/" + mermaidFile,
            registry),
        queues,
        tasks);
}

} // namespace

TEST(RuntimeGraphQueue, DropNewestKeepsOldItemsAndCountsDrops) {
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

TEST(RuntimeGraphQueue, OverwriteOldestKeepsNewestItemsAndCountsOverwrites) {
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

TEST(RuntimeGraphQueue, TryPopLatestDrainsQueueAndReturnsNewestItem) {
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

TEST(RuntimeGraphQueue, NotifierRunsForAcceptedPushesOnly) {
    SpscSharedPtrQueue<TestPacket> queue("packets", "TestPacket", 1, OverflowPolicy::DropNewest);
    int notifications = 0;
    queue.SetNotifier([&notifications]() { ++notifications; });

    EXPECT_TRUE(queue.Push(std::make_shared<TestPacket>(TestPacket{1})));
    EXPECT_FALSE(queue.Push(std::make_shared<TestPacket>(TestPacket{2})));

    EXPECT_EQ(notifications, 1);
    EXPECT_EQ(queue.Diagnostics().wakeups, 1u);
}

TEST(RuntimeGraphIngress, ExternalInterruptEventWakesConsumerTask) {
    Registry registry;
    registry.RegisterMessageType<TestPacket>("TestPacket");
    int packets = 0;
    registry.RegisterTaskFactory(
        "TestCountingSinkTask",
        {PortSpec{"in", "TestPacket"}},
        {},
        [&packets]() {
            return std::unique_ptr<ITask>(new TestCountingSinkTask(packets));
        });

    RuntimeGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [
        {"name": "irq_events", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "interrupt_consumer",
          "type": "TestCountingSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["irq_events"]},
          "inputs": {"in": "irq_events"}
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

TEST(RuntimeGraphIngress, RejectsTypeMismatchDuplicateIngressAndTaskProducerConflict) {
    auto registry = MakeRegistry();

    {
        RuntimeGraph graph(registry);
        graph.ConfigureJson(R"({
          "queues": [
            {"name": "irq_events", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
          ],
          "tasks": [
            {
              "name": "sink",
              "type": "TestSinkTask",
              "trigger": {"mode": "any_queue_ready", "queues": ["irq_events"]},
              "inputs": {"in": "irq_events"}
            }
          ]
        })");

        EXPECT_THROW(graph.CreateExternalIngress<OtherPacket>("irq_events"), std::runtime_error);
        auto ingress = graph.CreateExternalIngress<TestPacket>("irq_events");
        EXPECT_TRUE(ingress.Valid());
        EXPECT_THROW(graph.CreateExternalIngress<TestPacket>("irq_events"), std::runtime_error);
    }

    {
        RuntimeGraph graph(registry);
        graph.ConfigureJson(MinimalValidJson());
        EXPECT_THROW(graph.CreateExternalIngress<TestPacket>("packets"), std::runtime_error);
    }
}

TEST(RuntimeGraph, RunsPipelineConfiguredFromJsonFile) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

    graph.Configure(smartdrone::runtime_graph::ParseRuntimeGraphConfigJsonFile(
        std::string(TEST_RUNTIME_GRAPH_DIR) + "/basic_pipeline.json"));

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

TEST(RuntimeGraph, TaskCanPublishToMultipleQueuesForDifferentConsumers) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

    graph.Configure(smartdrone::runtime_graph::ParseRuntimeGraphConfigJsonFile(
        std::string(TEST_RUNTIME_GRAPH_DIR) + "/fanout_pipeline.json"));

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

TEST(RuntimeGraphTopology, RunsLinearChainTopologyFromJson) {
    RunTopologyFromJsonFile(
        "chain_pipeline.json",
        {"source_to_forward", "forward_to_sink"},
        {"source", "forward", "sink"});
}

TEST(RuntimeGraphTopology, RunsParallelIndependentTopologyFromJson) {
    RunTopologyFromJsonFile(
        "parallel_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(RuntimeGraphTopology, RunsFanInJoinTopologyFromJson) {
    RunTopologyFromJsonFile(
        "fanin_pipeline.json",
        {"left_packets", "right_packets"},
        {"left_source", "right_source", "join_sink"});
}

TEST(RuntimeGraphTopology, RunsDiamondTopologyFromJson) {
    RunTopologyFromJsonFile(
        "diamond_pipeline.json",
        {"left_packets", "right_packets", "forwarded_left_packets"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(RuntimeGraphMermaid, ConvertsMermaidTopologyToRuntimeConfig) {
    auto registry = MakeRegistry();
    const auto config = smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
        forward["type=TestForwardTask; trigger=any_queue_ready"]
        sink["type=TestSinkTask; trigger=any_queue_ready"]

        source -->|"type=TestPacket; depth=8; overflow=drop_newest"| forward
        forward -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
    )", registry);

    ASSERT_EQ(config.queues.size(), 2u);
    EXPECT_EQ(config.queues[0].name, "source_out_to_forward_in");
    EXPECT_EQ(config.queues[0].type, "TestPacket");
    EXPECT_EQ(config.queues[0].depth, 8u);
    EXPECT_EQ(config.queues[1].name, "forward_out_to_sink_in");

    ASSERT_EQ(config.tasks.size(), 3u);
    EXPECT_EQ(config.tasks[0].name, "source");
    EXPECT_EQ(config.tasks[0].outputs.at("out"), "source_out_to_forward_in");
    EXPECT_EQ(config.tasks[1].inputs.at("in"), "source_out_to_forward_in");
    EXPECT_EQ(config.tasks[1].outputs.at("out"), "forward_out_to_sink_in");
    EXPECT_EQ(config.tasks[1].trigger.queues, std::vector<std::string>{"source_out_to_forward_in"});
    EXPECT_EQ(config.tasks[2].inputs.at("in"), "forward_out_to_sink_in");
    EXPECT_EQ(config.tasks[2].trigger.queues, std::vector<std::string>{"forward_out_to_sink_in"});
}

TEST(RuntimeGraphMermaid, RunsTopologyCompiledFromMermaidFile) {
    RunTopologyFromMermaidFile(
        "chain_pipeline.mmd",
        {"source_out_to_forward_in", "forward_out_to_sink_in"},
        {"source", "forward", "sink"});
}

TEST(RuntimeGraphMermaid, ConvertsMarkdownMermaidBlockToRuntimeConfig) {
    auto registry = MakeRegistry();
    const auto config = smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaid(R"(
# Runtime Graph

```mermaid
flowchart LR
  source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
  sink["type=TestSinkTask; trigger=any_queue_ready"]
  source -->|"type=TestPacket; depth=8; overflow=drop_newest"| sink
```
    )", registry);

    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues[0].name, "source_out_to_sink_in");
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks[0].outputs.at("out"), "source_out_to_sink_in");
    EXPECT_EQ(config.tasks[1].inputs.at("in"), "source_out_to_sink_in");
}

TEST(RuntimeGraphMermaid, CompilesNativeSlamSubgraphFromMaintainedTopology) {
    auto registry = MakeNativeSlamShapeRegistry();
    const auto config = smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaidSubgraphFile(
        std::string(TEST_RUNTIME_GRAPH_DIR) + "/../../config/runtime_graph/native_runtime_topology.md",
        "slam_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 12u);
    ASSERT_EQ(config.queues.size(), 11u);

    RuntimeGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string& name) -> const smartdrone::runtime_graph::TaskConfig* {
        for (const auto& task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto* resource = findTask("NativeSlamResourceTask");
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->trigger.mode, smartdrone::runtime_graph::TriggerMode::Periodic);
    EXPECT_EQ(resource->trigger.interval, std::chrono::milliseconds(1));

    const auto* clock = findTask("NativeSlamClockTask");
    ASSERT_NE(clock, nullptr);
    EXPECT_EQ(clock->trigger.mode, smartdrone::runtime_graph::TriggerMode::Periodic);
    EXPECT_EQ(clock->trigger.interval, std::chrono::milliseconds(1));

    const auto* imuGate = findTask("NativeSlamImuGateTask");
    ASSERT_NE(imuGate, nullptr);
    EXPECT_EQ(imuGate->inputs.at("ready"), "NativeSlamResourceTask_ready_to_NativeSlamImuGateTask_ready");
    EXPECT_EQ(imuGate->inputs.at("tick"), "NativeSlamClockTask_tick_to_NativeSlamImuGateTask_tick");
    EXPECT_EQ(imuGate->trigger.queues,
              (std::vector<std::string>{"NativeSlamResourceTask_ready_to_NativeSlamImuGateTask_ready",
                                         "NativeSlamClockTask_tick_to_NativeSlamImuGateTask_tick"}));

    const auto* acquire = findTask("NativeSlamAcquireTask");
    ASSERT_NE(acquire, nullptr);
    EXPECT_EQ(acquire->inputs.at("frame_ready"),
              "NativeSlamImuGateTask_frame_ready_to_NativeSlamAcquireTask_frame_ready");
    EXPECT_EQ(acquire->trigger.queues,
              (std::vector<std::string>{"NativeSlamImuGateTask_frame_ready_to_NativeSlamAcquireTask_frame_ready"}));

    const auto* tracking = findTask("NativeSlamTrackingTask");
    ASSERT_NE(tracking, nullptr);
    EXPECT_EQ(tracking->inputs.at("prepared"),
              "NativeSlamAcquireTask_prepared_to_NativeSlamTrackingTask_prepared");
    EXPECT_EQ(tracking->trigger.queues,
              (std::vector<std::string>{"NativeSlamAcquireTask_prepared_to_NativeSlamTrackingTask_prepared"}));
}

TEST(RuntimeGraphMermaid, CompilesNativeCalibSubgraphFromMaintainedTopology) {
    auto registry = MakeNativeCalibShapeRegistry();
    const auto config = smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaidSubgraphFile(
        std::string(TEST_RUNTIME_GRAPH_DIR) + "/../../config/runtime_graph/native_runtime_topology.md",
        "calib_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 10u);
    ASSERT_EQ(config.queues.size(), 12u);

    RuntimeGraph graph(registry);
    EXPECT_NO_THROW(graph.Configure(config));

    auto findTask = [&config](const std::string& name) -> const smartdrone::runtime_graph::TaskConfig* {
        for (const auto& task : config.tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    };

    const auto* camera = findTask("CalibCameraAcquireTask");
    ASSERT_NE(camera, nullptr);
    EXPECT_EQ(camera->inputs.at("ready"), "CalibResourceTask_camera_to_CalibCameraAcquireTask_ready");
    EXPECT_EQ(camera->inputs.at("tick"), "NativeCalibClockTask_tick_to_CalibCameraAcquireTask_tick");

    const auto* pace = findTask("CalibPacingFilterTask");
    ASSERT_NE(pace, nullptr);
    EXPECT_EQ(pace->inputs.at("stereo"), "CalibCameraAcquireTask_pace_to_CalibPacingFilterTask_stereo");

    const auto* preview = findTask("CalibUdpPreviewTask");
    ASSERT_NE(preview, nullptr);
    EXPECT_EQ(preview->inputs.at("stereo"), "CalibCameraAcquireTask_preview_to_CalibUdpPreviewTask_stereo");
}

TEST(RuntimeGraphMermaidTopology, RunsBasicTopologyFromMermaid) {
    RunTopologyFromMermaidFile(
        "basic_pipeline.mmd",
        {"source_out_to_sink_in"},
        {"source", "sink"});
}

TEST(RuntimeGraphMermaidTopology, RunsFanoutTopologyFromMermaid) {
    RunTopologyFromMermaidFile(
        "fanout_pipeline.mmd",
        {"fanout_source_left_to_left_sink_in", "fanout_source_right_to_right_sink_in"},
        {"fanout_source", "left_sink", "right_sink"});
}

TEST(RuntimeGraphMermaidTopology, RunsLinearChainTopologyFromMermaid) {
    RunTopologyFromMermaidFile(
        "chain_pipeline.mmd",
        {"source_out_to_forward_in", "forward_out_to_sink_in"},
        {"source", "forward", "sink"});
}

TEST(RuntimeGraphMermaidTopology, RunsParallelIndependentTopologyFromMermaid) {
    RunTopologyFromMermaidFile(
        "parallel_pipeline.mmd",
        {"left_source_out_to_left_sink_in", "right_source_out_to_right_sink_in"},
        {"left_source", "left_sink", "right_source", "right_sink"});
}

TEST(RuntimeGraphMermaidTopology, RunsFanInJoinTopologyFromMermaid) {
    RunTopologyFromMermaidFile(
        "fanin_pipeline.mmd",
        {"left_source_out_to_join_sink_left", "right_source_out_to_join_sink_right"},
        {"left_source", "right_source", "join_sink"});
}

TEST(RuntimeGraphMermaidTopology, RunsDiamondTopologyFromMermaid) {
    RunTopologyFromMermaidFile(
        "diamond_pipeline.mmd",
        {"fanout_source_left_to_left_forward_in",
         "fanout_source_right_to_join_sink_right",
         "left_forward_out_to_join_sink_left"},
        {"fanout_source", "left_forward", "join_sink"});
}

TEST(RuntimeGraphMermaid, RejectsInvalidMermaidTopology) {
    EXPECT_THROW(smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaid(R"(
      flowchart LR
        source[type=TestSourceTask; trigger=periodic; interval_ms=1]
        source.out -->|type=TestPacket; depth=8; overflow=drop_newest| missing.in
    )"), std::runtime_error);

    EXPECT_THROW(smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaid(R"(
      flowchart LR
        source[type=TestSourceTask; trigger=periodic; interval_ms=1]
        sink[type=TestSinkTask; trigger=any_queue_ready]
        source.out -->|type=TestPacket; overflow=drop_newest| sink.in
    )"), std::runtime_error);

    EXPECT_THROW(smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaid(R"(
      flowchart LR
        source["type=TestSourceTask; trigger=periodic; interval_ms=1"]
        sink["type=TestSinkTask; trigger=any_queue_ready; trigger_queues=OtherPacket"]
        source -->|"type=TestPacket; depth=1; overflow=drop_newest"| sink
    )", MakeRegistry()), std::runtime_error);
}

TEST(RuntimeGraph, RejectsMultipleConsumersForSpscQueue) {
    const char* json = R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"out": "packets"}
        },
        {
          "name": "sink_a",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"in": "packets"}
        },
        {
          "name": "sink_b",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"in": "packets"}
        }
      ]
    })";

    ExpectConfigureThrows(json);
}

TEST(RuntimeGraph, AllQueueReadyWaitsForAllInputs) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

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
          "outputs": {"out": "left_packets"}
        },
        {
          "name": "right_source",
          "type": "TestSecondSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 2},
          "outputs": {"out": "right_packets"}
        },
        {
          "name": "all_sink",
          "type": "TestAllInputsSinkTask",
          "trigger": {"mode": "all_queue_ready", "queues": ["left_packets", "right_packets"]},
          "inputs": {"left": "left_packets", "right": "right_packets"}
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

TEST(RuntimeGraph, PeriodicOrAnyQueueReadyRunsWithQueueTrigger) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 8, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"out": "packets"}
        },
        {
          "name": "hybrid_sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "periodic_or_any_queue_ready", "interval_ms": 50, "queues": ["packets"]},
          "inputs": {"in": "packets"}
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

TEST(RuntimeGraph, PeriodicTaskCanRunWithoutQueues) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

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

TEST(RuntimeGraph, SupportsCapturedTaskFactoryForNativeAdapters) {
    int ticks = 0;
    Registry registry;
    registry.RegisterTaskFactory(
        "TestCountingTask",
        {},
        {},
        [&ticks]() {
            return std::unique_ptr<ITask>(new TestCountingTask(ticks));
        });

    RuntimeGraph graph(registry);
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

TEST(RuntimeGraph, TaskExceptionsAreCountedAndRunnerKeepsAlive) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

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

TEST(RuntimeGraph, ContextSlotErrorsAreCountedAsTaskErrors) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

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

TEST(RuntimeGraph, LifecycleStateAndAccessorsBehaveAsExpected) {
    auto registry = MakeRegistry();
    RuntimeGraph graph(registry);

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

TEST(RuntimeGraphConfig, ParsesEscapesAndRejectsMissingFile) {
    const auto config = smartdrone::runtime_graph::ParseRuntimeGraphConfigJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "tail_drop"}
      ],
      "tasks": [
        {
          "name": "source\nname",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"out": "packets"}
        }
      ]
    })");

    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.tasks.front().name, "source\nname");
    EXPECT_THROW(
        smartdrone::runtime_graph::ParseRuntimeGraphConfigJsonFile("/tmp/smart_drone_missing_runtime_graph.json"),
        std::runtime_error);
}

TEST(RuntimeGraphConfig, RejectsInvalidJsonAndUnsupportedEnumValues) {
    EXPECT_THROW(smartdrone::runtime_graph::ParseRuntimeGraphConfigJson("{"), std::runtime_error);
    EXPECT_THROW(smartdrone::runtime_graph::ParseRuntimeGraphConfigJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "bad_policy"}
      ],
      "tasks": []
    })"), std::runtime_error);
    EXPECT_THROW(smartdrone::runtime_graph::ParseRuntimeGraphConfigJson(R"({
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

TEST(RuntimeGraphReflection, RegistersMessagesAndTaskPortsFromCatalog) {
    Registry registry;
    smartdrone::runtime_graph::RuntimeGraphTypeCatalog::Global().RegisterReflectedMessageTypes(registry);
    smartdrone::runtime_graph::RuntimeGraphTypeCatalog::Global().RegisterReflectedTaskTypes(
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

    RuntimeGraph graph(registry);
    graph.ConfigureJson(R"({
      "queues": [
        {"name": "reflected_packets", "type": "ReflectedPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "ReflectedSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "outputs": {"out": "reflected_packets"}
        },
        {
          "name": "sink",
          "type": "ReflectedSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["reflected_packets"]},
          "inputs": {"in": "reflected_packets"}
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

TEST(RuntimeGraphValidation, RejectsInvalidTopologyConfigurations) {
    const std::vector<std::string> invalidJsons = {
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"},{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[{"name":"packets","type":"MissingPacket","depth":4,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":0,"overflow":"drop_newest"}],"tasks":[]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1}},{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1}}]})",
        R"({"queues":[],"tasks":[{"name":"missing","type":"MissingTask","trigger":{"mode":"periodic","interval_ms":1}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":0}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":[]},"inputs":{"in":"packets"}}]})",
        R"({"queues":[],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["missing"]},"inputs":{"in":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs":{"bad":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs":{"bad":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs":{"in":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs":{"out":"missing"}}]})",
        R"({"queues":[{"name":"packets","type":"OtherPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["packets"]},"inputs":{"in":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"OtherPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs":{"out":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"source_a","type":"TestSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs":{"out":"packets"}},{"name":"source_b","type":"TestSecondSourceTask","trigger":{"mode":"periodic","interval_ms":1},"outputs":{"out":"packets"}}]})",
        R"({"queues":[{"name":"packets","type":"TestPacket","depth":4,"overflow":"drop_newest"},{"name":"other","type":"TestPacket","depth":4,"overflow":"drop_newest"}],"tasks":[{"name":"sink","type":"TestSinkTask","trigger":{"mode":"any_queue_ready","queues":["other"]},"inputs":{"in":"packets"}}]})"
    };

    for (const auto& json : invalidJsons) {
        ExpectConfigureThrows(json);
    }
}
