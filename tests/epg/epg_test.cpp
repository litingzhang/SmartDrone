#include "common/epg/epg.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "core/application/epg/epg_runtime_optimizer.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"

namespace {

using epg::ITask;
using epg::OverflowPolicy;
using epg::PortSpec;
using epg::Registry;
using epg::EventPipelineGraph;
using epg::SpscSharedPtrQueue;
using epg::TaskContext;
using smartdrone::core::application::BuildEpgTaskTopologyVersion;
using smartdrone::core::application::EpgDomain;
using smartdrone::core::application::EpgTaskCatalogTypes;
using smartdrone::core::application::EpgManifestForDomain;

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

struct SlamResourceReady {};
struct SlamTick {};
struct SlamImuReady {};
struct SlamFrameReady {};
struct SlamPreparedFrame {};
struct SlamTrackedFrame {};
struct SlamPublishedFrame {};
struct SlamStatus {};
struct CalibResourceReady {};
struct CalibTick {};
struct CalibStereoFrame {};
struct CalibSavePair {};
struct CalibCaptureDone {};
struct CalibStopRequest {};
struct CalibStorageStatus {};
struct CalibImuStatus {};
struct CalibPreviewStatus {};
struct CalibFlushRequest {};
struct CalibStatus {};

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

class TestSlowTask final : public ITask {
public:
    void OnTick(TaskContext& context) override {
        (void)context;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
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
    registry.RegisterTaskType<TestSlowTask>(
        "TestSlowTask",
        {},
        {});
    return registry;
}

Registry MakeSlamShapeRegistry() {
    Registry registry;
    registry.RegisterMessageType<SlamResourceReady>("SlamResourceReady");
    registry.RegisterMessageType<SlamTick>("SlamTick");
    registry.RegisterMessageType<SlamImuReady>("SlamImuReady");
    registry.RegisterMessageType<SlamFrameReady>("SlamFrameReady");
    registry.RegisterMessageType<SlamPreparedFrame>("SlamPreparedFrame");
    registry.RegisterMessageType<SlamTrackedFrame>("SlamTrackedFrame");
    registry.RegisterMessageType<SlamPublishedFrame>("SlamPublishedFrame");
    registry.RegisterMessageType<SlamStatus>("SlamStatus");

    const auto factory = []() {
        return std::unique_ptr<ITask>(new TestHeartbeatTask());
    };
    registry.RegisterTaskFactory("SlamResourceTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamClockTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamImuPollTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamBackendTickTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamImuGateTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamAcquireTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamTrackingTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamPosePostprocessTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamPointCloudTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamLivePoseTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamMavlinkTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamUdpTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamDfxTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamMonitorTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgDfxSnapshotTask", {}, {}, factory);
    return registry;
}

Registry MakeSystemShapeRegistry() {
    Registry registry;
    const auto factory = []() {
        return std::unique_ptr<ITask>(new TestHeartbeatTask());
    };
    registry.RegisterTaskFactory("VehicleTelemetryRxTask", {}, {}, factory);
    registry.RegisterTaskFactory("SetpointStreamTask", {}, {}, factory);
    registry.RegisterTaskFactory("UdpCommandTask", {}, {}, factory);
    registry.RegisterTaskFactory("ManualControlTask", {}, {}, factory);
    registry.RegisterTaskFactory("ForceRestartTask", {}, {}, factory);
    registry.RegisterTaskFactory("RuntimeSupervisorTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgRedeployTask", {}, {}, factory);
    registry.RegisterTaskFactory("DiscoveryBeaconTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgDfxSnapshotTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgOptimizeTask", {}, {}, factory);
    return registry;
}

Registry MakeCalibShapeRegistry() {
    Registry registry;
    registry.RegisterMessageType<CalibResourceReady>("CalibResourceReady");
    registry.RegisterMessageType<CalibTick>("CalibTick");
    registry.RegisterMessageType<CalibStereoFrame>("CalibStereoFrame");
    registry.RegisterMessageType<CalibSavePair>("CalibSavePair");
    registry.RegisterMessageType<CalibCaptureDone>("CalibCaptureDone");
    registry.RegisterMessageType<CalibStopRequest>("CalibStopRequest");
    registry.RegisterMessageType<CalibStorageStatus>("CalibStorageStatus");
    registry.RegisterMessageType<CalibImuStatus>("CalibImuStatus");
    registry.RegisterMessageType<CalibPreviewStatus>("CalibPreviewStatus");
    registry.RegisterMessageType<CalibFlushRequest>("CalibFlushRequest");
    registry.RegisterMessageType<CalibStatus>("CalibStatus");

    const auto factory = []() {
        return std::unique_ptr<ITask>(new TestHeartbeatTask());
    };
    registry.RegisterTaskFactory("CalibResourceTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibClockTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibCameraAcquireTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibPacingFilterTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibStorageWriteTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibImuWriterTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibUdpPreviewTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibCompletionTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibFlushSyncTask", {}, {}, factory);
    registry.RegisterTaskFactory("CalibMonitorTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgDfxSnapshotTask", {}, {}, factory);
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

std::string RuntimeTopologyPath()
{
    return std::string(TEST_EPG_DIR) + "/../../" +
           EpgManifestForDomain(EpgDomain::SystemRuntime).topologyPath;
}

smartdrone::core::application::EpgTaskManifest MakeValidTestManifest()
{
    smartdrone::core::application::EpgTaskManifest manifest;
    manifest.subgraphName = "test_graph";
    manifest.topologyPath = "config/epg/test_topology.dot";
    manifest.topologyVersion = "config/epg/test_topology.dot:v1";
    manifest.artifactPaths.dfxSnapshotPath = "/tmp/test_epg.json";
    manifest.artifactPaths.profilePath = "/tmp/test_epg_profile.json";
    manifest.artifactPaths.optimizedConfigPath =
        "output/epg/optimized_test_graph.json";
    manifest.artifactPaths.solverReportPath =
        "output/epg/optimized_test_graph_report.json";
    manifest.catalog.push_back(
        {"TestSourceTask", "source", "cpu", 1000, 2000, false});
    return manifest;
}

Registry::TaskFactory TestSourceFactoryResolver(const std::string &taskType)
{
    if (taskType == "TestSourceTask" || taskType == "MissingTask") {
        return Registry::TaskFactory([]() {
            return std::unique_ptr<ITask>(new TestSourceTask());
        });
    }
    return Registry::TaskFactory{};
}

std::string ReadFileText(const std::string& path)
{
    std::ifstream input(path);
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

std::string MinimalQueueDiagnosticsJson()
{
    return R"({
            "maxDepthObserved": 0,
            "droppedNewest": 0,
            "overwrittenOldest": 0,
            "pushedPerSecond": 0,
            "poppedPerSecond": 0,
            "droppedPerSecond": 0
          })";
}

std::string MinimalTaskDiagnosticsJson()
{
    return R"({
            "maxLoopUs": 0,
            "averageLoopUs": 0,
            "p90LoopUs": 0,
            "p99LoopUs": 0,
            "utilizationPpm": 0,
            "budgetOverrunCount": 0,
            "deadlineMissCount": 0,
            "schedulingErrorCount": 0
          })";
}

std::string MinimalProfileDiagnosticsJson(std::uint64_t timestampMs)
{
    return std::string(R"({
            "graph": "test_graph",
            "timestampMs": )") + std::to_string(timestampMs) +
           R"(,
            "queues": {"packets": )" + MinimalQueueDiagnosticsJson() +
           R"(},
            "tasks": {"source": )" + MinimalTaskDiagnosticsJson() +
           R"(}
          })";
}

std::string MissingTaskProfileDiagnosticsJson(std::uint64_t timestampMs)
{
    return std::string(R"({
            "graph": "test_graph",
            "timestampMs": )") + std::to_string(timestampMs) +
           R"(,
            "queues": {"packets": )" + MinimalQueueDiagnosticsJson() +
           R"(},
            "tasks": {}
          })";
}

std::string MissingTaskDiagnosticsProfileJson()
{
    return std::string(R"({
      "schema": "smartdrone.epg.profile.v1",
      "graph": "test_graph",
      "topologyVersion": "test-topology",
      "timestampMs": 1000,
      "taskCatalog": [
        {
          "taskType": "TestSourceTask",
          "role": "source",
          "resource": "cpu",
          "budgetUs": 1000,
          "deadlineUs": 2000,
          "replaceable": false
        }
      ],
      "topology": {
        "queues": [
          {
            "name": "packets",
            "type": "TestPacket",
            "depth": 4,
            "overflow": "drop_newest"
          }
        ],
        "tasks": [
          {
            "name": "source",
            "type": "TestSourceTask",
            "trigger": {"mode": "periodic", "interval_ms": 1},
            "outputs": {"0": "packets"}
          }
        ]
      },
      "diagnostics": )") + MissingTaskProfileDiagnosticsJson(1000) +
           R"(
    })";
}

std::string NonReplaceableTaskProfileJson()
{
    return std::string(R"({
      "schema": "smartdrone.epg.profile.v1",
      "graph": "test_graph",
      "topologyVersion": "test-topology",
      "timestampMs": 1000,
      "taskCatalog": [
        {
          "taskType": "TestSourceTask",
          "role": "source",
          "resource": "cpu",
          "budgetUs": 1000,
          "deadlineUs": 2000,
          "replaceable": false
        }
      ],
      "topology": {
        "queues": [
          {
            "name": "packets",
            "type": "TestPacket",
            "depth": 4,
            "overflow": "drop_newest"
          }
        ],
        "tasks": [
          {
            "name": "source",
            "type": "TestSourceTask",
            "trigger": {"mode": "periodic", "interval_ms": 1},
            "outputs": {"0": "packets"}
          }
        ]
      },
      "diagnostics": {
        "queues": {"packets": )") + MinimalQueueDiagnosticsJson() +
           R"(},
        "tasks": {
          "source": {
            "maxLoopUs": 2600,
            "p90LoopUs": 2400,
            "p99LoopUs": 2600,
            "averageLoopUs": 2200,
            "utilizationPpm": 900000,
            "budgetOverrunCount": 2,
            "deadlineMissCount": 1,
            "schedulingErrorCount": 0
          }
        }
      }
    })";
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
        source["type=TestSourceTask; trigger=periodic; interval_ms=1; resource=cpu; cpu_affinity=-1; budget_us=500; deadline_us=900; backpressure_outputs=0; realtime=true; priority=42"]
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
    EXPECT_EQ(config.tasks[0].scheduling.resource, "cpu");
    EXPECT_EQ(config.tasks[0].scheduling.cpuAffinity, -1);
    EXPECT_EQ(config.tasks[0].scheduling.budgetUs, 500u);
    EXPECT_EQ(config.tasks[0].scheduling.deadlineUs, 900u);
    EXPECT_EQ(config.tasks[0].scheduling.backpressureOutputs,
              std::vector<epg::PortId>{0});
    EXPECT_TRUE(config.tasks[0].scheduling.realtime);
    EXPECT_EQ(config.tasks[0].scheduling.priority, 42);
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

TEST(EventPipelineGraphDot, CompilesSlamSubgraphFromMaintainedTopology) {
    auto registry = MakeSlamShapeRegistry();
    const auto config = epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_slam_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 15u);
    ASSERT_EQ(config.queues.size(), 23u);

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

    const auto* resource = findTask("SlamResourceTask");
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->trigger.mode, epg::TriggerMode::Periodic);
    EXPECT_EQ(resource->trigger.interval, std::chrono::milliseconds(100));

    const auto* clock = findTask("SlamClockTask");
    ASSERT_NE(clock, nullptr);
    EXPECT_EQ(clock->trigger.mode, epg::TriggerMode::Periodic);
    EXPECT_EQ(clock->trigger.interval, std::chrono::milliseconds(50));

    const auto* imuPoll = findTask("SlamImuPollTask");
    ASSERT_NE(imuPoll, nullptr);
    EXPECT_EQ(imuPoll->inputs.at(0), "SlamResourceTask_0_to_SlamImuPollTask_0");
    EXPECT_EQ(imuPoll->trigger.mode, epg::TriggerMode::PeriodicOrAnyQueueReady);
    EXPECT_EQ(imuPoll->trigger.interval, std::chrono::milliseconds(1));
    EXPECT_FALSE(imuPoll->scheduling.realtime);
    EXPECT_EQ(imuPoll->scheduling.priority, 0);
    EXPECT_EQ(imuPoll->trigger.queues,
              (std::vector<std::string>{"SlamResourceTask_0_to_SlamImuPollTask_0"}));

    const auto* backendTick = findTask("SlamBackendTickTask");
    ASSERT_NE(backendTick, nullptr);
    EXPECT_TRUE(backendTick->inputs.empty());
    EXPECT_EQ(backendTick->trigger.mode, epg::TriggerMode::Periodic);
    EXPECT_EQ(backendTick->trigger.interval, std::chrono::milliseconds(5));
    EXPECT_TRUE(backendTick->trigger.queues.empty());

    const auto* imuGate = findTask("SlamImuGateTask");
    ASSERT_NE(imuGate, nullptr);
    EXPECT_EQ(imuGate->inputs.at(0), "SlamImuPollTask_0_to_SlamImuGateTask_0");
    EXPECT_EQ(imuGate->inputs.at(1), "SlamClockTask_0_to_SlamImuGateTask_1");
    EXPECT_EQ(imuGate->trigger.queues,
              (std::vector<std::string>{"SlamImuPollTask_0_to_SlamImuGateTask_0",
                                         "SlamClockTask_0_to_SlamImuGateTask_1"}));

    const auto* acquire = findTask("SlamAcquireTask");
    ASSERT_NE(acquire, nullptr);
    EXPECT_EQ(acquire->inputs.at(0),
              "SlamImuGateTask_0_to_SlamAcquireTask_0");
    EXPECT_EQ(acquire->trigger.queues,
              (std::vector<std::string>{"SlamImuGateTask_0_to_SlamAcquireTask_0"}));

    const auto* tracking = findTask("SlamTrackingTask");
    ASSERT_NE(tracking, nullptr);
    EXPECT_EQ(tracking->inputs.at(0),
              "SlamAcquireTask_0_to_SlamTrackingTask_0");
    EXPECT_EQ(tracking->trigger.queues,
              (std::vector<std::string>{"SlamAcquireTask_0_to_SlamTrackingTask_0"}));

    const auto* dfxSnapshot = findTask("SlamGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));

    const auto* monitor = findTask("SlamMonitorTask");
    ASSERT_NE(monitor, nullptr);
    EXPECT_EQ(monitor->inputs.at(8), "SlamUdpTask_1_to_SlamMonitorTask_8");
    EXPECT_NE(std::find(monitor->trigger.queues.begin(),
                        monitor->trigger.queues.end(),
                        "SlamUdpTask_1_to_SlamMonitorTask_8"),
              monitor->trigger.queues.end());
}

TEST(EventPipelineGraphDot, CompilesSystemRuntimeSubgraphFromMaintainedTopology) {
    auto registry = MakeSystemShapeRegistry();
    const auto config = epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_system_runtime_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 10u);
    ASSERT_EQ(config.queues.size(), 0u);

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

    const auto* vehicleTelemetryRx = findTask("VehicleTelemetryRxTask");
    ASSERT_NE(vehicleTelemetryRx, nullptr);
    EXPECT_EQ(vehicleTelemetryRx->trigger.interval, std::chrono::milliseconds(2));

    const auto* setpointStream = findTask("SetpointStreamTask");
    ASSERT_NE(setpointStream, nullptr);
    EXPECT_EQ(setpointStream->trigger.interval, std::chrono::milliseconds(5));

    const auto* manualControl = findTask("ManualControlTask");
    ASSERT_NE(manualControl, nullptr);
    EXPECT_EQ(manualControl->trigger.interval, std::chrono::milliseconds(50));

    const auto* forceRestart = findTask("ForceRestartTask");
    ASSERT_NE(forceRestart, nullptr);
    EXPECT_EQ(forceRestart->trigger.interval, std::chrono::milliseconds(50));

    const auto* supervisor = findTask("RuntimeSupervisorTask");
    ASSERT_NE(supervisor, nullptr);
    EXPECT_EQ(supervisor->trigger.interval, std::chrono::milliseconds(100));

    const auto* redeploy = findTask("EpgRedeployTask");
    ASSERT_NE(redeploy, nullptr);
    EXPECT_EQ(redeploy->type, "EpgRedeployTask");
    EXPECT_EQ(redeploy->trigger.interval, std::chrono::milliseconds(500));

    const auto* dfxSnapshot = findTask("EpgDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));

    const auto* optimizer = findTask("EpgOptimizeTask");
    ASSERT_NE(optimizer, nullptr);
    EXPECT_EQ(optimizer->type, "EpgOptimizeTask");
    EXPECT_EQ(optimizer->trigger.interval, std::chrono::milliseconds(5000));
}

TEST(EventPipelineGraphDot, CompilesCalibSubgraphFromMaintainedTopology) {
    auto registry = MakeCalibShapeRegistry();
    const auto config = epg::ParseGraphConfigDotFile(
        RuntimeTopologyPath(),
        "cluster_calib_session_graph",
        registry);

    ASSERT_EQ(config.tasks.size(), 11u);
    ASSERT_EQ(config.queues.size(), 13u);

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
    EXPECT_EQ(camera->inputs.at(1), "CalibClockTask_0_to_CalibCameraAcquireTask_1");

    const auto* pace = findTask("CalibPacingFilterTask");
    ASSERT_NE(pace, nullptr);
    EXPECT_EQ(pace->inputs.at(0), "CalibCameraAcquireTask_0_to_CalibPacingFilterTask_0");

    const auto* preview = findTask("CalibUdpPreviewTask");
    ASSERT_NE(preview, nullptr);
    EXPECT_EQ(preview->inputs.at(0), "CalibCameraAcquireTask_1_to_CalibUdpPreviewTask_0");

    const auto* completion = findTask("CalibCompletionTask");
    ASSERT_NE(completion, nullptr);
    EXPECT_EQ(completion->inputs.at(4), "CalibResourceTask_2_to_CalibCompletionTask_4");
    EXPECT_NE(std::find(completion->trigger.queues.begin(), completion->trigger.queues.end(),
                        "CalibResourceTask_2_to_CalibCompletionTask_4"),
              completion->trigger.queues.end());

    const auto* dfxSnapshot = findTask("CalibGraphDfxSnapshotTask");
    ASSERT_NE(dfxSnapshot, nullptr);
    EXPECT_EQ(dfxSnapshot->type, "EpgDfxSnapshotTask");
    EXPECT_EQ(dfxSnapshot->trigger.interval, std::chrono::milliseconds(500));
}

TEST(EventPipelineGraphLifecycle, ResetForStartClearsStopRequest) {
    std::atomic<bool> stop{true};
    smartdrone::core::application::EpgGraphLifecycle lifecycle({
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

TEST(EventPipelineGraphRedeploy, TracksSystemAndSessionRequestsIndependently) {
    smartdrone::core::application::EpgRedeployCoordinator redeploy;

    redeploy.RequestSystemRedeploy({
        "cluster_system_runtime_graph",
        "optimized config changed",
    });
    EXPECT_TRUE(redeploy.SystemRedeployRequested());
    EXPECT_FALSE(redeploy.SessionRedeployRequested());
    smartdrone::core::application::EpgRedeployRequest systemRequest;
    EXPECT_TRUE(redeploy.TakeSystemRedeployRequest(systemRequest));
    EXPECT_EQ(systemRequest.graphName, "cluster_system_runtime_graph");
    EXPECT_EQ(systemRequest.reason, "optimized config changed");
    EXPECT_FALSE(redeploy.TakeSystemRedeployRequest(systemRequest));

    redeploy.RequestSessionRedeploy({
        "cluster_slam_session_graph",
        "optimized config changed",
    });
    EXPECT_TRUE(redeploy.SessionRedeployRequested());
    EXPECT_FALSE(redeploy.SystemRedeployRequested());
    smartdrone::core::application::EpgRedeployRequest sessionRequest;
    EXPECT_TRUE(redeploy.TakeSessionRedeployRequest(sessionRequest));
    EXPECT_EQ(sessionRequest.graphName, "cluster_slam_session_graph");
    EXPECT_EQ(sessionRequest.reason, "optimized config changed");
    EXPECT_FALSE(redeploy.TakeSessionRedeployRequest(sessionRequest));
}

TEST(EventPipelineGraphRedeploy, WaitsForSystemRedeploySignal) {
    smartdrone::core::application::EpgRedeployCoordinator redeploy;
    std::atomic<bool> requested{false};
    std::thread worker([&redeploy, &requested]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        requested.store(true);
        redeploy.RequestSystemRedeploy({
            "cluster_system_runtime_graph",
            "optimized config changed",
        });
    });

    EXPECT_TRUE(redeploy.WaitForSystemRedeploy(std::chrono::milliseconds(200)));
    EXPECT_TRUE(requested.load());
    smartdrone::core::application::EpgRedeployRequest request;
    EXPECT_TRUE(redeploy.TakeSystemRedeployRequest(request));
    EXPECT_EQ(request.graphName, "cluster_system_runtime_graph");
    EXPECT_EQ(request.reason, "optimized config changed");
    worker.join();
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

TEST(EventPipelineGraph, PeriodicOrAnyQueueReadyRunsWithoutQueuedItems) {
    auto registry = MakeRegistry();
    int ticks = 0;
    registry.RegisterTaskFactory(
        "CountingHybridTask", {PortSpec{0, "TestPacket"}}, {},
        [&ticks]() { return std::unique_ptr<ITask>(new TestCountingTask(ticks)); });
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 1, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "hybrid",
          "type": "CountingHybridTask",
          "trigger": {"mode": "periodic_or_any_queue_ready", "interval_ms": 5, "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    graph.Stop();

    EXPECT_GT(ticks, 0);
    EXPECT_EQ(graph.TaskDiagnostics().at("hybrid").errorCount, 0u);
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

TEST(EventPipelineGraph, TaskBudgetAndDeadlineViolationsAreCounted) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(R"({
      "queues": [],
      "tasks": [
        {
          "name": "slow",
          "type": "TestSlowTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "scheduling": {"budget_us": 1, "deadline_us": 1}
        }
      ]
    })");

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    graph.Stop();

    const auto diag = graph.TaskDiagnostics().at("slow");
    EXPECT_GT(diag.loopCount, 0u);
    EXPECT_GT(diag.budgetOverrunCount, 0u);
    EXPECT_GT(diag.deadlineMissCount, 0u);
    EXPECT_GT(diag.p50LoopUs, 0u);
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
    graph.RequestStop();
    EXPECT_TRUE(graph.Running());
    for (int i = 0; i < 50 && graph.Running(); ++i) {
        if (graph.JoinStopped()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(graph.JoinStopped());
    EXPECT_FALSE(graph.Running());

    graph.Start();
    EXPECT_TRUE(graph.Running());
    graph.Stop();
    EXPECT_FALSE(graph.Running());

    graph.Stop();
    EXPECT_FALSE(graph.Running());
}

TEST(EventPipelineGraphManifest, BuildsCentralizedArtifactPaths) {
    using smartdrone::core::application::BuildEpgTaskArtifactPaths;

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

TEST(EventPipelineGraphManifest, RuntimeManifestsUseCentralizedArtifacts) {
    const auto &system = EpgManifestForDomain(EpgDomain::SystemRuntime);
    const auto systemTaskTypes = EpgTaskCatalogTypes(system);
    EXPECT_EQ(systemTaskTypes.size(), 10u);
    EXPECT_EQ(systemTaskTypes.front(), "VehicleTelemetryRxTask");
    EXPECT_NE(std::find(systemTaskTypes.begin(), systemTaskTypes.end(),
                        "EpgRedeployTask"),
              systemTaskTypes.end());
    EXPECT_EQ(system.topologyPath, "config/epg/epg_topology.dot");
    EXPECT_EQ(
        system.topologyVersion,
        BuildEpgTaskTopologyVersion({system.topologyPath, "v2"}));
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
    EXPECT_EQ(slamTaskTypes.size(), 15u);
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

TEST(EventPipelineGraphManifest, ValidatesRuntimeTuningManifest) {
    auto manifest = MakeValidTestManifest();
    manifest.runtimeTuning.push_back({"source", true, false, false});
    epg::GraphConfig config;
    config.tasks.push_back({"source", "TestSourceTask"});

    EXPECT_NO_THROW(
        smartdrone::core::application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"source", true, false, false}}));
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"source", false, true, false}}),
        std::runtime_error);
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"missing", true, false, false}}),
        std::runtime_error);

    auto duplicate = manifest;
    duplicate.runtimeTuning.push_back({"source", false, true, false});
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskManifest(duplicate),
        std::runtime_error);

    auto incomplete = manifest;
    incomplete.runtimeTuning.push_back({"", false, false, false});
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskManifest(incomplete),
        std::runtime_error);

    auto missingGraphTask = manifest;
    missingGraphTask.runtimeTuning.push_back({"missing", true, false, false});
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskGraphManifest(
            missingGraphTask, config),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsDuplicateCatalogTaskTypes) {
    auto manifest = MakeValidTestManifest();
    manifest.catalog.push_back(
        {"TestSourceTask", "duplicate", "cpu", 1000, 2000, false});

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsAliasTargetOutsideCatalog) {
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"LegacySourceTask", "MissingTask"});

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsDuplicateAliases) {
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"LegacySourceTask", "TestSourceTask"});
    manifest.aliases.push_back({"LegacySourceTask", "TestSourceTask"});

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsAliasShadowingCatalogType) {
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"TestSourceTask", "TestSourceTask"});

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteAliasMetadata) {
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"", "TestSourceTask"});

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsEmptyCatalog) {
    auto manifest = MakeValidTestManifest();
    manifest.catalog.clear();

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteCatalogMetadata) {
    auto manifest = MakeValidTestManifest();
    manifest.catalog[0].role.clear();

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsInvalidCatalogTiming) {
    auto manifest = MakeValidTestManifest();
    manifest.catalog[0].budgetUs = 2000;
    manifest.catalog[0].deadlineUs = 1000;

    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteManifestMetadata) {
    {
        auto manifest = MakeValidTestManifest();
        manifest.subgraphName.clear();

        EXPECT_THROW(
            smartdrone::core::application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
    {
        auto manifest = MakeValidTestManifest();
        manifest.topologyVersion.clear();

        EXPECT_THROW(
            smartdrone::core::application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
    {
        auto manifest = MakeValidTestManifest();
        manifest.artifactPaths.profilePath.clear();

        EXPECT_THROW(
            smartdrone::core::application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
}

TEST(EventPipelineGraphManifest, RejectsOptimizedGraphMismatch) {
    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    const auto optimized = epg::ParseOptimizedGraphJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 1},
          "scheduling": {
            "resource": "cpu",
            "budget_us": 1000,
            "deadline_us": 2000
          },
          "outputs": {"0": "packets"}
        }
      ]
    })");
    EXPECT_NO_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, optimized));
    const auto report = epg::ParseSolverReportJson(R"({
      "schema": "smartdrone.epg.solver_report.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "solverVersion": "unit-test",
      "objective": {
        "name": "unit",
        "score": {
          "queuePressure": 0,
          "periodicOverloadUs": 0,
          "schedulingErrors": 0,
          "budgetOverruns": 0,
          "deadlineMisses": 0,
          "utilizationOverPpm": 0,
          "totalPenalty": 0
        }
      },
      "constraints": {
        "maxQueueDepth": 16,
        "maxPeriodicIntervalMs": 1000,
        "targetUtilizationPpm": 800000
      },
      "decisions": [
        {"kind": "task", "name": "source", "reason": "keep"}
      ]
    })");
    EXPECT_NO_THROW(
        smartdrone::core::application::ValidateEpgSolverReportManifest(
            manifest, optimized.metadata, report.metadata));

    auto wrongTarget = optimized;
    wrongTarget.metadata.targetGraph = "other_graph";
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongTarget),
        std::runtime_error);

    auto wrongTopology = optimized;
    wrongTopology.metadata.topologyVersion = "other-topology";
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongTopology),
        std::runtime_error);

    auto wrongSource = optimized;
    wrongSource.metadata.sourceProfile = "other_graph";
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongSource),
        std::runtime_error);

    auto wrongGeneration = optimized;
    wrongGeneration.metadata.generatedAtMs = 1;
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongGeneration),
        std::runtime_error);

    auto wrongTask = optimized;
    wrongTask.config.tasks[0].type = "TestSinkTask";
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongTask),
        std::runtime_error);

    auto wrongScheduling = optimized;
    wrongScheduling.config.tasks[0].scheduling.budgetUs = 999;
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongScheduling),
        std::runtime_error);

    auto wrongReportTime = report.metadata;
    wrongReportTime.generatedAtMs = 999;
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgSolverReportManifest(
            manifest, optimized.metadata, wrongReportTime),
        std::runtime_error);

    auto wrongReportVersion = report.metadata;
    wrongReportVersion.solverVersion = "other-solver";
    EXPECT_THROW(
        smartdrone::core::application::ValidateEpgSolverReportManifest(
            manifest, optimized.metadata, wrongReportVersion),
        std::runtime_error);
}

TEST(EventPipelineGraph, ProfileJsonIncludesTopologyAndDiagnostics) {
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.ConfigureJson(MinimalValidJson());
    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    graph.Stop();

    const std::string profile = graph.ProfileJson(
        "test_graph", 123, "test-topology",
        R"([{"taskType":"TestSourceTask","role":"source","resource":"cpu","budgetUs":1000,"deadlineUs":2000,"replaceable":false}])");
    EXPECT_NE(profile.find(std::string("\"schema\": \"") +
                               epg::GRAPH_PROFILE_SCHEMA + "\""),
              std::string::npos);
    EXPECT_NE(profile.find("\"graph\": \"test_graph\""), std::string::npos);
    EXPECT_NE(profile.find("\"topologyVersion\": \"test-topology\""),
              std::string::npos);
    EXPECT_NE(profile.find("\"taskCatalog\""), std::string::npos);
    EXPECT_NE(profile.find("\"timestampMs\": 123"), std::string::npos);
    EXPECT_NE(profile.find("\"topology\""), std::string::npos);
    EXPECT_NE(profile.find("\"queues\""), std::string::npos);
    EXPECT_NE(profile.find("\"tasks\""), std::string::npos);
    EXPECT_NE(profile.find("\"name\": \"packets\""), std::string::npos);
    EXPECT_NE(profile.find("\"type\": \"TestPacket\""), std::string::npos);
    EXPECT_NE(profile.find("\"overflow\": \"drop_newest\""), std::string::npos);
    EXPECT_NE(profile.find("\"name\": \"source\""), std::string::npos);
    EXPECT_NE(profile.find("\"mode\": \"periodic\""), std::string::npos);
    EXPECT_NE(profile.find("\"interval_ms\": 1"), std::string::npos);
    EXPECT_NE(profile.find("\"backpressure_outputs\""), std::string::npos);
    EXPECT_NE(profile.find("\"diagnostics\""), std::string::npos);
    EXPECT_NE(profile.find("\"loopCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"totalLoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"averageLoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"p50LoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"p90LoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"p99LoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"windowMs\""), std::string::npos);
    EXPECT_NE(profile.find("\"utilizationPpm\""), std::string::npos);
    EXPECT_NE(profile.find("\"budgetOverrunCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"deadlineMissCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"schedulingErrorCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"pushedPerSecond\""), std::string::npos);
    EXPECT_NE(profile.find("\"poppedPerSecond\""), std::string::npos);
    EXPECT_NE(profile.find("\"droppedPerSecond\""), std::string::npos);
    EXPECT_NE(profile.find("\"maxDepthObserved\""), std::string::npos);

    const auto topology = epg::ParseGraphConfigJsonField(profile, "topology");
    ASSERT_EQ(topology.queues.size(), 1u);
    ASSERT_EQ(topology.tasks.size(), 2u);
    EXPECT_EQ(topology.queues.front().name, "packets");
    EXPECT_EQ(topology.tasks.front().name, "source");

    const auto metadata = epg::ParseGraphProfileMetadataJson(profile);
    EXPECT_EQ(metadata.schema, epg::GRAPH_PROFILE_SCHEMA);
    EXPECT_EQ(metadata.graph, "test_graph");
    EXPECT_EQ(metadata.topologyVersion, "test-topology");
    EXPECT_EQ(metadata.timestampMs, 123u);

    const auto diagnostics = epg::ParseGraphProfileDiagnosticsJson(profile);
    ASSERT_EQ(diagnostics.queues.size(), 1u);
    ASSERT_EQ(diagnostics.tasks.size(), 2u);
    EXPECT_NE(diagnostics.queues.find("packets"), diagnostics.queues.end());
    EXPECT_NE(diagnostics.tasks.find("source"), diagnostics.tasks.end());

    const auto parsedProfile = epg::ParseGraphProfileJson(profile);
    EXPECT_EQ(parsedProfile.metadata.graph, "test_graph");
    ASSERT_EQ(parsedProfile.taskCatalog.size(), 1u);
    EXPECT_EQ(parsedProfile.taskCatalog.front().taskType, "TestSourceTask");
    EXPECT_EQ(parsedProfile.taskCatalog.front().role, "source");
    EXPECT_EQ(parsedProfile.taskCatalog.front().resource, "cpu");
    EXPECT_EQ(parsedProfile.taskCatalog.front().budgetUs, 1000u);
    EXPECT_EQ(parsedProfile.taskCatalog.front().deadlineUs, 2000u);
    ASSERT_EQ(parsedProfile.topology.queues.size(), 1u);
    ASSERT_EQ(parsedProfile.diagnostics.tasks.size(), 2u);
    EXPECT_THROW(
        epg::ParseGraphProfileJson("{\"schema\":\"smartdrone.epg.profile.v1\",\"graph\":\"missing\"}"),
        std::runtime_error);
}

TEST(EventPipelineGraph, RejectsProfileDiagnosticsMissingMetric)
{
    const auto profile = std::string(R"({
      "diagnostics": {
        "queues": {
          "packets": {
            "maxDepthObserved": 0
          }
        },
        "tasks": {"source": )") + MinimalTaskDiagnosticsJson() +
        R"(}
      }
    })";

    try {
        (void)epg::ParseGraphProfileDiagnosticsJson(profile);
        FAIL() << "expected missing diagnostic metric to be rejected";
    } catch (const std::runtime_error &error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("missing json field: droppedNewest"),
                  std::string::npos);
    }
}

TEST(EventPipelineGraphOptimizer, WritesOptimizedConfigFromFreshProfile) {
    const std::string profilePath = "/tmp/smartdrone_epg_optimizer_test_profile.json";
    const std::string outputPath = "/tmp/smartdrone_epg_optimizer_test_optimized.json";
    const std::string reportPath =
        "/tmp/smartdrone_epg_optimizer_test_optimized_report.json";
    const std::uint64_t nowMs = 1000;
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath,
        R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "budgetUs": 1500,
              "deadlineUs": 3000,
              "replaceable": true
            },
            {
              "taskType": "TestSinkTask",
              "role": "sink",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 3000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
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
                "trigger": {
                  "mode": "any_queue_ready",
                  "queues": ["packets"]
                },
                "inputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": {
            "graph": "test_graph",
            "timestampMs": 1000,
            "queues": {
              "packets": {
                "type": "TestPacket",
                "size": 4,
                "depth": 4,
                "pushed": 100,
                "popped": 70,
                "droppedNewest": 3,
                "overwrittenOldest": 0,
                "wakeups": 70,
                "maxDepthObserved": 4,
                "firstActivityMs": 1,
                "lastActivityMs": 1000,
                "windowMs": 999,
                "pushedPerSecond": 100,
                "poppedPerSecond": 70,
                "droppedPerSecond": 3
              }
            },
            "tasks": {
              "source": {
                "lastLoopUs": 2200,
                "maxLoopUs": 2600,
                "p50LoopUs": 2000,
                "p90LoopUs": 2400,
                "p99LoopUs": 2600,
                "totalLoopUs": 2600,
                "averageLoopUs": 2200,
                "loopCount": 1,
                "errorCount": 0,
                "idleWakeups": 0,
                "firstLoopMs": 1,
                "lastLoopMs": 1000,
                "windowMs": 999,
                "utilizationPpm": 900000,
                "budgetOverrunCount": 2,
                "deadlineMissCount": 0,
                "schedulingErrorCount": 0,
                "lastSchedulingError": 0
              },
              "sink": {
                "lastLoopUs": 200,
                "maxLoopUs": 300,
                "p50LoopUs": 200,
                "p90LoopUs": 250,
                "p99LoopUs": 300,
                "totalLoopUs": 300,
                "averageLoopUs": 200,
                "loopCount": 1,
                "errorCount": 0,
                "idleWakeups": 0,
                "firstLoopMs": 1,
                "lastLoopMs": 1000,
                "windowMs": 999,
                "utilizationPpm": 100000,
                "budgetOverrunCount": 0,
                "deadlineMissCount": 0,
                "schedulingErrorCount": 0,
                "lastSchedulingError": 0
              }
            }
          }
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath = reportPath;
    manifest.catalog[0] =
        {"TestSourceTask", "source", "cpu", 1500, 3000, true};
    manifest.catalog.push_back(
        {"TestSinkTask", "sink", "cpu", 1000, 3000, false});

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 10);
    EXPECT_TRUE(result.optimized) << result.message;

    const std::string optimized = ReadFileText(outputPath);
    EXPECT_NE(optimized.find(std::string("\"schema\": \"") +
                                 epg::OPTIMIZED_GRAPH_SCHEMA + "\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"targetGraph\": \"test_graph\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"sourceProfile\": \"test_graph\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"topologyVersion\": \"test-topology\""),
              std::string::npos);
    EXPECT_NE(optimized.find(std::string("\"solverVersion\": \"") +
                                 epg::NATIVE_HEURISTIC_SOLVER_VERSION + "\""),
              std::string::npos);
    EXPECT_NE(optimized.find("\"generatedAtMs\": 1010"),
              std::string::npos);

    const auto config = epg::ParseGraphConfigJson(optimized);
    ASSERT_EQ(config.queues.size(), 1u);
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.queues.front().depth, 8u);
    EXPECT_EQ(config.tasks.front().trigger.interval.count(), 3);
    EXPECT_EQ(config.tasks.front().scheduling.budgetUs, 1500u);
    EXPECT_EQ(config.tasks.front().scheduling.deadlineUs, 3000u);

    const std::string report = ReadFileText(reportPath);
    EXPECT_NE(report.find(std::string("\"schema\": \"") +
                              epg::SOLVER_REPORT_SCHEMA + "\""),
              std::string::npos);
    EXPECT_NE(report.find("\"targetGraph\": \"test_graph\""),
              std::string::npos);
    EXPECT_NE(report.find("\"topologyVersion\": \"test-topology\""),
              std::string::npos);
    EXPECT_NE(report.find(std::string("\"solverVersion\": \"") +
                              epg::NATIVE_HEURISTIC_SOLVER_VERSION + "\""),
              std::string::npos);
    EXPECT_NE(report.find("\"generatedAtMs\": 1010"), std::string::npos);
    EXPECT_NE(report.find("\"reason\": \"increase_depth\""),
              std::string::npos);
    EXPECT_NE(report.find("\"reason\": \"increase_interval\""),
              std::string::npos);
    EXPECT_NE(report.find("\"catalogRole\": \"source\""),
              std::string::npos);
    EXPECT_NE(report.find("\"replaceable\": true"), std::string::npos);
    EXPECT_NE(report.find("\"budgetOverruns\": 2"), std::string::npos);
    const auto optimizedGraph = epg::ParseOptimizedGraphJson(optimized);
    const auto parsedReport = epg::ParseSolverReportJson(report);
    EXPECT_NO_THROW(
        smartdrone::core::application::ValidateEpgSolverReportManifest(
            manifest, optimizedGraph.metadata, parsedReport.metadata));
    EXPECT_EQ(parsedReport.objectiveName,
              "minimize_epg_pressure_overload_deadline_and_scheduling_penalty");
    EXPECT_EQ(parsedReport.constraints.maxQueueDepth, 16u);
    ASSERT_EQ(parsedReport.decisions.size(), 3u);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
}

TEST(EventPipelineGraphOptimizer, KeepsNonReplaceableTaskInterval) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_non_replaceable_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_non_replaceable_optimized.json";
    const std::string reportPath =
        "/tmp/smartdrone_epg_optimizer_non_replaceable_report.json";
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath, NonReplaceableTaskProfileJson());

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath = reportPath;

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    const auto config = epg::ParseGraphConfigJson(ReadFileText(outputPath));
    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.tasks.front().trigger.interval.count(), 1);
    EXPECT_NE(ReadFileText(reportPath).find("not_replaceable+"),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
}

TEST(EventPipelineGraphOptimizer, RejectsProfileTasksOutsideManifest) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_outside_manifest_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_outside_manifest_optimized.json";
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 2000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
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
                "trigger": {
                  "mode": "any_queue_ready",
                  "queues": ["packets"]
                },
                "inputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": )") + MinimalProfileDiagnosticsJson(1000) +
          R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("outside manifest"), std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
}

TEST(EventPipelineGraphOptimizer, RejectsProfileCatalogMismatch) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_catalog_mismatch_profile.json";
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "gpu",
              "budgetUs": 1000,
              "deadlineUs": 2000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
            ],
            "tasks": [
              {
                "name": "source",
                "type": "TestSourceTask",
                "trigger": {"mode": "periodic", "interval_ms": 1},
                "outputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": )") + MinimalProfileDiagnosticsJson(1000) +
          R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("catalog mismatch"), std::string::npos);

    (void)std::remove(profilePath.c_str());
}

TEST(EventPipelineGraphOptimizer, RejectsProfileMissingTaskDiagnostics) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_missing_task_diag_profile.json";
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath, MissingTaskDiagnosticsProfileJson());

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("profile diagnostics missing task: source"),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
}

TEST(EventPipelineGraphOptimizer, CreatesArtifactDirectories) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_nested_profile.json";
    const std::string outputDir =
        "/tmp/smartdrone_epg_optimizer_nested_artifacts";
    const std::string outputPath = outputDir + "/optimized/config.json";
    const std::string reportPath = outputDir + "/reports/report.json";
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 2000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
            ],
            "tasks": [
              {
                "name": "source",
                "type": "TestSourceTask",
                "trigger": {"mode": "periodic", "interval_ms": 1},
                "outputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": )") + MinimalProfileDiagnosticsJson(1000) +
          R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath = reportPath;

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_TRUE(result.optimized) << result.message;
    EXPECT_NE(ReadFileText(outputPath).find(epg::OPTIMIZED_GRAPH_SCHEMA),
              std::string::npos);
    EXPECT_NE(ReadFileText(reportPath).find(epg::SOLVER_REPORT_SCHEMA),
              std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(reportPath.c_str());
    (void)std::remove((outputDir + "/optimized").c_str());
    (void)std::remove((outputDir + "/reports").c_str());
    (void)std::remove(outputDir.c_str());
}

TEST(EventPipelineGraphOptimizer, ReportsArtifactWriteFailure) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_write_failure_profile.json";
    const std::string blockedParent =
        "/tmp/smartdrone_epg_optimizer_blocked_parent";
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        blockedParent, "{}");
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 1000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 2000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
            ],
            "tasks": [
              {
                "name": "source",
                "type": "TestSourceTask",
                "trigger": {"mode": "periodic", "interval_ms": 1},
                "outputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": )") + MinimalProfileDiagnosticsJson(1000) +
          R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = blockedParent + "/config.json";
    manifest.artifactPaths.solverReportPath = blockedParent + "/report.json";

    const auto result =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, 1010);
    EXPECT_FALSE(result.optimized);
    EXPECT_NE(result.message.find("not a directory"), std::string::npos);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(blockedParent.c_str());
}

TEST(EventPipelineGraphOptimizer, ReportsUnchangedConfigWithoutRedeploy) {
    const std::string profilePath =
        "/tmp/smartdrone_epg_optimizer_unchanged_profile.json";
    const std::string outputPath =
        "/tmp/smartdrone_epg_optimizer_unchanged_optimized.json";
    const std::uint64_t nowMs = 2000;
    smartdrone::core::application::WriteEpgDfxSnapshotFile(
        profilePath,
        std::string(R"({
          "schema": "smartdrone.epg.profile.v1",
          "graph": "test_graph",
          "topologyVersion": "test-topology",
          "timestampMs": 2000,
          "taskCatalog": [
            {
              "taskType": "TestSourceTask",
              "role": "source",
              "resource": "cpu",
              "budgetUs": 1000,
              "deadlineUs": 2000,
              "replaceable": false
            }
          ],
          "topology": {
            "queues": [
              {
                "name": "packets",
                "type": "TestPacket",
                "depth": 4,
                "overflow": "drop_newest"
              }
            ],
            "tasks": [
              {
                "name": "source",
                "type": "TestSourceTask",
                "trigger": {"mode": "periodic", "interval_ms": 1},
                "outputs": {"0": "packets"}
              }
            ]
          },
          "diagnostics": )") + MinimalProfileDiagnosticsJson(2000) +
          R"(
        })");

    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    manifest.artifactPaths.profilePath = profilePath;
    manifest.artifactPaths.optimizedConfigPath = outputPath;
    manifest.artifactPaths.solverReportPath =
        "/tmp/smartdrone_epg_optimizer_unchanged_optimized_report.json";

    const auto first =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 10);
    ASSERT_TRUE(first.optimized) << first.message;
    EXPECT_TRUE(first.configChanged);

    const auto second =
        smartdrone::core::application::OptimizeEpgProfileForManifest(
            manifest, nowMs + 20);
    EXPECT_TRUE(second.optimized) << second.message;
    EXPECT_FALSE(second.configChanged);

    (void)std::remove(profilePath.c_str());
    (void)std::remove(outputPath.c_str());
    (void)std::remove(manifest.artifactPaths.solverReportPath.c_str());
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
          "scheduling": {
            "resource": "cpu",
            "cpu_affinity": -1,
            "budget_us": 500,
            "deadline_us": 900,
            "backpressure_outputs": [0],
            "realtime": true,
            "priority": 42
          },
          "outputs": {"0": "packets"}
        }
      ]
    })");

    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.tasks.front().name, "source\nname");
    EXPECT_EQ(config.tasks.front().scheduling.resource, "cpu");
    EXPECT_EQ(config.tasks.front().scheduling.cpuAffinity, -1);
    EXPECT_EQ(config.tasks.front().scheduling.budgetUs, 500u);
    EXPECT_EQ(config.tasks.front().scheduling.deadlineUs, 900u);
    EXPECT_EQ(config.tasks.front().scheduling.backpressureOutputs,
              std::vector<epg::PortId>{0});
    EXPECT_TRUE(config.tasks.front().scheduling.realtime);
    EXPECT_EQ(config.tasks.front().scheduling.priority, 42);
    EXPECT_THROW(
        epg::ParseGraphConfigJsonFile("/tmp/smart_drone_missing_epg.json"),
        std::runtime_error);
}

TEST(GraphConfig, ParsesOptimizedRuntimeConfigJson) {
    const auto config = epg::ParseGraphConfigJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 6, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 3},
          "outputs": {"0": "packets"}
        },
        {
          "name": "sink",
          "type": "TestSinkTask",
          "trigger": {"mode": "any_queue_ready", "queues": ["packets"]},
          "inputs": {"0": "packets"}
        }
      ]
    })");

    ASSERT_EQ(config.queues.size(), 1u);
    EXPECT_EQ(config.queues.front().depth, 6u);
    ASSERT_EQ(config.tasks.size(), 2u);
    EXPECT_EQ(config.tasks.front().trigger.interval,
              std::chrono::milliseconds(3));

    const auto metadata = epg::ParseOptimizedGraphMetadataJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [],
      "tasks": []
    })");
    EXPECT_EQ(metadata.schema, epg::OPTIMIZED_GRAPH_SCHEMA);
    EXPECT_EQ(metadata.targetGraph, "test_graph");
    EXPECT_EQ(metadata.topologyVersion, "test-topology");
    EXPECT_EQ(metadata.solverVersion, "unit-test");
    EXPECT_EQ(metadata.sourceProfile, "test_graph");
    EXPECT_EQ(metadata.sourceTimestampMs, 123u);
    EXPECT_EQ(metadata.generatedAtMs, 456u);

    const auto optimized = epg::ParseOptimizedGraphJson(R"({
      "schema": "smartdrone.epg.optimized_config.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "solverVersion": "unit-test",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "queues": [
        {"name": "packets", "type": "TestPacket", "depth": 6, "overflow": "drop_newest"}
      ],
      "tasks": [
        {
          "name": "source",
          "type": "TestSourceTask",
          "trigger": {"mode": "periodic", "interval_ms": 3},
          "outputs": {"0": "packets"}
        }
      ]
    })");
    EXPECT_EQ(optimized.metadata.targetGraph, "test_graph");
    ASSERT_EQ(optimized.config.queues.size(), 1u);
    ASSERT_EQ(optimized.config.tasks.size(), 1u);
    const auto report = epg::ParseSolverReportJson(R"({
      "schema": "smartdrone.epg.solver_report.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "solverVersion": "unit-test",
      "objective": {
        "name": "unit",
        "score": {
          "queuePressure": 0,
          "periodicOverloadUs": 0,
          "schedulingErrors": 0,
          "budgetOverruns": 0,
          "deadlineMisses": 0,
          "utilizationOverPpm": 0,
          "totalPenalty": 0
        }
      },
      "constraints": {
        "maxQueueDepth": 16,
        "maxPeriodicIntervalMs": 1000,
        "targetUtilizationPpm": 800000
      },
      "decisions": [
        {"kind": "task", "name": "source", "reason": "keep"}
      ]
    })");
    EXPECT_EQ(report.metadata.schema, epg::SOLVER_REPORT_SCHEMA);
    EXPECT_EQ(report.metadata.targetGraph, "test_graph");
    EXPECT_EQ(report.metadata.topologyVersion, "test-topology");
    EXPECT_EQ(report.metadata.sourceProfile, "test_graph");
    EXPECT_EQ(report.metadata.sourceTimestampMs, 123u);
    EXPECT_EQ(report.metadata.generatedAtMs, 456u);
    EXPECT_EQ(report.metadata.solverVersion, "unit-test");
    EXPECT_EQ(report.objectiveName, "unit");
    EXPECT_EQ(report.score.totalPenalty, 0u);
    EXPECT_EQ(report.constraints.maxQueueDepth, 16u);
    ASSERT_EQ(report.decisions.size(), 1u);
    EXPECT_EQ(report.decisions.front().name, "source");
    EXPECT_THROW(
        epg::ParseSolverReportJson(R"({
          "schema": "smartdrone.epg.solver_report.v1",
          "targetGraph": "test_graph",
          "topologyVersion": "test-topology",
          "sourceProfile": "test_graph",
          "sourceTimestampMs": 123,
          "generatedAtMs": 456,
          "solverVersion": "unit-test",
          "objective": {"name": "unit", "score": {}},
          "constraints": {},
          "decisions": []
        })"),
        std::runtime_error);
    EXPECT_THROW(
        epg::ParseOptimizedGraphJson(R"({
          "schema": "smartdrone.epg.optimized_config.v1",
          "targetGraph": "test_graph",
          "topologyVersion": "test-topology",
          "solverVersion": "unit-test",
          "queues": [],
          "tasks": []
        })"),
        std::runtime_error);
}

TEST(GraphConfig, ParsesNestedGraphConfigJsonField) {
    const auto config = epg::ParseGraphConfigJsonField(R"({
      "schema": "smartdrone.epg.profile.v1",
      "topology": {
        "queues": [
          {"name": "packets", "type": "TestPacket", "depth": 4, "overflow": "drop_newest"}
        ],
        "tasks": [
          {
            "name": "source",
            "type": "TestSourceTask",
            "trigger": {"mode": "periodic", "interval_ms": 1},
            "outputs": {"0": "packets"}
          }
        ]
      },
      "diagnostics": {}
    })", "topology");

    ASSERT_EQ(config.queues.size(), 1u);
    ASSERT_EQ(config.tasks.size(), 1u);
    EXPECT_EQ(config.queues.front().name, "packets");
    EXPECT_EQ(config.tasks.front().name, "source");
    EXPECT_THROW(
        epg::ParseGraphConfigJsonField("{\"diagnostics\": {}}", "topology"),
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
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"realtime":true,"priority":0}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"realtime":true,"priority":100}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"resource":""}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"cpu_affinity":-2}}]})",
        R"({"queues":[],"tasks":[{"name":"heartbeat","type":"TestHeartbeatTask","trigger":{"mode":"periodic","interval_ms":1},"scheduling":{"budget_us":2000,"deadline_us":1000}}]})",
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
