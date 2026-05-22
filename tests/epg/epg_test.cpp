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

using Epg::EventPipelineGraph;
using Epg::ITask;
using Epg::OverflowPolicy;
using Epg::PortSpec;
using Epg::Registry;
using Epg::SpscSharedPtrQueue;
using Epg::TaskContext;
using SmartDrone::Core::Application::BuildEpgTaskTopologyVersion;
using SmartDrone::Core::Application::EpgDomain;
using SmartDrone::Core::Application::EpgManifestForDomain;
using SmartDrone::Core::Application::EpgTaskCatalogTypes;

struct TestPacket {
    int sequence{};
};

struct OtherPacket {
    int value{};
};

struct ReflectedPacket {
    int sequence{};
};
const bool REFLECTED_PACKET_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<ReflectedPacket>(
        "ReflectedPacket");

struct SlamResourceReady {};
struct SlamTick {};
struct SlamImuReady {};
struct SlamFrameReady {};
struct SlamPreparedFrame {};
struct SlamTrackedFrame {};
struct SlamPublishedFrame {};
struct SlamPreviewReady {};
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
struct SystemRuntimePulse {};

class TestSourceTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        auto packet = context.Make<TestPacket>();
        packet->sequence = ++m_sequence;
        context.Push<TestPacket>(0, std::move(packet));
    }

  private:
    int m_sequence{};
};

class TestSecondSourceTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        auto packet = context.Make<TestPacket>();
        packet->sequence = ++m_sequence;
        context.Push<TestPacket>(0, std::move(packet));
    }

  private:
    int m_sequence{};
};

class TestFanoutSourceTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
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
    void OnTick(TaskContext &context) override
    {
        while (auto packet = context.TryPop<TestPacket>(0)) {
            (void)packet;
        }
    }
};

class TestForwardTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        while (auto packet = context.TryPop<TestPacket>(0)) {
            auto forwarded = context.Make<TestPacket>();
            forwarded->sequence = packet->sequence;
            context.Push<TestPacket>(0, std::move(forwarded));
        }
    }
};

class TestAllInputsSinkTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        auto left = context.TryPop<TestPacket>(0);
        auto right = context.TryPop<TestPacket>(1);
        if (!left || !right) {
            throw std::runtime_error("all-input sink woke before both queues were ready");
        }
    }
};

class TestHeartbeatTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        (void)context;
    }
};

class TestThrowingTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        (void)context;
        throw std::runtime_error("expected test exception");
    }
};

class TestBadContextTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        (void)context.TryPop<TestPacket>(999);
    }
};

class TestCountingTask final : public ITask {
  public:
    explicit TestCountingTask(int &ticks)
        : m_ticks(ticks)
    {
    }

    void OnTick(TaskContext &context) override
    {
        (void)context;
        ++m_ticks;
    }

  private:
    int &m_ticks;
};

class TestCountingSinkTask final : public ITask {
  public:
    explicit TestCountingSinkTask(int &packets)
        : m_packets(packets)
    {
    }

    void OnTick(TaskContext &context) override
    {
        while (auto packet = context.TryPop<TestPacket>(0)) {
            (void)packet;
            ++m_packets;
        }
    }

  private:
    int &m_packets;
};

class TestSlowTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        (void)context;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
};

class ReflectedSourceTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        auto packet = context.Make<ReflectedPacket>();
        packet->sequence = ++m_sequence;
        context.Push(0, std::move(packet));
    }

  private:
    int m_sequence{};
};
const bool REFLECTED_SOURCE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTask<ReflectedSourceTask>(
        "ReflectedSourceTask",
        std::vector<Epg::PortSpec>{},
        std::vector<Epg::PortSpec>{{0, "ReflectedPacket"}});

class ReflectedSinkTask final : public ITask {
  public:
    void OnTick(TaskContext &context) override
    {
        while (auto packet = context.TryPop<ReflectedPacket>(0)) {
            (void)packet;
        }
    }
};
const bool REFLECTED_SINK_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTask<ReflectedSinkTask>(
        "ReflectedSinkTask",
        std::vector<Epg::PortSpec>{{0, "ReflectedPacket"}},
        std::vector<Epg::PortSpec>{});

Registry MakeRegistry()
{
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

Registry MakeSlamShapeRegistry()
{
    Registry registry;
    registry.RegisterMessageType<SlamResourceReady>("SlamResourceReady");
    registry.RegisterMessageType<SlamTick>("SlamTick");
    registry.RegisterMessageType<SlamImuReady>("SlamImuReady");
    registry.RegisterMessageType<SlamFrameReady>("SlamFrameReady");
    registry.RegisterMessageType<SlamPreparedFrame>("SlamPreparedFrame");
    registry.RegisterMessageType<SlamTrackedFrame>("SlamTrackedFrame");
    registry.RegisterMessageType<SlamPublishedFrame>("SlamPublishedFrame");
    registry.RegisterMessageType<SlamPreviewReady>("SlamPreviewReady");
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
    registry.RegisterTaskFactory("SlamPreviewTxTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamDfxTask", {}, {}, factory);
    registry.RegisterTaskFactory("SlamMonitorTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgDfxSnapshotTask", {}, {}, factory);
    return registry;
}

Registry MakeSystemShapeRegistry()
{
    Registry registry;
    registry.RegisterMessageType<SystemRuntimePulse>("SystemRuntimePulse");
    const auto factory = []() {
        return std::unique_ptr<ITask>(new TestHeartbeatTask());
    };
    registry.RegisterTaskFactory("VehicleTelemetryRxTask", {}, {}, factory);
    registry.RegisterTaskFactory("SetpointStreamTask", {}, {}, factory);
    registry.RegisterTaskFactory("UdpReceiveTask", {}, {}, factory);
    registry.RegisterTaskFactory("UdpHeartbeatTxTask", {}, {}, factory);
    registry.RegisterTaskFactory("UdpHeartbeatTimeoutTask", {}, {}, factory);
    registry.RegisterTaskFactory("UdpStateTxTask", {}, {}, factory);
    registry.RegisterTaskFactory("UdpPointCloudTxTask", {}, {}, factory);
    registry.RegisterTaskFactory("ManualControlTask", {}, {}, factory);
    registry.RegisterTaskFactory("ForceRestartTask", {}, {}, factory);
    registry.RegisterTaskFactory("RuntimeSupervisorTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgRedeployTask", {}, {}, factory);
    registry.RegisterTaskFactory("DiscoveryBeaconTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgDfxSnapshotTask", {}, {}, factory);
    registry.RegisterTaskFactory("EpgOptimizeTask", {}, {}, factory);
    return registry;
}

Registry MakeCalibShapeRegistry()
{
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

std::string MinimalValidJson()
{
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

void ExpectConfigureThrows(const std::string &json)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);
    EXPECT_THROW(graph.ConfigureJson(json), std::runtime_error);
}

void RunTopology(const Epg::GraphConfig &config,
                 const std::vector<std::string> &queues,
                 const std::vector<std::string> &tasks)
{
    auto registry = MakeRegistry();
    EventPipelineGraph graph(registry);

    graph.Configure(config);

    graph.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    graph.Stop();

    const auto queueDiagnostics = graph.QueueDiagnostics();
    for (const auto &queueName : queues) {
        const auto queueIt = queueDiagnostics.find(queueName);
        ASSERT_NE(queueIt, queueDiagnostics.end()) << queueName;
        EXPECT_GT(queueIt->second.pushed, 0u) << queueName;
        EXPECT_GT(queueIt->second.popped, 0u) << queueName;
        EXPECT_GT(queueIt->second.wakeups, 0u) << queueName;
    }

    const auto taskDiagnostics = graph.TaskDiagnostics();
    for (const auto &taskName : tasks) {
        const auto taskIt = taskDiagnostics.find(taskName);
        ASSERT_NE(taskIt, taskDiagnostics.end()) << taskName;
        EXPECT_GT(taskIt->second.loopCount, 0u) << taskName;
        EXPECT_EQ(taskIt->second.errorCount, 0u) << taskName;
    }
}

void RunTopologyFromJsonFile(const std::string &jsonFile,
                             const std::vector<std::string> &queues,
                             const std::vector<std::string> &tasks)
{
    RunTopology(
        Epg::ParseGraphConfigJsonFile(
            std::string(TEST_EPG_DIR) + "/" + jsonFile),
        queues,
        tasks);
}

void RunTopologyFromDotFile(const std::string &dotFile,
                            const std::vector<std::string> &queues,
                            const std::vector<std::string> &tasks)
{
    auto registry = MakeRegistry();
    RunTopology(
        Epg::ParseGraphConfigDotFile(
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

SmartDrone::Core::Application::EpgTaskManifest MakeValidTestManifest()
{
    SmartDrone::Core::Application::EpgTaskManifest manifest;
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

std::string ReadFileText(const std::string &path)
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
            "resourceWaitCount": 0,
            "maxResourceWaitUs": 0,
            "averageResourceWaitUs": 0,
            "totalResourceWaitUs": 0,
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
            "timestampMs": )") +
           std::to_string(timestampMs) +
           R"(,
            "queues": {"packets": )" +
           MinimalQueueDiagnosticsJson() +
           R"(},
            "tasks": {"source": )" +
           MinimalTaskDiagnosticsJson() +
           R"(}
          })";
}

std::string MissingTaskProfileDiagnosticsJson(std::uint64_t timestampMs)
{
    return std::string(R"({
            "graph": "test_graph",
            "timestampMs": )") +
           std::to_string(timestampMs) +
           R"(,
            "queues": {"packets": )" +
           MinimalQueueDiagnosticsJson() +
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
      "diagnostics": )") +
           MissingTaskProfileDiagnosticsJson(1000) +
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
        "queues": {"packets": )") +
           MinimalQueueDiagnosticsJson() +
           R"(},
        "tasks": {
          "source": {
            "maxLoopUs": 2600,
            "p90LoopUs": 2400,
            "p99LoopUs": 2600,
            "averageLoopUs": 2200,
            "resourceWaitCount": 0,
            "maxResourceWaitUs": 0,
            "averageResourceWaitUs": 0,
            "totalResourceWaitUs": 0,
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

#include "epg_test_runtime_basic_cases.inc"
#include "epg_test_runtime_validation_cases.inc"
#include "epg_test_manifest_cases.inc"
#include "epg_test_optimizer_cases.inc"
#include "epg_test_config_cases.inc"
