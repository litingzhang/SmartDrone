const char *const MANIFEST_OPTIMIZED_GRAPH_JSON = R"({
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
            "cpu_affinity": -1,
            "budget_us": 1000,
            "deadline_us": 2000,
            "topology_level": 0,
            "phase_offset_ms": 0
          },
          "outputs": {"0": "packets"}
        }
      ]
    })";

const char *const MANIFEST_SOLVER_REPORT_JSON = R"({
      "schema": "smartdrone.epg.solver_report.v1",
      "targetGraph": "test_graph",
      "topologyVersion": "test-topology",
      "sourceProfile": "test_graph",
      "sourceTimestampMs": 123,
      "generatedAtMs": 456,
      "solverVersion": "unit-test",
      "objective": {
        "name": "global_minimize_predicted_epg_penalty_discrete_topology",
        "score": {
          "queuePressure": 2,
          "periodicOverloadUs": 500,
          "resourceWaitUs": 0,
          "schedulingErrors": 1,
          "budgetOverruns": 2,
          "deadlineMisses": 3,
          "utilizationOverPpm": 100000,
          "totalPenalty": 131500
        }
      },
      "constraints": {
        "maxQueueDepth": 16,
        "maxPeriodicIntervalMs": 1000,
        "targetUtilizationPpm": 800000
      },
      "decisions": [
        {
          "kind": "queue",
          "name": "packets",
          "depthBefore": 4,
          "depthAfter": 4,
          "pressureBefore": 2,
          "pressureAfter": 2,
          "maxDepthObserved": 4,
          "droppedNewest": 2,
          "overwrittenOldest": 0,
          "pushedPerSecond": 0,
          "poppedPerSecond": 0,
          "droppedPerSecond": 0,
          "reason": "keep"
        },
        {
          "kind": "task",
          "name": "source",
          "intervalBeforeMs": 1,
          "intervalAfterMs": 1,
          "maxLoopUs": 1500,
          "averageLoopUs": 1000,
          "p90LoopUs": 1200,
          "p99LoopUs": 1500,
          "effectiveLoopUs": 1500,
          "resourceWaitCount": 0,
          "maxResourceWaitUs": 0,
          "averageResourceWaitUs": 0,
          "totalResourceWaitUs": 0,
          "predictedResourceWaitUs": 0,
          "utilizationPpm": 900000,
          "targetUtilizationPpm": 800000,
          "budgetUs": 1000,
          "deadlineUs": 2000,
          "topologyLevel": 0,
          "phaseOffsetMs": 0,
          "durationMs": 2,
          "cpuBindingAffinity": 0,
          "cpuBindingStartMs": 0,
          "cpuBindingFinishMs": 2,
          "cpuBindingMakespanMs": 2,
          "cpuAffinityBefore": -1,
          "cpuAffinityAfter": -1,
          "catalogRole": "source",
          "replaceable": false,
          "resourceBefore": "cpu",
          "resourceAfter": "cpu",
          "budgetOverrunCount": 2,
          "deadlineMissCount": 3,
          "schedulingErrorCount": 1,
          "reason": "global_optimum_cpu_binding"
        }
      ]
    })";

Epg::GraphProfile MakeManifestSourceProfile(
    const Epg::OptimizedGraph &optimized)
{
    auto sourceProfile = Epg::GraphProfile{};
    sourceProfile.metadata.schema = Epg::GRAPH_PROFILE_SCHEMA;
    sourceProfile.metadata.graph = "test_graph";
    sourceProfile.metadata.topologyVersion = "test-topology";
    sourceProfile.metadata.timestampMs = 123;
    sourceProfile.topology = optimized.config;
    sourceProfile.topology.queues[0].depth = 4;
    sourceProfile.topology.tasks[0].scheduling.cpuAffinity = -1;
    sourceProfile.diagnostics.queues["packets"].maxDepthObserved = 4;
    sourceProfile.diagnostics.queues["packets"].droppedNewest = 2;
    sourceProfile.diagnostics.tasks["source"].maxLoopUs = 1500;
    sourceProfile.diagnostics.tasks["source"].averageLoopUs = 1000;
    sourceProfile.diagnostics.tasks["source"].p90LoopUs = 1200;
    sourceProfile.diagnostics.tasks["source"].p99LoopUs = 1500;
    sourceProfile.diagnostics.tasks["source"].utilizationPpm = 900000;
    sourceProfile.diagnostics.tasks["source"].budgetOverrunCount = 2;
    sourceProfile.diagnostics.tasks["source"].deadlineMissCount = 3;
    sourceProfile.diagnostics.tasks["source"].schedulingErrorCount = 1;
    return sourceProfile;
}

void ExpectManifestProfileHappyPath(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimized,
    const Epg::GraphProfile &sourceProfile,
    const Epg::SolverReport &report)
{
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, optimized));
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, optimized, report));
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, report));
}

void ExpectOptimizedGraphManifestRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimized)
{
    auto wrongTarget = optimized;
    wrongTarget.metadata.targetGraph = "other_graph";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongTarget),
        std::runtime_error);

    auto wrongTopology = optimized;
    wrongTopology.metadata.topologyVersion = "other-topology";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongTopology),
        std::runtime_error);

    auto wrongSource = optimized;
    wrongSource.metadata.sourceProfile = "other_graph";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongSource),
        std::runtime_error);
}

void ExpectOptimizedGraphConfigRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimized)
{
    auto wrongGeneration = optimized;
    wrongGeneration.metadata.generatedAtMs = 1;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongGeneration),
        std::runtime_error);

    auto wrongTask = optimized;
    wrongTask.config.tasks[0].type = "TestSinkTask";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongTask),
        std::runtime_error);

    auto wrongScheduling = optimized;
    wrongScheduling.config.tasks[0].scheduling.budgetUs = 999;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, wrongScheduling),
        std::runtime_error);
}

void ExpectSolverReportMetadataRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto wrongReportTime = report.metadata;
    wrongReportTime.generatedAtMs = 999;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReportManifest(
            manifest, optimized.metadata, wrongReportTime),
        std::runtime_error);

    auto wrongReportVersion = report.metadata;
    wrongReportVersion.solverVersion = "other-solver";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReportManifest(
            manifest, optimized.metadata, wrongReportVersion),
        std::runtime_error);
}

void ExpectSolverReportDecisionRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto missingDecision = report;
    missingDecision.decisions.pop_back();
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, missingDecision),
        std::runtime_error);

    auto duplicateDecision = report;
    duplicateDecision.decisions.push_back(duplicateDecision.decisions.front());
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, duplicateDecision),
        std::runtime_error);
}

void ExpectSolverReportScoreRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto wrongScore = report;
    wrongScore.score.totalPenalty = 1;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongScore),
        std::runtime_error);

    auto wrongScoreComponent = report;
    wrongScoreComponent.score.queuePressure = 1;
    wrongScoreComponent.score.totalPenalty = 130500;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongScoreComponent),
        std::runtime_error);
}

void ExpectSolverReportDetailRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto wrongDecisionDepth = report;
    wrongDecisionDepth.decisions[0].depthAfter = 999;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongDecisionDepth),
        std::runtime_error);

    auto wrongTaskInterval = report;
    wrongTaskInterval.decisions[1].intervalAfterMs = 999;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongTaskInterval),
        std::runtime_error);

    auto wrongConstraint = report;
    wrongConstraint.constraints.maxPeriodicIntervalMs = 0;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongConstraint),
        std::runtime_error);
}

void ExpectSolverReportCatalogRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto wrongCatalogRole = report;
    wrongCatalogRole.decisions[1].catalogRole = "other";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongCatalogRole),
        std::runtime_error);

    auto wrongReplaceable = report;
    wrongReplaceable.decisions[1].replaceable = true;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongReplaceable),
        std::runtime_error);

    auto wrongReason = report;
    wrongReason.decisions[1].reason = "keep";
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongReason),
        std::runtime_error);
}

void ExpectSourceProfileRejections(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto wrongSourceProfile = sourceProfile;
    wrongSourceProfile.metadata.timestampMs = 999;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, wrongSourceProfile, optimized, report),
        std::runtime_error);

    auto wrongSourceQueue = sourceProfile;
    wrongSourceQueue.topology.queues[0].depth = 999;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, wrongSourceQueue, optimized, report),
        std::runtime_error);

    auto wrongSourceTask = sourceProfile;
    wrongSourceTask.topology.tasks[0].trigger.interval =
        std::chrono::milliseconds(999);
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, wrongSourceTask, optimized, report),
        std::runtime_error);
}

void ExpectNonOptimalReportRejected(
    const SmartDrone::Core::Application::EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimized,
    const Epg::SolverReport &report)
{
    auto nonOptimalReport = report;
    nonOptimalReport.decisions[0].depthAfter = 6;
    nonOptimalReport.decisions[0].pressureAfter = 0;
    nonOptimalReport.decisions[0].reason = "global_optimum_depth";
    nonOptimalReport.score.queuePressure = 0;
    nonOptimalReport.score.totalPenalty = 129500;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, nonOptimalReport),
        std::runtime_error);
}

void ExpectProfileJsonTextFields(const std::string &profile)
{
    EXPECT_NE(profile.find(std::string("\"schema\": \"") +
                           Epg::GRAPH_PROFILE_SCHEMA + "\""),
              std::string::npos);
    EXPECT_NE(profile.find("\"graph\": \"test_graph\""), std::string::npos);
    EXPECT_NE(profile.find("\"topologyVersion\": \"test-topology\""),
              std::string::npos);
    EXPECT_NE(profile.find("\"taskCatalog\""), std::string::npos);
    EXPECT_NE(profile.find("\"timestampMs\": 123"), std::string::npos);
    EXPECT_NE(profile.find("\"topology\""), std::string::npos);
    EXPECT_NE(profile.find("\"queues\""), std::string::npos);
    EXPECT_NE(profile.find("\"tasks\""), std::string::npos);
}

void ExpectProfileJsonTopologyText(const std::string &profile)
{
    EXPECT_NE(profile.find("\"name\": \"packets\""), std::string::npos);
    EXPECT_NE(profile.find("\"type\": \"TestPacket\""), std::string::npos);
    EXPECT_NE(profile.find("\"overflow\": \"drop_newest\""),
              std::string::npos);
    EXPECT_NE(profile.find("\"name\": \"source\""), std::string::npos);
    EXPECT_NE(profile.find("\"mode\": \"periodic\""), std::string::npos);
    EXPECT_NE(profile.find("\"interval_ms\": 1"), std::string::npos);
    EXPECT_NE(profile.find("\"backpressure_outputs\""), std::string::npos);
}

void ExpectProfileJsonDiagnosticText(const std::string &profile)
{
    EXPECT_NE(profile.find("\"diagnostics\""), std::string::npos);
    EXPECT_NE(profile.find("\"loopCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"totalLoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"averageLoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"p50LoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"p90LoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"p99LoopUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"windowMs\""), std::string::npos);
    EXPECT_NE(profile.find("\"utilizationPpm\""), std::string::npos);
}

void ExpectProfileJsonResourceText(const std::string &profile)
{
    EXPECT_NE(profile.find("\"resourceWaitCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"maxResourceWaitUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"averageResourceWaitUs\""), std::string::npos);
    EXPECT_NE(profile.find("\"budgetOverrunCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"deadlineMissCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"schedulingErrorCount\""), std::string::npos);
    EXPECT_NE(profile.find("\"pushedPerSecond\""), std::string::npos);
    EXPECT_NE(profile.find("\"poppedPerSecond\""), std::string::npos);
    EXPECT_NE(profile.find("\"droppedPerSecond\""), std::string::npos);
    EXPECT_NE(profile.find("\"maxDepthObserved\""), std::string::npos);
}

void ExpectParsedProfileJson(const std::string &profile)
{
    const auto topology = Epg::ParseGraphConfigJsonField(profile, "topology");
    ASSERT_EQ(topology.queues.size(), 1u);
    ASSERT_EQ(topology.tasks.size(), 2u);
    EXPECT_EQ(topology.queues.front().name, "packets");
    EXPECT_EQ(topology.tasks.front().name, "source");

    const auto metadata = Epg::ParseGraphProfileMetadataJson(profile);
    EXPECT_EQ(metadata.schema, Epg::GRAPH_PROFILE_SCHEMA);
    EXPECT_EQ(metadata.graph, "test_graph");
    EXPECT_EQ(metadata.topologyVersion, "test-topology");
    EXPECT_EQ(metadata.timestampMs, 123u);
}

void ExpectParsedProfileDetails(const std::string &profile)
{
    const auto diagnostics = Epg::ParseGraphProfileDiagnosticsJson(profile);
    ASSERT_EQ(diagnostics.queues.size(), 1u);
    ASSERT_EQ(diagnostics.tasks.size(), 2u);
    EXPECT_NE(diagnostics.queues.find("packets"), diagnostics.queues.end());
    EXPECT_NE(diagnostics.tasks.find("source"), diagnostics.tasks.end());

    const auto parsedProfile = Epg::ParseGraphProfileJson(profile);
    EXPECT_EQ(parsedProfile.metadata.graph, "test_graph");
    ASSERT_EQ(parsedProfile.taskCatalog.size(), 1u);
    EXPECT_EQ(parsedProfile.taskCatalog.front().taskType, "TestSourceTask");
    EXPECT_EQ(parsedProfile.taskCatalog.front().role, "source");
    EXPECT_EQ(parsedProfile.taskCatalog.front().resource, "cpu");
    EXPECT_EQ(parsedProfile.taskCatalog.front().budgetUs, 1000u);
    EXPECT_EQ(parsedProfile.taskCatalog.front().deadlineUs, 2000u);
    ASSERT_EQ(parsedProfile.topology.queues.size(), 1u);
    ASSERT_EQ(parsedProfile.diagnostics.tasks.size(), 2u);
}
