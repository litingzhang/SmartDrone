TEST(EventPipelineGraphManifest, BuildsCentralizedArtifactPaths)
{
    using SmartDrone::Core::Application::BuildEpgTaskArtifactPaths;

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

TEST(EventPipelineGraphManifest, RuntimeManifestsUseCentralizedArtifacts)
{
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
        BuildEpgTaskTopologyVersion({system.topologyPath, "v3"}));
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

TEST(EventPipelineGraphManifest, ValidatesRuntimeTuningManifest)
{
    auto manifest = MakeValidTestManifest();
    manifest.runtimeTuning.push_back({"source", true, false, false});
    Epg::GraphConfig config;
    config.tasks.push_back({"source", "TestSourceTask"});

    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"source", true, false, false}}));
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"source", false, true, false}}),
        std::runtime_error);
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskRuntimeTuning(
            manifest, config, {{"missing", true, false, false}}),
        std::runtime_error);

    auto duplicate = manifest;
    duplicate.runtimeTuning.push_back({"source", false, true, false});
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskManifest(duplicate),
        std::runtime_error);

    auto incomplete = manifest;
    incomplete.runtimeTuning.push_back({"", false, false, false});
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskManifest(incomplete),
        std::runtime_error);

    auto missingGraphTask = manifest;
    missingGraphTask.runtimeTuning.push_back({"missing", true, false, false});
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskGraphManifest(
            missingGraphTask, config),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsDuplicateCatalogTaskTypes)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog.push_back(
        {"TestSourceTask", "duplicate", "cpu", 1000, 2000, false});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsAliasTargetOutsideCatalog)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"LegacySourceTask", "MissingTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsDuplicateAliases)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"LegacySourceTask", "TestSourceTask"});
    manifest.aliases.push_back({"LegacySourceTask", "TestSourceTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsAliasShadowingCatalogType)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"TestSourceTask", "TestSourceTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteAliasMetadata)
{
    auto manifest = MakeValidTestManifest();
    manifest.aliases.push_back({"", "TestSourceTask"});

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsEmptyCatalog)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog.clear();

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteCatalogMetadata)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog[0].role.clear();

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsInvalidCatalogTiming)
{
    auto manifest = MakeValidTestManifest();
    manifest.catalog[0].budgetUs = 2000;
    manifest.catalog[0].deadlineUs = 1000;

    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
            manifest, TestSourceFactoryResolver),
        std::runtime_error);
}

TEST(EventPipelineGraphManifest, RejectsIncompleteManifestMetadata)
{
    {
        auto manifest = MakeValidTestManifest();
        manifest.subgraphName.clear();

        EXPECT_THROW(
            SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
    {
        auto manifest = MakeValidTestManifest();
        manifest.topologyVersion.clear();

        EXPECT_THROW(
            SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
    {
        auto manifest = MakeValidTestManifest();
        manifest.artifactPaths.profilePath.clear();

        EXPECT_THROW(
            SmartDrone::Core::Application::ValidateEpgTaskFactoryManifest(
                manifest, TestSourceFactoryResolver),
            std::runtime_error);
    }
}

TEST(EventPipelineGraphManifest, RejectsOptimizedGraphMismatch)
{
    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    const auto optimized = Epg::ParseOptimizedGraphJson(R"({
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
        SmartDrone::Core::Application::ValidateEpgOptimizedGraphManifest(
            manifest, optimized));
    const auto report = Epg::ParseSolverReportJson(R"({
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
          "utilizationPpm": 900000,
          "targetUtilizationPpm": 800000,
          "budgetUs": 1000,
          "deadlineUs": 2000,
          "catalogRole": "source",
          "replaceable": false,
          "budgetOverrunCount": 2,
          "deadlineMissCount": 3,
          "schedulingErrorCount": 1,
          "reason": "not_replaceable+utilization_over_target+budget_overrun+deadline_miss+scheduling_error"
        }
      ]
    })");
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, optimized, report));
    auto sourceProfile = Epg::GraphProfile{};
    sourceProfile.metadata.schema = Epg::GRAPH_PROFILE_SCHEMA;
    sourceProfile.metadata.graph = "test_graph";
    sourceProfile.metadata.topologyVersion = "test-topology";
    sourceProfile.metadata.timestampMs = 123;
    sourceProfile.topology = optimized.config;
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
    EXPECT_NO_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, report));

    auto nonOptimalReport = report;
    nonOptimalReport.decisions[0].depthAfter = 5;
    nonOptimalReport.decisions[0].pressureAfter = 2;
    nonOptimalReport.decisions[0].reason = "global_optimum_depth";
    nonOptimalReport.score.totalPenalty = 131500;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, nonOptimalReport),
        std::runtime_error);

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
    wrongConstraint.constraints.maxQueueDepth = 1;
    EXPECT_THROW(
        SmartDrone::Core::Application::ValidateEpgSolverReport(
            manifest, sourceProfile, optimized, wrongConstraint),
        std::runtime_error);

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

TEST(EventPipelineGraph, ProfileJsonIncludesTopologyAndDiagnostics)
{
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
    EXPECT_THROW(
        Epg::ParseGraphProfileJson("{\"schema\":\"smartdrone.epg.profile.v1\",\"graph\":\"missing\"}"),
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
        "tasks": {"source": )") +
                         MinimalTaskDiagnosticsJson() +
                         R"(}
      }
    })";

    try {
        (void)Epg::ParseGraphProfileDiagnosticsJson(profile);
        FAIL() << "expected missing diagnostic metric to be rejected";
    } catch (const std::runtime_error &error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("missing json field: droppedNewest"),
                  std::string::npos);
    }
}
