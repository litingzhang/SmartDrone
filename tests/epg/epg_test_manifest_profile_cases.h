TEST(EventPipelineGraphManifest, RejectsOptimizedGraphMismatch)
{
    auto manifest = MakeValidTestManifest();
    manifest.topologyVersion = "test-topology";
    const auto optimized =
        Epg::ParseOptimizedGraphJson(MANIFEST_OPTIMIZED_GRAPH_JSON);
    const auto report = Epg::ParseSolverReportJson(MANIFEST_SOLVER_REPORT_JSON);
    const auto sourceProfile = MakeManifestSourceProfile(optimized);

    ExpectManifestProfileHappyPath(manifest, optimized, sourceProfile, report);
    ExpectNonOptimalReportRejected(manifest, sourceProfile, optimized, report);
    ExpectOptimizedGraphManifestRejections(manifest, optimized);
    ExpectOptimizedGraphConfigRejections(manifest, optimized);
    ExpectSolverReportMetadataRejections(manifest, optimized, report);
    ExpectSolverReportDecisionRejections(
        manifest, sourceProfile, optimized, report);
    ExpectSolverReportScoreRejections(
        manifest, sourceProfile, optimized, report);
    ExpectSolverReportDetailRejections(
        manifest, sourceProfile, optimized, report);
    ExpectSolverReportCatalogRejections(
        manifest, sourceProfile, optimized, report);
    ExpectSourceProfileRejections(manifest, sourceProfile, optimized, report);
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
    ExpectProfileJsonTextFields(profile);
    ExpectProfileJsonTopologyText(profile);
    ExpectProfileJsonDiagnosticText(profile);
    ExpectProfileJsonResourceText(profile);
    ExpectParsedProfileJson(profile);
    ExpectParsedProfileDetails(profile);
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
