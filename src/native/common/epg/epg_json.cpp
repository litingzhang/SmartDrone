#include "common/epg/epg.h"

#include <cctype>
#include <fstream>
#include <sstream>

namespace Epg {
namespace {

class JsonValue {
  public:
    enum class Kind {
        Null,
        Bool,
        Number,
        String,
        Array,
        Object
    };

    Kind kind{Kind::Null};
    bool boolean{};
    double number{};
    std::string string;
    std::vector<JsonValue> array;
    std::map<std::string, JsonValue> object;

    const JsonValue &At(const std::string &key) const
    {
        auto it = object.find(key);
        if (it == object.end()) {
            throw std::runtime_error("missing json field: " + key);
        }
        return it->second;
    }

    const JsonValue *Find(const std::string &key) const
    {
        auto it = object.find(key);
        return it == object.end() ? nullptr : &it->second;
    }
};

class JsonParser {
  public:
    explicit JsonParser(const std::string &text)
        : m_text(text)
    {
    }

    JsonValue Parse()
    {
        auto value = ParseValue();
        SkipWhitespace();
        if (m_pos != m_text.size()) {
            Fail("unexpected trailing json content");
        }
        return value;
    }

  private:
    JsonValue ParseValue()
    {
        SkipWhitespace();
        if (m_pos >= m_text.size()) {
            Fail("unexpected end of json");
        }

        const char c = m_text[m_pos];
        if (c == '{') {
            return ParseObject();
        }
        if (c == '[') {
            return ParseArray();
        }
        if (c == '"') {
            JsonValue value;
            value.kind = JsonValue::Kind::String;
            value.string = ParseString();
            return value;
        }
        if (c == 't' || c == 'f') {
            return ParseBool();
        }
        if (c == 'n') {
            return ParseNull();
        }
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
            return ParseNumber();
        }
        Fail("invalid json value");
        return {};
    }

    JsonValue ParseObject()
    {
        JsonValue value;
        value.kind = JsonValue::Kind::Object;
        Expect('{');
        SkipWhitespace();
        if (Consume('}')) {
            return value;
        }
        for (;;) {
            SkipWhitespace();
            if (Peek() != '"') {
                Fail("expected json object key");
            }
            auto key = ParseString();
            SkipWhitespace();
            Expect(':');
            value.object.emplace(std::move(key), ParseValue());
            SkipWhitespace();
            if (Consume('}')) {
                break;
            }
            Expect(',');
        }
        return value;
    }

    JsonValue ParseArray()
    {
        JsonValue value;
        value.kind = JsonValue::Kind::Array;
        Expect('[');
        SkipWhitespace();
        if (Consume(']')) {
            return value;
        }
        for (;;) {
            value.array.push_back(ParseValue());
            SkipWhitespace();
            if (Consume(']')) {
                break;
            }
            Expect(',');
        }
        return value;
    }

    std::string ParseString()
    {
        Expect('"');
        std::string result;
        while (m_pos < m_text.size()) {
            const char c = m_text[m_pos++];
            if (c == '"') {
                return result;
            }
            if (c == '\\') {
                if (m_pos >= m_text.size()) {
                    Fail("unterminated json escape");
                }
                const char escaped = m_text[m_pos++];
                switch (escaped) {
                case '"':
                case '\\':
                case '/':
                    result.push_back(escaped);
                    break;
                case 'b':
                    result.push_back('\b');
                    break;
                case 'f':
                    result.push_back('\f');
                    break;
                case 'n':
                    result.push_back('\n');
                    break;
                case 'r':
                    result.push_back('\r');
                    break;
                case 't':
                    result.push_back('\t');
                    break;
                default:
                    Fail("unsupported json escape");
                }
            } else {
                result.push_back(c);
            }
        }
        Fail("unterminated json string");
        return {};
    }

    JsonValue ParseBool()
    {
        JsonValue value;
        value.kind = JsonValue::Kind::Bool;
        if (Match("true")) {
            value.boolean = true;
            return value;
        }
        if (Match("false")) {
            value.boolean = false;
            return value;
        }
        Fail("invalid json boolean");
        return {};
    }

    JsonValue ParseNull()
    {
        if (!Match("null")) {
            Fail("invalid json null");
        }
        return {};
    }

    JsonValue ParseNumber()
    {
        const auto start = m_pos;
        if (Peek() == '-') {
            ++m_pos;
        }
        while (m_pos < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_pos]))) {
            ++m_pos;
        }
        if (m_pos < m_text.size() && m_text[m_pos] == '.') {
            ++m_pos;
            while (m_pos < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_pos]))) {
                ++m_pos;
            }
        }
        if (m_pos < m_text.size() && (m_text[m_pos] == 'e' || m_text[m_pos] == 'E')) {
            ++m_pos;
            if (m_pos < m_text.size() && (m_text[m_pos] == '+' || m_text[m_pos] == '-')) {
                ++m_pos;
            }
            while (m_pos < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_pos]))) {
                ++m_pos;
            }
        }

        JsonValue value;
        value.kind = JsonValue::Kind::Number;
        value.number = std::stod(m_text.substr(start, m_pos - start));
        return value;
    }

    void SkipWhitespace()
    {
        while (m_pos < m_text.size() &&
               std::isspace(static_cast<unsigned char>(m_text[m_pos]))) {
            ++m_pos;
        }
    }

    char Peek() const
    {
        return m_pos < m_text.size() ? m_text[m_pos] : '\0';
    }

    bool Consume(char expected)
    {
        if (Peek() != expected) {
            return false;
        }
        ++m_pos;
        return true;
    }

    void Expect(char expected)
    {
        if (!Consume(expected)) {
            Fail(std::string("expected '") + expected + "'");
        }
    }

    bool Match(const char *literal)
    {
        const std::string expected(literal);
        if (m_text.compare(m_pos, expected.size(), expected) != 0) {
            return false;
        }
        m_pos += expected.size();
        return true;
    }

    [[noreturn]] void Fail(const std::string &message) const
    {
        throw std::runtime_error("json parse error at byte " + std::to_string(m_pos) + ": " + message);
    }

    const std::string &m_text;
    std::size_t m_pos{};
};

std::string RequiredString(const JsonValue &value, const std::string &field)
{
    const auto &child = value.At(field);
    if (child.kind != JsonValue::Kind::String) {
        throw std::runtime_error("json field must be string: " + field);
    }
    return child.string;
}

std::size_t RequiredSize(const JsonValue &value, const std::string &field)
{
    const auto &child = value.At(field);
    if (child.kind != JsonValue::Kind::Number || child.number < 0) {
        throw std::runtime_error("json field must be non-negative number: " + field);
    }
    return static_cast<std::size_t>(child.number);
}

std::chrono::milliseconds OptionalMilliseconds(const JsonValue &value,
                                               const std::string &field,
                                               std::chrono::milliseconds fallback)
{
    const auto *child = value.Find(field);
    if (!child) {
        return fallback;
    }
    if (child->kind != JsonValue::Kind::Number || child->number < 0) {
        throw std::runtime_error("json field must be non-negative number: " + field);
    }
    return std::chrono::milliseconds(static_cast<int>(child->number));
}

bool OptionalBool(const JsonValue &value, const std::string &field, bool fallback)
{
    const auto *child = value.Find(field);
    if (!child) {
        return fallback;
    }
    if (child->kind != JsonValue::Kind::Bool) {
        throw std::runtime_error("json field must be bool: " + field);
    }
    return child->boolean;
}

bool RequiredBool(const JsonValue &value, const std::string &field)
{
    const auto &child = value.At(field);
    if (child.kind != JsonValue::Kind::Bool) {
        throw std::runtime_error("json field must be bool: " + field);
    }
    return child.boolean;
}

int OptionalInt(const JsonValue &value, const std::string &field, int fallback)
{
    const auto *child = value.Find(field);
    if (!child) {
        return fallback;
    }
    if (child->kind != JsonValue::Kind::Number) {
        throw std::runtime_error("json field must be number: " + field);
    }
    return static_cast<int>(child->number);
}

std::uint64_t OptionalUInt64(const JsonValue &value,
                             const std::string &field,
                             std::uint64_t fallback)
{
    const auto *child = value.Find(field);
    if (!child) {
        return fallback;
    }
    if (child->kind != JsonValue::Kind::Number || child->number < 0) {
        throw std::runtime_error("json field must be non-negative number: " + field);
    }
    return static_cast<std::uint64_t>(child->number);
}

std::uint64_t RequiredUInt64(const JsonValue &value,
                             const std::string &field)
{
    const auto &child = value.At(field);
    if (child.kind != JsonValue::Kind::Number || child.number < 0) {
        throw std::runtime_error("json field must be non-negative number: " + field);
    }
    return static_cast<std::uint64_t>(child.number);
}

std::string OptionalString(const JsonValue &value,
                           const std::string &field,
                           const std::string &fallback)
{
    const auto *child = value.Find(field);
    if (!child) {
        return fallback;
    }
    if (child->kind != JsonValue::Kind::String) {
        throw std::runtime_error("json field must be string: " + field);
    }
    return child->string;
}

std::vector<std::string> OptionalStringArray(const JsonValue &value, const std::string &field)
{
    std::vector<std::string> result;
    const auto *child = value.Find(field);
    if (!child) {
        return result;
    }
    if (child->kind != JsonValue::Kind::Array) {
        throw std::runtime_error("json field must be string array: " + field);
    }
    for (const auto &item : child->array) {
        if (item.kind != JsonValue::Kind::String) {
            throw std::runtime_error("json array item must be string: " + field);
        }
        result.push_back(item.string);
    }
    return result;
}

std::vector<PortId> OptionalPortIdArray(const JsonValue &value,
                                        const std::string &field)
{
    std::vector<PortId> result;
    const auto *child = value.Find(field);
    if (!child) {
        return result;
    }
    if (child->kind != JsonValue::Kind::Array) {
        throw std::runtime_error("json field must be port id array: " + field);
    }
    for (const auto &item : child->array) {
        if (item.kind != JsonValue::Kind::Number || item.number < 0) {
            throw std::runtime_error("json array item must be port id: " + field);
        }
        result.push_back(static_cast<PortId>(item.number));
    }
    return result;
}

TaskSchedulingConfig OptionalScheduling(const JsonValue &value)
{
    TaskSchedulingConfig scheduling;
    const auto *child = value.Find("scheduling");
    if (!child) {
        return scheduling;
    }
    if (child->kind != JsonValue::Kind::Object) {
        throw std::runtime_error("task scheduling must be object");
    }
    scheduling.resource = OptionalString(*child, "resource", scheduling.resource);
    scheduling.cpuAffinity =
        OptionalInt(*child, "cpu_affinity", scheduling.cpuAffinity);
    scheduling.budgetUs =
        OptionalUInt64(*child, "budget_us", scheduling.budgetUs);
    scheduling.deadlineUs =
        OptionalUInt64(*child, "deadline_us", scheduling.deadlineUs);
    scheduling.backpressureOutputs =
        OptionalPortIdArray(*child, "backpressure_outputs");
    scheduling.realtime = OptionalBool(*child, "realtime", false);
    scheduling.priority = OptionalInt(*child, "priority", 0);
    return scheduling;
}

PortId ParsePortId(const std::string &value, const std::string &field)
{
    std::size_t parsedChars = 0;
    const auto parsed = std::stoul(value, &parsedChars, 10);
    if (parsedChars != value.size()) {
        throw std::runtime_error("json port id must be a non-negative integer: " + field);
    }
    return static_cast<PortId>(parsed);
}

std::map<PortId, std::string> OptionalPortQueueMap(const JsonValue &value, const std::string &field)
{
    std::map<PortId, std::string> result;
    const auto *child = value.Find(field);
    if (!child) {
        return result;
    }
    if (child->kind != JsonValue::Kind::Object) {
        throw std::runtime_error("json field must be object: " + field);
    }
    for (const auto &pair : child->object) {
        if (pair.second.kind != JsonValue::Kind::String) {
            throw std::runtime_error("json object value must be string: " + field + "." + pair.first);
        }
        result[ParsePortId(pair.first, field + "." + pair.first)] = pair.second.string;
    }
    return result;
}

OverflowPolicy ParseOverflow(const std::string &value)
{
    if (value == "drop_newest" || value == "tail_drop") {
        return OverflowPolicy::DropNewest;
    }
    if (value == "overwrite_oldest" || value == "circular_overwrite") {
        return OverflowPolicy::OverwriteOldest;
    }
    throw std::runtime_error("unsupported queue overflow policy: " + value);
}

TriggerMode ParseTriggerMode(const std::string &value)
{
    if (value == "periodic") {
        return TriggerMode::Periodic;
    }
    if (value == "any_queue_ready") {
        return TriggerMode::AnyQueueReady;
    }
    if (value == "all_queue_ready") {
        return TriggerMode::AllQueueReady;
    }
    if (value == "periodic_or_any_queue_ready") {
        return TriggerMode::PeriodicOrAnyQueueReady;
    }
    throw std::runtime_error("unsupported task trigger mode: " + value);
}

const JsonValue &RequiredArrayField(const JsonValue &value,
                                    const std::string &field)
{
    const auto &child = value.At(field);
    if (child.kind != JsonValue::Kind::Array) {
        throw std::runtime_error("json field must be array: " + field);
    }
    return child;
}

void RequireConfigObject(const JsonValue &value, const std::string &message)
{
    if (value.kind != JsonValue::Kind::Object) {
        throw std::runtime_error(message);
    }
}

QueueConfig ParseQueueConfig(const JsonValue &item)
{
    RequireConfigObject(item, "queue config must be object");
    QueueConfig queue;
    queue.name = RequiredString(item, "name");
    queue.type = RequiredString(item, "type");
    queue.depth = RequiredSize(item, "depth");
    queue.overflow = ParseOverflow(RequiredString(item, "overflow"));
    return queue;
}

TriggerConfig ParseTaskTriggerConfig(const JsonValue &item,
                                     const std::string &taskName)
{
    const auto &trigger = item.At("trigger");
    if (trigger.kind != JsonValue::Kind::Object) {
        throw std::runtime_error("task trigger must be object: " + taskName);
    }

    TriggerConfig config;
    config.mode = ParseTriggerMode(RequiredString(trigger, "mode"));
    config.interval =
        OptionalMilliseconds(trigger, "interval_ms", std::chrono::milliseconds(0));
    config.queues = OptionalStringArray(trigger, "queues");
    return config;
}

TaskConfig ParseTaskConfig(const JsonValue &item)
{
    RequireConfigObject(item, "task config must be object");
    TaskConfig task;
    task.name = RequiredString(item, "name");
    task.type = RequiredString(item, "type");
    task.inputs = OptionalPortQueueMap(item, "inputs");
    task.outputs = OptionalPortQueueMap(item, "outputs");
    task.trigger = ParseTaskTriggerConfig(item, task.name);
    task.scheduling = OptionalScheduling(item);
    return task;
}

void AppendQueueConfigs(const JsonValue &queues, GraphConfig &config)
{
    for (const auto &item : queues.array) {
        config.queues.push_back(ParseQueueConfig(item));
    }
}

void AppendTaskConfigs(const JsonValue &tasks, GraphConfig &config)
{
    for (const auto &item : tasks.array) {
        config.tasks.push_back(ParseTaskConfig(item));
    }
}

QueueProfileMetrics ParseQueueProfileMetrics(const JsonValue &item)
{
    RequireConfigObject(item, "queue profile metrics must be object");
    QueueProfileMetrics metrics;
    metrics.maxDepthObserved = RequiredUInt64(item, "maxDepthObserved");
    metrics.droppedNewest = RequiredUInt64(item, "droppedNewest");
    metrics.overwrittenOldest = RequiredUInt64(item, "overwrittenOldest");
    metrics.pushedPerSecond = RequiredUInt64(item, "pushedPerSecond");
    metrics.poppedPerSecond = RequiredUInt64(item, "poppedPerSecond");
    metrics.droppedPerSecond = RequiredUInt64(item, "droppedPerSecond");
    return metrics;
}

TaskProfileMetrics ParseTaskProfileMetrics(const JsonValue &item)
{
    RequireConfigObject(item, "task profile metrics must be object");
    TaskProfileMetrics metrics;
    metrics.maxLoopUs = RequiredUInt64(item, "maxLoopUs");
    metrics.averageLoopUs = RequiredUInt64(item, "averageLoopUs");
    metrics.p90LoopUs = RequiredUInt64(item, "p90LoopUs");
    metrics.p99LoopUs = RequiredUInt64(item, "p99LoopUs");
    metrics.resourceWaitCount =
        OptionalUInt64(item, "resourceWaitCount", metrics.resourceWaitCount);
    metrics.maxResourceWaitUs =
        OptionalUInt64(item, "maxResourceWaitUs", metrics.maxResourceWaitUs);
    metrics.averageResourceWaitUs = OptionalUInt64(
        item, "averageResourceWaitUs", metrics.averageResourceWaitUs);
    metrics.totalResourceWaitUs = OptionalUInt64(
        item, "totalResourceWaitUs", metrics.totalResourceWaitUs);
    metrics.utilizationPpm = RequiredUInt64(item, "utilizationPpm");
    metrics.budgetOverrunCount = RequiredUInt64(item, "budgetOverrunCount");
    metrics.deadlineMissCount = RequiredUInt64(item, "deadlineMissCount");
    metrics.schedulingErrorCount =
        RequiredUInt64(item, "schedulingErrorCount");
    return metrics;
}

GraphProfileTaskCatalogEntry ParseTaskCatalogEntry(const JsonValue &item)
{
    RequireConfigObject(item, "task catalog entry must be object");
    GraphProfileTaskCatalogEntry entry;
    entry.taskType = RequiredString(item, "taskType");
    entry.role = RequiredString(item, "role");
    entry.resource = RequiredString(item, "resource");
    entry.budgetUs = RequiredUInt64(item, "budgetUs");
    entry.deadlineUs = RequiredUInt64(item, "deadlineUs");
    entry.replaceable = OptionalBool(item, "replaceable", entry.replaceable);
    return entry;
}

std::vector<GraphProfileTaskCatalogEntry> ParseTaskCatalogObject(
    const JsonValue &root)
{
    const auto *catalog = root.Find("taskCatalog");
    if (!catalog) {
        throw std::runtime_error("missing EventPipelineGraph json field: taskCatalog");
    }
    if (catalog->kind != JsonValue::Kind::Array) {
        throw std::runtime_error("json field must be array: taskCatalog");
    }
    std::vector<GraphProfileTaskCatalogEntry> entries;
    entries.reserve(catalog->array.size());
    for (const auto &item : catalog->array) {
        entries.push_back(ParseTaskCatalogEntry(item));
    }
    return entries;
}

const JsonValue *FindObjectField(const JsonValue &value,
                                 const std::string &field)
{
    const auto *child = value.Find(field);
    if (!child || child->kind != JsonValue::Kind::Object) {
        return nullptr;
    }
    return child;
}

GraphConfig ParseGraphConfigObject(const JsonValue &root)
{
    RequireConfigObject(root, "EventPipelineGraph json root must be object");

    GraphConfig config;
    AppendQueueConfigs(RequiredArrayField(root, "queues"), config);
    AppendTaskConfigs(RequiredArrayField(root, "tasks"), config);
    return config;
}

GraphProfileMetadata ParseGraphProfileMetadataObject(const JsonValue &root)
{
    RequireConfigObject(root, "EventPipelineGraph profile root must be object");

    GraphProfileMetadata metadata;
    metadata.schema = RequiredString(root, "schema");
    metadata.graph = RequiredString(root, "graph");
    metadata.topologyVersion =
        OptionalString(root, "topologyVersion", metadata.topologyVersion);
    metadata.timestampMs =
        OptionalUInt64(root, "timestampMs", metadata.timestampMs);
    return metadata;
}

GraphProfileDiagnostics ParseGraphProfileDiagnosticsObject(
    const JsonValue &root)
{
    RequireConfigObject(root, "EventPipelineGraph profile root must be object");

    GraphProfileDiagnostics diagnostics;
    const auto *diag = FindObjectField(root, "diagnostics");
    if (!diag) {
        return diagnostics;
    }
    if (const auto *queues = FindObjectField(*diag, "queues")) {
        for (const auto &item : queues->object) {
            diagnostics.queues[item.first] =
                ParseQueueProfileMetrics(item.second);
        }
    }
    if (const auto *tasks = FindObjectField(*diag, "tasks")) {
        for (const auto &item : tasks->object) {
            diagnostics.tasks[item.first] =
                ParseTaskProfileMetrics(item.second);
        }
    }
    return diagnostics;
}

OptimizedGraphMetadata ParseOptimizedGraphMetadataObject(
    const JsonValue &root)
{
    RequireConfigObject(root, "optimized graph json root must be object");

    OptimizedGraphMetadata metadata;
    metadata.schema = RequiredString(root, "schema");
    metadata.targetGraph = RequiredString(root, "targetGraph");
    metadata.topologyVersion = RequiredString(root, "topologyVersion");
    metadata.solverVersion = RequiredString(root, "solverVersion");
    metadata.sourceProfile = RequiredString(root, "sourceProfile");
    metadata.sourceTimestampMs = RequiredUInt64(root, "sourceTimestampMs");
    metadata.generatedAtMs = RequiredUInt64(root, "generatedAtMs");
    return metadata;
}

SolverReportMetadata ParseSolverReportMetadataObject(
    const JsonValue &root)
{
    RequireConfigObject(root, "solver report json root must be object");

    SolverReportMetadata metadata;
    metadata.schema = RequiredString(root, "schema");
    metadata.targetGraph = RequiredString(root, "targetGraph");
    metadata.topologyVersion = RequiredString(root, "topologyVersion");
    metadata.sourceProfile = RequiredString(root, "sourceProfile");
    metadata.solverVersion = RequiredString(root, "solverVersion");
    metadata.sourceTimestampMs = RequiredUInt64(root, "sourceTimestampMs");
    metadata.generatedAtMs = RequiredUInt64(root, "generatedAtMs");
    return metadata;
}

SolverReportScore ParseSolverReportScoreObject(const JsonValue &root)
{
    RequireConfigObject(root, "solver report score must be object");
    SolverReportScore score;
    score.queuePressure = RequiredUInt64(root, "queuePressure");
    score.periodicOverloadUs = RequiredUInt64(root, "periodicOverloadUs");
    score.resourceWaitUs =
        OptionalUInt64(root, "resourceWaitUs", score.resourceWaitUs);
    score.schedulingErrors = RequiredUInt64(root, "schedulingErrors");
    score.budgetOverruns = RequiredUInt64(root, "budgetOverruns");
    score.deadlineMisses = RequiredUInt64(root, "deadlineMisses");
    score.utilizationOverPpm = RequiredUInt64(root, "utilizationOverPpm");
    score.totalPenalty = RequiredUInt64(root, "totalPenalty");
    return score;
}

SolverReportConstraints ParseSolverReportConstraintsObject(
    const JsonValue &root)
{
    RequireConfigObject(root, "solver report constraints must be object");
    SolverReportConstraints constraints;
    constraints.maxQueueDepth = RequiredUInt64(root, "maxQueueDepth");
    constraints.maxPeriodicIntervalMs =
        RequiredUInt64(root, "maxPeriodicIntervalMs");
    constraints.targetUtilizationPpm =
        RequiredUInt64(root, "targetUtilizationPpm");
    return constraints;
}

std::string ParseSolverReportObjectiveName(const JsonValue &root)
{
    const auto &objective = root.At("objective");
    RequireConfigObject(objective, "solver report objective must be object");
    return RequiredString(objective, "name");
}

SolverReportScore ParseSolverReportObjectiveScore(const JsonValue &root)
{
    const auto &objective = root.At("objective");
    RequireConfigObject(objective, "solver report objective must be object");
    const auto &score = objective.At("score");
    return ParseSolverReportScoreObject(score);
}

void ParseQueueSolverReportDecision(const JsonValue &item,
                                    SolverReportDecision &decision)
{
    decision.depthBefore = RequiredUInt64(item, "depthBefore");
    decision.depthAfter = RequiredUInt64(item, "depthAfter");
    decision.pressureBefore = RequiredUInt64(item, "pressureBefore");
    decision.pushedPerSecond = RequiredUInt64(item, "pushedPerSecond");
    decision.poppedPerSecond = RequiredUInt64(item, "poppedPerSecond");
    decision.droppedPerSecond = RequiredUInt64(item, "droppedPerSecond");
}

void ParseTaskSolverReportDecision(const JsonValue &item,
                                   SolverReportDecision &decision)
{
    decision.intervalBeforeMs = RequiredUInt64(item, "intervalBeforeMs");
    decision.intervalAfterMs = RequiredUInt64(item, "intervalAfterMs");
    decision.maxLoopUs = RequiredUInt64(item, "maxLoopUs");
    decision.averageLoopUs = RequiredUInt64(item, "averageLoopUs");
    decision.p90LoopUs = RequiredUInt64(item, "p90LoopUs");
    decision.p99LoopUs = RequiredUInt64(item, "p99LoopUs");
    decision.effectiveLoopUs = RequiredUInt64(item, "effectiveLoopUs");
    decision.resourceWaitCount =
        OptionalUInt64(item, "resourceWaitCount", decision.resourceWaitCount);
    decision.maxResourceWaitUs = OptionalUInt64(
        item, "maxResourceWaitUs", decision.maxResourceWaitUs);
    decision.averageResourceWaitUs = OptionalUInt64(
        item, "averageResourceWaitUs", decision.averageResourceWaitUs);
    decision.totalResourceWaitUs = OptionalUInt64(
        item, "totalResourceWaitUs", decision.totalResourceWaitUs);
    decision.utilizationPpm = RequiredUInt64(item, "utilizationPpm");
    decision.targetUtilizationPpm =
        RequiredUInt64(item, "targetUtilizationPpm");
    decision.budgetUs = RequiredUInt64(item, "budgetUs");
    decision.deadlineUs = RequiredUInt64(item, "deadlineUs");
    decision.catalogRole = RequiredString(item, "catalogRole");
    decision.replaceable = RequiredBool(item, "replaceable");
    decision.budgetOverrunCount = RequiredUInt64(item, "budgetOverrunCount");
    decision.deadlineMissCount = RequiredUInt64(item, "deadlineMissCount");
    decision.schedulingErrorCount =
        RequiredUInt64(item, "schedulingErrorCount");
}

SolverReportDecision ParseSolverReportDecision(const JsonValue &item)
{
    RequireConfigObject(item, "solver report decision must be object");
    SolverReportDecision decision;
    decision.kind = RequiredString(item, "kind");
    decision.name = RequiredString(item, "name");
    decision.reason = RequiredString(item, "reason");
    if (decision.kind == "queue") {
        ParseQueueSolverReportDecision(item, decision);
        return decision;
    }
    if (decision.kind == "task") {
        ParseTaskSolverReportDecision(item, decision);
        return decision;
    }
    throw std::runtime_error("unsupported solver report decision kind: " +
                             decision.kind);
}

std::vector<SolverReportDecision> ParseSolverReportDecisionsObject(
    const JsonValue &root)
{
    const auto &decisions = RequiredArrayField(root, "decisions");
    std::vector<SolverReportDecision> result;
    result.reserve(decisions.array.size());
    for (const auto &item : decisions.array) {
        result.push_back(ParseSolverReportDecision(item));
    }
    return result;
}

SolverReport ParseSolverReportObject(const JsonValue &root)
{
    RequireConfigObject(root, "solver report json root must be object");
    return {
        ParseSolverReportMetadataObject(root),
        ParseSolverReportObjectiveName(root),
        ParseSolverReportObjectiveScore(root),
        ParseSolverReportConstraintsObject(root.At("constraints")),
        ParseSolverReportDecisionsObject(root),
    };
}

} // namespace

GraphProfile ParseGraphProfileJson(const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    RequireConfigObject(root, "EventPipelineGraph profile root must be object");
    const auto *topology = root.Find("topology");
    if (!topology) {
        throw std::runtime_error("missing EventPipelineGraph json field: topology");
    }
    return {
        ParseGraphProfileMetadataObject(root),
        ParseTaskCatalogObject(root),
        ParseGraphConfigObject(*topology),
        ParseGraphProfileDiagnosticsObject(root),
    };
}

GraphProfileMetadata ParseGraphProfileMetadataJson(
    const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return ParseGraphProfileMetadataObject(root);
}

GraphProfileDiagnostics ParseGraphProfileDiagnosticsJson(
    const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return ParseGraphProfileDiagnosticsObject(root);
}

OptimizedGraphMetadata ParseOptimizedGraphMetadataJson(
    const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return ParseOptimizedGraphMetadataObject(root);
}

OptimizedGraph ParseOptimizedGraphJson(const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return {
        ParseOptimizedGraphMetadataObject(root),
        ParseGraphConfigObject(root),
    };
}

SolverReportMetadata ParseSolverReportMetadataJson(
    const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return ParseSolverReportMetadataObject(root);
}

SolverReport ParseSolverReportJson(const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return ParseSolverReportObject(root);
}

GraphConfig ParseGraphConfigJson(const std::string &jsonText)
{
    const auto root = JsonParser(jsonText).Parse();
    return ParseGraphConfigObject(root);
}

GraphConfig ParseGraphConfigJsonField(const std::string &jsonText,
                                      const std::string &field)
{
    const auto root = JsonParser(jsonText).Parse();
    RequireConfigObject(root, "EventPipelineGraph json root must be object");
    const auto *child = root.Find(field);
    if (!child) {
        throw std::runtime_error("missing EventPipelineGraph json field: " +
                                 field);
    }
    return ParseGraphConfigObject(*child);
}

GraphConfig ParseGraphConfigJsonFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open EventPipelineGraph json file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ParseGraphConfigJson(buffer.str());
}

} // namespace Epg
