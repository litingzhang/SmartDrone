#include "common/epg/epg.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "common/numeric_parse.h"
#include "common/epg/epg_trigger_modes.h"

namespace Epg {
namespace {

constexpr char DOT_RECORD_OPEN = '{';
constexpr char DOT_RECORD_CLOSE = '}';
constexpr char DOT_QUOTE = '"';
constexpr char DOT_ATTRIBUTE_OPEN = '[';
constexpr char DOT_ATTRIBUTE_CLOSE = ']';
constexpr char DOT_STATEMENT_END = ';';
constexpr char DOT_FIELD_SEPARATOR = ',';

std::string Trim(const std::string &value)
{
    auto begin = value.begin();
    while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }
    auto end = value.end();
    while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    return std::string(begin, end);
}

std::string StripQuotes(std::string value)
{
    value = Trim(value);
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
    }
    return value;
}

std::size_t ParseSize(const std::string &value, const std::string &field)
{
    std::size_t parsed = 0;
    if (!SmartDrone::Common::TryParseSizeFull(value.c_str(), parsed)) {
        throw std::runtime_error("DOT numeric field is invalid: " + field + "=" + value);
    }
    return parsed;
}

PortId ParsePortId(const std::string &value, const std::string &field)
{
    return static_cast<PortId>(ParseSize(value, field));
}

bool ParseBool(const std::string &value, const std::string &field)
{
    if (value == "true" || value == "1" || value == "yes") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no") {
        return false;
    }
    throw std::runtime_error("DOT bool field is invalid: " + field + "=" + value);
}

int ParseInt(const std::string &value, const std::string &field)
{
    int parsed = 0;
    if (!SmartDrone::Common::TryParseIntFull(value.c_str(), 10, parsed)) {
        throw std::runtime_error("DOT integer field is invalid: " + field + "=" + value);
    }
    return parsed;
}

OverflowPolicy ParseDotOverflow(const std::string &value)
{
    if (value == "drop_newest" || value == "tail_drop") {
        return OverflowPolicy::DropNewest;
    }
    if (value == "overwrite_oldest" || value == "circular_overwrite") {
        return OverflowPolicy::OverwriteOldest;
    }
    throw std::runtime_error("unsupported DOT queue overflow policy: " + value);
}

TriggerMode ParseDotTriggerMode(const std::string &value)
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
    if (value == "periodic_or_external") {
        return TriggerMode::PeriodicOrExternal;
    }
    throw std::runtime_error("unsupported DOT task trigger mode: " + value);
}

std::vector<std::string> Split(const std::string &text, char delimiter)
{
    std::vector<std::string> result;
    std::string current;
    for (char c : text) {
        if (c == delimiter) {
            auto item = Trim(current);
            if (!item.empty()) {
                result.push_back(item);
            }
            current.clear();
            continue;
        }
        current.push_back(c);
    }
    auto item = Trim(current);
    if (!item.empty()) {
        result.push_back(item);
    }
    return result;
}

std::vector<std::string> SplitTriggerQueueRefs(const std::string &text)
{
    return Split(text, '+');
}

std::map<std::string, std::string> ParseFields(const std::vector<std::string> &fields)
{
    std::map<std::string, std::string> result;
    for (const auto &field : fields) {
        const auto pos = field.find('=');
        if (pos == std::string::npos) {
            continue;
        }
        auto key = Trim(field.substr(0, pos));
        auto value = StripQuotes(field.substr(pos + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error("DOT field key and value must be non-empty: " + field);
        }
        result[std::move(key)] = std::move(value);
    }
    return result;
}

std::string RequireField(const std::map<std::string, std::string> &fields,
                         const std::string &key,
                         const std::string &owner)
{
    const auto it = fields.find(key);
    if (it == fields.end()) {
        throw std::runtime_error("missing DOT field '" + key + "' on " + owner);
    }
    return it->second;
}

std::vector<std::string> ParseRecordLabelFields(std::string label)
{
    label = StripQuotes(std::move(label));
    if (label.size() >= 2 && label.front() == DOT_RECORD_OPEN &&
        label.back() == DOT_RECORD_CLOSE) {
        label = label.substr(1, label.size() - 2);
    }
    return Split(label, '|');
}

std::string UnescapeDotLabel(std::string value)
{
    std::string result;
    result.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '\\' && i + 1 < value.size()) {
            const char next = value[++i];
            if (next == '\\' && i + 1 < value.size()) {
                const char escapedNext = value[++i];
                if (escapedNext == 'n' || escapedNext == 'N' ||
                    escapedNext == 'l' || escapedNext == 'r') {
                    result.push_back('\n');
                } else {
                    result.push_back(escapedNext);
                }
                continue;
            }
            if (next == 'n' || next == 'N' || next == 'l' || next == 'r') {
                result.push_back('\n');
            } else {
                result.push_back(next);
            }
            continue;
        }
        result.push_back(value[i]);
    }
    return result;
}

std::map<std::string, std::string> ParseAttributeBlock(const std::string &text)
{
    std::map<std::string, std::string> attrs;
    std::string current;
    bool inQuote = false;
    int nestedBraces = 0;
    auto flush = [&]() {
        auto field = Trim(current);
        current.clear();
        if (field.empty()) {
            return;
        }
        const auto pos = field.find('=');
        if (pos == std::string::npos) {
            return;
        }
        auto key = Trim(field.substr(0, pos));
        auto value = StripQuotes(field.substr(pos + 1));
        attrs[std::move(key)] = std::move(value);
    };

    for (char c : text) {
        if (c == DOT_QUOTE) {
            inQuote = !inQuote;
            current.push_back(c);
            continue;
        }
        if (inQuote) {
            if (c == DOT_RECORD_OPEN) {
                ++nestedBraces;
            } else if (c == DOT_RECORD_CLOSE && nestedBraces > 0) {
                --nestedBraces;
            }
        }
        if (!inQuote && c == DOT_FIELD_SEPARATOR) {
            flush();
            continue;
        }
        current.push_back(c);
    }
    flush();
    return attrs;
}

std::string SanitizeQueueName(std::string value)
{
    for (auto &c : value) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            c = '_';
        }
    }
    return value;
}

std::string AutoQueueName(const std::string &fromNode,
                          PortId fromPort,
                          const std::string &toNode,
                          PortId toPort)
{
    return SanitizeQueueName(fromNode + "_" + std::to_string(fromPort) +
                             "_to_" + toNode + "_" + std::to_string(toPort));
}

std::vector<std::string> ResolveTriggerQueues(
    const TaskConfig &task,
    const std::map<std::string, QueueConfig> &queueByName)
{
    if (!IsQueueTriggeredMode(task.trigger.mode)) {
        return task.trigger.queues;
    }
    if (task.trigger.queues.empty()) {
        std::vector<std::string> queues;
        for (const auto &input : task.inputs) {
            queues.push_back(input.second);
        }
        return queues;
    }

    std::vector<std::string> queues;
    std::set<std::string> seen;
    for (const auto &ref : task.trigger.queues) {
        const auto queueIt = queueByName.find(ref);
        if (queueIt != queueByName.end()) {
            if (seen.insert(ref).second) {
                queues.push_back(ref);
            }
            continue;
        }

        bool matched = false;
        for (const auto &input : task.inputs) {
            const auto inputQueueIt = queueByName.find(input.second);
            if (inputQueueIt == queueByName.end()) {
                continue;
            }
            if (inputQueueIt->second.type != ref) {
                continue;
            }
            matched = true;
            if (seen.insert(input.second).second) {
                queues.push_back(input.second);
            }
        }
        if (!matched) {
            throw std::runtime_error("DOT trigger_queues references non-input queue or type: " +
                                     task.name + " -> " + ref);
        }
    }
    return queues;
}

struct DotStatement {
    int depth{};
    std::string text;
};

struct StatementCollector {
    std::vector<DotStatement> statements;
    std::string current;
    int depth = 0;
    int statementDepth = 0;
    bool inQuote = false;
    bool inAttributes = false;
};

void FlushCollectedStatement(StatementCollector &collector)
{
    auto statement = Trim(collector.current);
    if (!statement.empty()) {
        collector.statements.push_back(DotStatement{collector.statementDepth, std::move(statement)});
    }
    collector.current.clear();
}

bool IsDotLineComment(const std::string &dotText,
                      std::size_t index,
                      const StatementCollector &collector)
{
    if (collector.inQuote) {
        return false;
    }
    if (dotText[index] == '#') {
        return true;
    }
    return dotText[index] == '/' && index + 1 < dotText.size() && dotText[index + 1] == '/';
}

void SkipDotLineComment(const std::string &dotText, std::size_t &index)
{
    while (index < dotText.size() && dotText[index] != '\n') {
        ++index;
    }
}

void HandleStatementBrace(StatementCollector &collector, char c)
{
    if (c == DOT_RECORD_OPEN) {
        ++collector.depth;
        collector.current.clear();
        collector.statementDepth = collector.depth;
        return;
    }

    FlushCollectedStatement(collector);
    --collector.depth;
    collector.statementDepth = collector.depth;
}

bool TryFinishStatement(StatementCollector &collector, char c)
{
    if (!collector.inQuote && c == DOT_ATTRIBUTE_OPEN) {
        collector.inAttributes = true;
        return false;
    }
    if (!collector.inQuote && c == DOT_ATTRIBUTE_CLOSE) {
        collector.inAttributes = false;
        return false;
    }
    if (collector.inQuote || collector.inAttributes ||
        c != DOT_STATEMENT_END) {
        return false;
    }
    FlushCollectedStatement(collector);
    return true;
}

void AppendStatementChar(StatementCollector &collector, char c)
{
    if (collector.current.empty() && !std::isspace(static_cast<unsigned char>(c))) {
        collector.statementDepth = collector.depth;
    }
    collector.current.push_back(c);
}

std::vector<DotStatement> CollectStatements(const std::string &dotText)
{
    StatementCollector collector;
    for (std::size_t i = 0; i < dotText.size(); ++i) {
        const char c = dotText[i];
        if (c == DOT_QUOTE) {
            collector.inQuote = !collector.inQuote;
        }
        if (IsDotLineComment(dotText, i, collector)) {
            SkipDotLineComment(dotText, i);
            continue;
        }
        if (!collector.inQuote &&
            (c == DOT_RECORD_OPEN || c == DOT_RECORD_CLOSE)) {
            HandleStatementBrace(collector, c);
            continue;
        }
        if (TryFinishStatement(collector, c)) {
            continue;
        }
        AppendStatementChar(collector, c);
    }
    FlushCollectedStatement(collector);
    return collector.statements;
}

std::size_t FindSubgraphOpenBrace(const std::string &dotText,
                                  const std::string &subgraphName)
{
    const std::string needle = "subgraph " + subgraphName;
    const auto begin = dotText.find(needle);
    if (begin == std::string::npos) {
        throw std::runtime_error("DOT subgraph not found: " + subgraphName);
    }
    const auto open = dotText.find(DOT_RECORD_OPEN, begin + needle.size());
    if (open == std::string::npos) {
        throw std::runtime_error("DOT subgraph missing body: " + subgraphName);
    }
    return open;
}

std::size_t FindMatchingBrace(const std::string &dotText,
                              std::size_t open,
                              const std::string &subgraphName)
{
    bool inQuote = false;
    int depth = 0;
    for (std::size_t i = open; i < dotText.size(); ++i) {
        const char c = dotText[i];
        if (c == DOT_QUOTE) {
            inQuote = !inQuote;
            continue;
        }
        if (inQuote) {
            continue;
        }
        if (c == DOT_RECORD_OPEN) {
            ++depth;
            continue;
        }
        if (c == DOT_RECORD_CLOSE) {
            --depth;
            if (depth == 0) {
                return i;
            }
        }
    }
    throw std::runtime_error("DOT subgraph not closed: " + subgraphName);
}

std::string ExtractSubgraphBody(const std::string &dotText, const std::string &subgraphName)
{
    const auto open = FindSubgraphOpenBrace(dotText, subgraphName);
    const auto close = FindMatchingBrace(dotText, open, subgraphName);
    return dotText.substr(open + 1, close - open - 1);
}

bool SplitStatementAndAttributes(const std::string &statement,
                                 std::string &head,
                                 std::map<std::string, std::string> &attrs)
{
    const auto open = statement.find('[');
    const auto close = statement.rfind(']');
    if (open == std::string::npos || close == std::string::npos || close <= open) {
        return false;
    }
    head = Trim(statement.substr(0, open));
    attrs = ParseAttributeBlock(statement.substr(open + 1, close - open - 1));
    return true;
}

void ApplyTaskTimingFields(TaskConfig &task,
                           const std::map<std::string, std::string> &fields);
void ApplyTaskSchedulingFields(
    TaskConfig &task, const std::map<std::string, std::string> &fields);

TaskConfig ParseTaskStatement(const std::string &head,
                              const std::map<std::string, std::string> &attrs)
{
    const auto labelIt = attrs.find("label");
    if (labelIt == attrs.end()) {
        throw std::runtime_error("DOT task node missing label: " + head);
    }
    const auto fields = ParseFields(ParseRecordLabelFields(labelIt->second));

    TaskConfig task;
    task.name = head;
    task.type = RequireField(fields, "type", task.name);
    task.trigger.mode = ParseDotTriggerMode(RequireField(fields, "trigger", task.name));
    ApplyTaskTimingFields(task, fields);
    ApplyTaskSchedulingFields(task, fields);
    return task;
}

void ApplyTaskTimingFields(TaskConfig &task,
                           const std::map<std::string, std::string> &fields)
{
    const auto intervalIt = fields.find("interval_ms");
    if (intervalIt != fields.end()) {
        task.trigger.interval = std::chrono::milliseconds(
            static_cast<int>(ParseSize(intervalIt->second, "interval_ms")));
    }
    const auto triggerQueuesIt = fields.find("trigger_queues");
    if (triggerQueuesIt != fields.end()) {
        task.trigger.queues = SplitTriggerQueueRefs(triggerQueuesIt->second);
    }
}

void ApplyTaskSchedulingFields(TaskConfig &task,
                               const std::map<std::string, std::string> &fields)
{
    const auto realtimeIt = fields.find("realtime");
    if (realtimeIt != fields.end()) {
        task.scheduling.realtime = ParseBool(realtimeIt->second, "realtime");
    }
    const auto priorityIt = fields.find("priority");
    if (priorityIt != fields.end()) {
        task.scheduling.priority = static_cast<int>(ParseSize(priorityIt->second, "priority"));
    }
    const auto resourceIt = fields.find("resource");
    if (resourceIt != fields.end()) {
        task.scheduling.resource = resourceIt->second;
    }
    const auto cpuAffinityIt = fields.find("cpu_affinity");
    if (cpuAffinityIt != fields.end()) {
        task.scheduling.cpuAffinity = ParseInt(cpuAffinityIt->second, "cpu_affinity");
    }
    const auto budgetIt = fields.find("budget_us");
    if (budgetIt != fields.end()) {
        task.scheduling.budgetUs = ParseSize(budgetIt->second, "budget_us");
    }
    const auto deadlineIt = fields.find("deadline_us");
    if (deadlineIt != fields.end()) {
        task.scheduling.deadlineUs = ParseSize(deadlineIt->second, "deadline_us");
    }
    const auto topologyLevelIt = fields.find("topology_level");
    if (topologyLevelIt != fields.end()) {
        task.scheduling.topologyLevel =
            ParseSize(topologyLevelIt->second, "topology_level");
    }
    const auto phaseOffsetIt = fields.find("phase_offset_ms");
    if (phaseOffsetIt != fields.end()) {
        task.scheduling.phaseOffsetMs =
            ParseSize(phaseOffsetIt->second, "phase_offset_ms");
        task.scheduling.phaseOffsetConfigured = true;
    }
    const auto backpressureIt = fields.find("backpressure_outputs");
    if (backpressureIt != fields.end()) {
        for (const auto &port : Split(backpressureIt->second, '+')) {
            task.scheduling.backpressureOutputs.push_back(
                ParsePortId(port, "backpressure_outputs"));
        }
    }
}

void MergePortSpec(std::map<std::string, std::vector<PortSpec>> &byTask,
                   const std::string &taskType,
                   PortId port,
                   const std::string &queueType)
{
    byTask[taskType].push_back(PortSpec{port, queueType});
}

struct DotParseContext {
    GraphConfig config;
    std::map<std::string, std::size_t> taskIndexByName;
    std::map<std::string, QueueConfig> queueByName;
    std::set<std::string> queueNames;
    std::map<std::string, std::vector<PortSpec>> graphInputsByTaskType;
    std::map<std::string, std::vector<PortSpec>> graphOutputsByTaskType;
    Registry &registry;
};

struct EdgeEndpoints {
    std::string fromNode;
    std::string toNode;
    std::size_t fromTask = 0;
    std::size_t toTask = 0;
    PortId fromPort = 0;
    PortId toPort = 0;
};

bool IsEdgeStatement(const std::string &head)
{
    return head.find("->") != std::string::npos;
}

bool IsGraphDefaultStatement(const std::string &head)
{
    return head == "graph" || head == "node" || head == "edge";
}

std::size_t RequireTaskIndex(const DotParseContext &context,
                             const std::string &node,
                             const std::string &direction)
{
    const auto taskIt = context.taskIndexByName.find(node);
    if (taskIt != context.taskIndexByName.end()) {
        return taskIt->second;
    }
    throw std::runtime_error("DOT edge references undefined " + direction + " node: " + node);
}

EdgeEndpoints ParseEdgeEndpoints(const DotParseContext &context,
                                 const std::string &head,
                                 const std::map<std::string, std::string> &attrs)
{
    const auto arrow = head.find("->");
    EdgeEndpoints endpoints;
    endpoints.fromNode = Trim(head.substr(0, arrow));
    endpoints.toNode = Trim(head.substr(arrow + 2));
    endpoints.fromTask = RequireTaskIndex(context, endpoints.fromNode, "source");
    endpoints.toTask = RequireTaskIndex(context, endpoints.toNode, "target");
    endpoints.fromPort = ParsePortId(RequireField(attrs, "taillabel", head), "taillabel");
    endpoints.toPort = ParsePortId(RequireField(attrs, "headlabel", head), "headlabel");
    return endpoints;
}

void ApplyQueueLabelField(QueueConfig &queue, const std::string &field)
{
    const auto pos = field.find('=');
    if (pos == std::string::npos) {
        queue.overflow = ParseDotOverflow(field);
        return;
    }

    const auto key = field.substr(0, pos);
    const auto value = field.substr(pos + 1);
    if (key == "depth") {
        queue.depth = ParseSize(value, "depth");
    } else if (key == "overflow") {
        queue.overflow = ParseDotOverflow(value);
    }
}

void ApplyQueueLabelLine(QueueConfig &queue, const std::string &line)
{
    for (const auto &field : Split(line, ' ')) {
        ApplyQueueLabelField(queue, field);
    }
}

QueueConfig ParseEdgeQueue(const std::string &head,
                           const std::map<std::string, std::string> &attrs,
                           const EdgeEndpoints &endpoints)
{
    const auto label = UnescapeDotLabel(RequireField(attrs, "label", head));
    const auto labelFields = Split(label, '\n');
    if (labelFields.empty()) {
        throw std::runtime_error("DOT edge label must include queue type: " + head);
    }

    QueueConfig queue;
    queue.name = AutoQueueName(
        endpoints.fromNode, endpoints.fromPort, endpoints.toNode, endpoints.toPort);
    queue.type = Trim(labelFields.front());
    for (std::size_t i = 1; i < labelFields.size(); ++i) {
        ApplyQueueLabelLine(queue, labelFields[i]);
    }
    if (queue.depth == 0) {
        throw std::runtime_error("DOT edge missing depth: " + head);
    }
    return queue;
}

void AddQueue(DotParseContext &context, const QueueConfig &queue)
{
    if (!context.queueNames.insert(queue.name).second) {
        throw std::runtime_error("duplicate DOT queue name: " + queue.name);
    }
    context.config.queues.push_back(queue);
    context.queueByName[queue.name] = queue;
}

void ConnectEdgePorts(DotParseContext &context,
                      const EdgeEndpoints &endpoints,
                      const QueueConfig &queue)
{
    auto &from = context.config.tasks[endpoints.fromTask];
    auto &to = context.config.tasks[endpoints.toTask];
    MergePortSpec(context.graphOutputsByTaskType, from.type, endpoints.fromPort, queue.type);
    MergePortSpec(context.graphInputsByTaskType, to.type, endpoints.toPort, queue.type);
    if (!from.outputs.emplace(endpoints.fromPort, queue.name).second) {
        throw std::runtime_error("DOT output port is connected more than once: " +
                                 endpoints.fromNode + "." + std::to_string(endpoints.fromPort));
    }
    if (!to.inputs.emplace(endpoints.toPort, queue.name).second) {
        throw std::runtime_error("DOT input port is connected more than once: " +
                                 endpoints.toNode + "." + std::to_string(endpoints.toPort));
    }
}

void ParseEdgeStatement(DotParseContext &context,
                        const std::string &head,
                        const std::map<std::string, std::string> &attrs)
{
    const auto endpoints = ParseEdgeEndpoints(context, head, attrs);
    const auto queue = ParseEdgeQueue(head, attrs, endpoints);
    AddQueue(context, queue);
    ConnectEdgePorts(context, endpoints, queue);
}

void ParseTaskNodeStatement(DotParseContext &context,
                            const std::string &head,
                            const std::map<std::string, std::string> &attrs)
{
    if (attrs.find("label") == attrs.end()) {
        return;
    }
    auto task = ParseTaskStatement(head, attrs);
    if (context.taskIndexByName.find(task.name) != context.taskIndexByName.end()) {
        throw std::runtime_error("duplicate DOT node id: " + task.name);
    }
    if (context.registry.FindTaskType(task.type) == nullptr) {
        throw std::runtime_error("DOT node uses unregistered task type: " +
                                 task.name + " type=" + task.type);
    }
    context.taskIndexByName[task.name] = context.config.tasks.size();
    context.config.tasks.push_back(std::move(task));
}

void ParseTopLevelStatement(DotParseContext &context, const DotStatement &statement)
{
    if (statement.depth != 0) {
        return;
    }
    std::string head;
    std::map<std::string, std::string> attrs;
    if (!SplitStatementAndAttributes(statement.text, head, attrs)) {
        return;
    }
    if (IsEdgeStatement(head)) {
        ParseEdgeStatement(context, head, attrs);
        return;
    }
    if (IsGraphDefaultStatement(head)) {
        return;
    }
    ParseTaskNodeStatement(context, head, attrs);
}

void ResolveTaskTriggers(DotParseContext &context)
{
    for (auto &task : context.config.tasks) {
        task.trigger.queues = ResolveTriggerQueues(task, context.queueByName);
    }
}

void MergeRegisteredTaskPorts(DotParseContext &context)
{
    std::set<std::string> mergedTaskTypes;
    for (const auto &task : context.config.tasks) {
        if (!mergedTaskTypes.insert(task.type).second) {
            continue;
        }
        context.registry.MergeTaskPorts(
            task.type,
            context.graphInputsByTaskType[task.type],
            context.graphOutputsByTaskType[task.type]);
    }
}

GraphConfig ParseGraphStatements(const std::vector<DotStatement> &statements,
                                 Registry &registry)
{
    DotParseContext context{{}, {}, {}, {}, {}, {}, registry};
    for (const auto &statement : statements) {
        ParseTopLevelStatement(context, statement);
    }
    ResolveTaskTriggers(context);
    MergeRegisteredTaskPorts(context);
    return std::move(context.config);
}

} // namespace

GraphConfig ParseGraphConfigDot(const std::string &dotText,
                                const std::string &subgraphName,
                                Registry &registry)
{
    const auto body = ExtractSubgraphBody(dotText, subgraphName);
    const auto statements = CollectStatements(body);
    return ParseGraphStatements(statements, registry);
}

GraphConfig ParseGraphConfigDotFile(const std::string &path,
                                    const std::string &subgraphName,
                                    Registry &registry)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open EventPipelineGraph DOT file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ParseGraphConfigDot(buffer.str(), subgraphName, registry);
}

} // namespace Epg
