#include "common/epg/epg.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace epg {
namespace {

std::string Trim(const std::string& value) {
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

std::string StripQuotes(std::string value) {
    value = Trim(value);
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
    }
    return value;
}

std::size_t ParseSize(const std::string& value, const std::string& field) {
    std::size_t parsedChars = 0;
    std::size_t parsed = 0;
    try {
        parsed = std::stoul(value, &parsedChars, 10);
    } catch (const std::exception&) {
        throw std::runtime_error("DOT numeric field is invalid: " + field + "=" + value);
    }
    if (parsedChars != value.size()) {
        throw std::runtime_error("DOT numeric field is invalid: " + field + "=" + value);
    }
    return parsed;
}

PortId ParsePortId(const std::string& value, const std::string& field) {
    return static_cast<PortId>(ParseSize(value, field));
}

OverflowPolicy ParseDotOverflow(const std::string& value) {
    if (value == "drop_newest" || value == "tail_drop") {
        return OverflowPolicy::DropNewest;
    }
    if (value == "overwrite_oldest" || value == "circular_overwrite") {
        return OverflowPolicy::OverwriteOldest;
    }
    throw std::runtime_error("unsupported DOT queue overflow policy: " + value);
}

TriggerMode ParseDotTriggerMode(const std::string& value) {
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
    throw std::runtime_error("unsupported DOT task trigger mode: " + value);
}

std::vector<std::string> Split(const std::string& text, char delimiter) {
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

std::vector<std::string> SplitTriggerQueueRefs(const std::string& text) {
    return Split(text, '+');
}

std::map<std::string, std::string> ParseFields(const std::vector<std::string>& fields) {
    std::map<std::string, std::string> result;
    for (const auto& field : fields) {
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

std::string RequireField(const std::map<std::string, std::string>& fields,
                         const std::string& key,
                         const std::string& owner) {
    const auto it = fields.find(key);
    if (it == fields.end()) {
        throw std::runtime_error("missing DOT field '" + key + "' on " + owner);
    }
    return it->second;
}

std::vector<std::string> ParseRecordLabelFields(std::string label) {
    label = StripQuotes(std::move(label));
    if (label.size() >= 2 && label.front() == '{' && label.back() == '}') {
        label = label.substr(1, label.size() - 2);
    }
    return Split(label, '|');
}

std::string UnescapeDotLabel(std::string value) {
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

std::map<std::string, std::string> ParseAttributeBlock(const std::string& text) {
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
        if (c == '"') {
            inQuote = !inQuote;
            current.push_back(c);
            continue;
        }
        if (inQuote) {
            if (c == '{') {
                ++nestedBraces;
            } else if (c == '}' && nestedBraces > 0) {
                --nestedBraces;
            }
        }
        if (!inQuote && c == ',') {
            flush();
            continue;
        }
        current.push_back(c);
    }
    flush();
    return attrs;
}

std::string SanitizeQueueName(std::string value) {
    for (auto& c : value) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            c = '_';
        }
    }
    return value;
}

std::string AutoQueueName(const std::string& fromNode,
                          PortId fromPort,
                          const std::string& toNode,
                          PortId toPort) {
    return SanitizeQueueName(fromNode + "_" + std::to_string(fromPort) +
                             "_to_" + toNode + "_" + std::to_string(toPort));
}

bool IsQueueTriggeredMode(TriggerMode mode) {
    return mode == TriggerMode::AnyQueueReady ||
           mode == TriggerMode::AllQueueReady ||
           mode == TriggerMode::PeriodicOrAnyQueueReady;
}

std::vector<std::string> ResolveTriggerQueues(
    const TaskConfig& task,
    const std::map<std::string, QueueConfig>& queueByName) {
    if (!IsQueueTriggeredMode(task.trigger.mode)) {
        return task.trigger.queues;
    }
    if (task.trigger.queues.empty()) {
        std::vector<std::string> queues;
        for (const auto& input : task.inputs) {
            queues.push_back(input.second);
        }
        return queues;
    }

    std::vector<std::string> queues;
    std::set<std::string> seen;
    for (const auto& ref : task.trigger.queues) {
        const auto queueIt = queueByName.find(ref);
        if (queueIt != queueByName.end()) {
            if (seen.insert(ref).second) {
                queues.push_back(ref);
            }
            continue;
        }

        bool matched = false;
        for (const auto& input : task.inputs) {
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

std::vector<DotStatement> CollectStatements(const std::string& dotText) {
    std::vector<DotStatement> statements;
    std::string current;
    int depth = 0;
    int statementDepth = 0;
    bool inQuote = false;
    bool inAttributes = false;

    for (std::size_t i = 0; i < dotText.size(); ++i) {
        const char c = dotText[i];
        if (c == '"') {
            inQuote = !inQuote;
        }
        if (!inQuote && c == '/' && i + 1 < dotText.size() && dotText[i + 1] == '/') {
            while (i < dotText.size() && dotText[i] != '\n') {
                ++i;
            }
            continue;
        }
        if (!inQuote && c == '#') {
            while (i < dotText.size() && dotText[i] != '\n') {
                ++i;
            }
            continue;
        }
        if (!inQuote && c == '{') {
            ++depth;
            current.clear();
            statementDepth = depth;
            continue;
        }
        if (!inQuote && c == '}') {
            if (!Trim(current).empty()) {
                statements.push_back(DotStatement{statementDepth, Trim(current)});
            }
            current.clear();
            --depth;
            statementDepth = depth;
            continue;
        }
        if (!inQuote && c == '[') {
            inAttributes = true;
        } else if (!inQuote && c == ']') {
            inAttributes = false;
        }
        if (!inQuote && !inAttributes && c == ';') {
            auto statement = Trim(current);
            if (!statement.empty()) {
                statements.push_back(DotStatement{statementDepth, std::move(statement)});
            }
            current.clear();
            continue;
        }
        if (current.empty() && !std::isspace(static_cast<unsigned char>(c))) {
            statementDepth = depth;
        }
        current.push_back(c);
    }
    auto statement = Trim(current);
    if (!statement.empty()) {
        statements.push_back(DotStatement{statementDepth, std::move(statement)});
    }
    return statements;
}

std::string ExtractSubgraphBody(const std::string& dotText, const std::string& subgraphName) {
    const std::string needle = "subgraph " + subgraphName;
    const auto begin = dotText.find(needle);
    if (begin == std::string::npos) {
        throw std::runtime_error("DOT subgraph not found: " + subgraphName);
    }
    const auto open = dotText.find('{', begin + needle.size());
    if (open == std::string::npos) {
        throw std::runtime_error("DOT subgraph missing body: " + subgraphName);
    }
    bool inQuote = false;
    int depth = 0;
    for (std::size_t i = open; i < dotText.size(); ++i) {
        const char c = dotText[i];
        if (c == '"') {
            inQuote = !inQuote;
            continue;
        }
        if (inQuote) {
            continue;
        }
        if (c == '{') {
            ++depth;
            continue;
        }
        if (c == '}') {
            --depth;
            if (depth == 0) {
                return dotText.substr(open + 1, i - open - 1);
            }
        }
    }
    throw std::runtime_error("DOT subgraph not closed: " + subgraphName);
}

bool SplitStatementAndAttributes(const std::string& statement,
                                 std::string& head,
                                 std::map<std::string, std::string>& attrs) {
    const auto open = statement.find('[');
    const auto close = statement.rfind(']');
    if (open == std::string::npos || close == std::string::npos || close <= open) {
        return false;
    }
    head = Trim(statement.substr(0, open));
    attrs = ParseAttributeBlock(statement.substr(open + 1, close - open - 1));
    return true;
}

TaskConfig ParseTaskStatement(const std::string& head,
                              const std::map<std::string, std::string>& attrs) {
    const auto labelIt = attrs.find("label");
    if (labelIt == attrs.end()) {
        throw std::runtime_error("DOT task node missing label: " + head);
    }
    const auto fields = ParseFields(ParseRecordLabelFields(labelIt->second));

    TaskConfig task;
    task.name = head;
    task.type = RequireField(fields, "type", task.name);
    task.trigger.mode = ParseDotTriggerMode(RequireField(fields, "trigger", task.name));

    const auto intervalIt = fields.find("interval_ms");
    if (intervalIt != fields.end()) {
        task.trigger.interval = std::chrono::milliseconds(
            static_cast<int>(ParseSize(intervalIt->second, "interval_ms")));
    }
    const auto triggerQueuesIt = fields.find("trigger_queues");
    if (triggerQueuesIt != fields.end()) {
        task.trigger.queues = SplitTriggerQueueRefs(triggerQueuesIt->second);
    }
    return task;
}

void MergePortSpec(std::map<std::string, std::vector<PortSpec>>& byTask,
                   const std::string& taskType,
                   PortId port,
                   const std::string& queueType) {
    byTask[taskType].push_back(PortSpec{port, queueType});
}

} // namespace

GraphConfig ParseGraphConfigDot(const std::string& dotText,
                                              const std::string& subgraphName,
                                              Registry& registry) {
    GraphConfig config;
    std::map<std::string, std::size_t> taskIndexByName;
    std::map<std::string, QueueConfig> queueByName;
    std::set<std::string> queueNames;
    std::map<std::string, std::vector<PortSpec>> graphInputsByTaskType;
    std::map<std::string, std::vector<PortSpec>> graphOutputsByTaskType;

    const auto body = ExtractSubgraphBody(dotText, subgraphName);
    const auto statements = CollectStatements(body);
    for (const auto& statement : statements) {
        if (statement.depth != 0) {
            continue;
        }
        std::string head;
        std::map<std::string, std::string> attrs;
        if (!SplitStatementAndAttributes(statement.text, head, attrs)) {
            continue;
        }
        if (head.find("->") != std::string::npos) {
            const auto arrow = head.find("->");
            const auto fromNode = Trim(head.substr(0, arrow));
            const auto toNode = Trim(head.substr(arrow + 2));
            const auto fromTask = taskIndexByName.find(fromNode);
            const auto toTask = taskIndexByName.find(toNode);
            if (fromTask == taskIndexByName.end()) {
                throw std::runtime_error("DOT edge references undefined source node: " + fromNode);
            }
            if (toTask == taskIndexByName.end()) {
                throw std::runtime_error("DOT edge references undefined target node: " + toNode);
            }

            const auto fromPort = ParsePortId(RequireField(attrs, "taillabel", head), "taillabel");
            const auto toPort = ParsePortId(RequireField(attrs, "headlabel", head), "headlabel");
            const auto label = UnescapeDotLabel(RequireField(attrs, "label", head));
            const auto labelFields = Split(label, '\n');
            if (labelFields.empty()) {
                throw std::runtime_error("DOT edge label must include queue type: " + head);
            }

            QueueConfig queue;
            queue.name = AutoQueueName(fromNode, fromPort, toNode, toPort);
            queue.type = Trim(labelFields.front());
            for (std::size_t i = 1; i < labelFields.size(); ++i) {
                for (const auto& field : Split(labelFields[i], ' ')) {
                    const auto pos = field.find('=');
                    if (pos == std::string::npos) {
                        queue.overflow = ParseDotOverflow(field);
                        continue;
                    }
                    const auto key = field.substr(0, pos);
                    const auto value = field.substr(pos + 1);
                    if (key == "depth") {
                        queue.depth = ParseSize(value, "depth");
                    } else if (key == "overflow") {
                        queue.overflow = ParseDotOverflow(value);
                    }
                }
            }
            if (queue.depth == 0) {
                throw std::runtime_error("DOT edge missing depth: " + head);
            }

            if (!queueNames.insert(queue.name).second) {
                throw std::runtime_error("duplicate DOT queue name: " + queue.name);
            }
            config.queues.push_back(queue);
            queueByName[queue.name] = queue;

            auto& from = config.tasks[fromTask->second];
            auto& to = config.tasks[toTask->second];
            MergePortSpec(graphOutputsByTaskType, from.type, fromPort, queue.type);
            MergePortSpec(graphInputsByTaskType, to.type, toPort, queue.type);
            if (!from.outputs.emplace(fromPort, queue.name).second) {
                throw std::runtime_error("DOT output port is connected more than once: " +
                                         fromNode + "." + std::to_string(fromPort));
            }
            if (!to.inputs.emplace(toPort, queue.name).second) {
                throw std::runtime_error("DOT input port is connected more than once: " +
                                         toNode + "." + std::to_string(toPort));
            }
            continue;
        }

        if (head == "graph" || head == "node" || head == "edge") {
            continue;
        }
        if (attrs.find("label") == attrs.end()) {
            continue;
        }
        auto task = ParseTaskStatement(head, attrs);
        if (taskIndexByName.find(task.name) != taskIndexByName.end()) {
            throw std::runtime_error("duplicate DOT node id: " + task.name);
        }
        const auto* taskType = registry.FindTaskType(task.type);
        if (!taskType) {
            throw std::runtime_error("DOT node uses unregistered task type: " +
                                     task.name + " type=" + task.type);
        }
        taskIndexByName[task.name] = config.tasks.size();
        config.tasks.push_back(std::move(task));
    }

    for (auto& task : config.tasks) {
        task.trigger.queues = ResolveTriggerQueues(task, queueByName);
    }

    std::set<std::string> mergedTaskTypes;
    for (const auto& task : config.tasks) {
        if (!mergedTaskTypes.insert(task.type).second) {
            continue;
        }
        registry.MergeTaskPorts(
            task.type,
            graphInputsByTaskType[task.type],
            graphOutputsByTaskType[task.type]);
    }
    return config;
}

GraphConfig ParseGraphConfigDotFile(const std::string& path,
                                                  const std::string& subgraphName,
                                                  Registry& registry) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open EventPipelineGraph DOT file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ParseGraphConfigDot(buffer.str(), subgraphName, registry);
}

} // namespace epg
