#include "common/runtime_graph/runtime_graph.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace smartdrone {
namespace runtime_graph {
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

std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool PeerNameMatchesPort(const std::string& peerNode, const std::string& portName) {
    const auto peer = ToLower(peerNode);
    const auto port = ToLower(portName);
    if (peer.find(port) != std::string::npos) {
        return true;
    }
    if (port.size() >= 4 && port.back() == 'e') {
        return peer.find(port.substr(0, port.size() - 1)) != std::string::npos;
    }
    return false;
}

std::string StripQuotes(std::string value) {
    value = Trim(value);
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::string NormalizeMermaidLabel(std::string value) {
    const std::vector<std::string> breaks = {"<br/>", "<br />", "<br>"};
    for (const auto& marker : breaks) {
        std::size_t pos = 0;
        while ((pos = value.find(marker, pos)) != std::string::npos) {
            value.replace(pos, marker.size(), ";");
            ++pos;
        }
    }
    return value;
}

std::vector<std::string> SplitFields(const std::string& text) {
    std::vector<std::string> fields;
    std::string current;
    bool inQuote = false;
    char quote = '\0';

    for (char c : text) {
        if ((c == '"' || c == '\'') && (!inQuote || c == quote)) {
            inQuote = !inQuote;
            quote = inQuote ? c : '\0';
            current.push_back(c);
            continue;
        }
        if (!inQuote && (c == ';' || c == ',')) {
            auto field = Trim(current);
            if (!field.empty()) {
                fields.push_back(field);
            }
            current.clear();
            continue;
        }
        current.push_back(c);
    }

    auto field = Trim(current);
    if (!field.empty()) {
        fields.push_back(field);
    }
    return fields;
}

std::vector<std::string> SplitTriggerQueueRefs(const std::string& text) {
    std::vector<std::string> refs;
    std::string current;
    for (char c : text) {
        if (c == '+') {
            auto ref = Trim(current);
            if (!ref.empty()) {
                refs.push_back(ref);
            }
            current.clear();
            continue;
        }
        current.push_back(c);
    }

    auto ref = Trim(current);
    if (!ref.empty()) {
        refs.push_back(ref);
    }
    return refs;
}

std::map<std::string, std::string> ParseFields(const std::string& text) {
    std::map<std::string, std::string> result;
    for (const auto& field : SplitFields(NormalizeMermaidLabel(text))) {
        const auto pos = field.find('=');
        if (pos == std::string::npos) {
            throw std::runtime_error("Mermaid field must use key=value: " + field);
        }
        const auto key = Trim(field.substr(0, pos));
        const auto value = StripQuotes(field.substr(pos + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error("Mermaid field key and value must be non-empty: " + field);
        }
        result[key] = value;
    }
    return result;
}

OverflowPolicy ParseMermaidOverflow(const std::string& value) {
    if (value == "drop_newest" || value == "tail_drop") {
        return OverflowPolicy::DropNewest;
    }
    if (value == "overwrite_oldest" || value == "circular_overwrite") {
        return OverflowPolicy::OverwriteOldest;
    }
    throw std::runtime_error("unsupported Mermaid queue overflow policy: " + value);
}

TriggerMode ParseMermaidTriggerMode(const std::string& value) {
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
    throw std::runtime_error("unsupported Mermaid task trigger mode: " + value);
}

std::size_t ParseSize(const std::string& value, const std::string& field) {
    std::size_t parsedChars = 0;
    const auto parsed = std::stoul(value, &parsedChars, 10);
    if (parsedChars != value.size()) {
        throw std::runtime_error("Mermaid numeric field is invalid: " + field + "=" + value);
    }
    return parsed;
}

bool IsQueueTriggeredMode(TriggerMode mode) {
    return mode == TriggerMode::AnyQueueReady ||
           mode == TriggerMode::AllQueueReady ||
           mode == TriggerMode::PeriodicOrAnyQueueReady;
}

std::string RequireField(const std::map<std::string, std::string>& fields,
                         const std::string& key,
                         const std::string& owner) {
    const auto it = fields.find(key);
    if (it == fields.end()) {
        throw std::runtime_error("missing Mermaid field '" + key + "' on " + owner);
    }
    return it->second;
}

std::string SanitizeQueueName(std::string value) {
    for (auto& c : value) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            c = '_';
        }
    }
    return value;
}

struct Endpoint {
    std::string node;
    std::string port;
};

Endpoint ParseEndpoint(const std::string& value) {
    const auto endpoint = Trim(value);
    const auto pos = endpoint.find('.');
    if (pos == std::string::npos) {
        if (endpoint.empty()) {
            throw std::runtime_error("Mermaid edge endpoint must not be empty");
        }
        return Endpoint{endpoint, ""};
    }
    if (pos == 0 || pos + 1 >= endpoint.size()) {
        throw std::runtime_error("Mermaid edge endpoint must be node.port: " + endpoint);
    }
    return Endpoint{endpoint.substr(0, pos), endpoint.substr(pos + 1)};
}

std::string AutoQueueName(const Endpoint& from, const Endpoint& to) {
    return SanitizeQueueName(from.node + "_" + from.port + "_to_" + to.node + "_" + to.port);
}

TaskConfig ParseNodeLine(const std::string& line) {
    const auto open = line.find('[');
    const auto close = line.rfind(']');
    if (open == std::string::npos || close == std::string::npos || close <= open) {
        throw std::runtime_error("Mermaid node must use id[key=value; ...]: " + line);
    }

    TaskConfig task;
    task.name = Trim(line.substr(0, open));
    if (task.name.empty()) {
        throw std::runtime_error("Mermaid node id must not be empty: " + line);
    }

    const auto fields = ParseFields(StripQuotes(line.substr(open + 1, close - open - 1)));
    task.type = RequireField(fields, "type", task.name);
    task.trigger.mode = ParseMermaidTriggerMode(RequireField(fields, "trigger", task.name));

    const auto intervalIt = fields.find("interval_ms");
    if (intervalIt != fields.end()) {
        task.trigger.interval = std::chrono::milliseconds(
            static_cast<int>(ParseSize(intervalIt->second, "interval_ms")));
    }

    const auto triggerQueuesIt = fields.find("trigger_queues");
    if (triggerQueuesIt != fields.end()) {
        task.trigger.queues = SplitTriggerQueueRefs(triggerQueuesIt->second);
        if (task.trigger.queues.empty()) {
            throw std::runtime_error("Mermaid trigger_queues must not be empty: " + task.name);
        }
    }

    return task;
}

struct EdgeConfig {
    Endpoint from;
    Endpoint to;
    QueueConfig queue;
};

EdgeConfig ParseEdgeLine(const std::string& line) {
    const auto arrow = line.find("-->");
    if (arrow == std::string::npos) {
        throw std::runtime_error("Mermaid edge must use -->: " + line);
    }

    EdgeConfig edge;
    edge.from = ParseEndpoint(line.substr(0, arrow));

    std::string label;
    std::string right;
    const auto labelStart = arrow + 3;
    if (labelStart < line.size() && line[labelStart] == '|') {
        const auto labelEnd = line.find('|', labelStart + 1);
        if (labelEnd == std::string::npos) {
            throw std::runtime_error("Mermaid edge label is missing closing '|': " + line);
        }
        label = line.substr(labelStart + 1, labelEnd - labelStart - 1);
        right = line.substr(labelEnd + 1);
    } else {
        right = line.substr(labelStart);
    }

    edge.to = ParseEndpoint(right);

    const auto fields = ParseFields(StripQuotes(label));
    const auto fromIt = fields.find("from");
    if (edge.from.port.empty() && fromIt != fields.end()) {
        edge.from.port = fromIt->second;
    }
    const auto toIt = fields.find("to");
    if (edge.to.port.empty() && toIt != fields.end()) {
        edge.to.port = toIt->second;
    }

    const auto nameIt = fields.find("name");
    if (nameIt != fields.end()) {
        edge.queue.name = nameIt->second;
    }
    edge.queue.type = RequireField(fields, "type", edge.queue.name);
    edge.queue.depth = ParseSize(RequireField(fields, "depth", edge.queue.name), "depth");
    edge.queue.overflow = ParseMermaidOverflow(RequireField(fields, "overflow", edge.queue.name));
    return edge;
}

const Registry::TaskTypeInfo& FindTaskTypeOrThrow(const Registry& registry,
                                                  const TaskConfig& task) {
    const auto* type = registry.FindTaskType(task.type);
    if (!type) {
        throw std::runtime_error("Mermaid node uses unregistered task type: " +
                                 task.name + " type=" + task.type);
    }
    return *type;
}

std::vector<PortSpec> MatchingPorts(const std::vector<PortSpec>& ports,
                                    const std::string& messageType) {
    std::vector<PortSpec> result;
    for (const auto& port : ports) {
        if (port.type == messageType) {
            result.push_back(port);
        }
    }
    return result;
}

std::string InferPort(const std::vector<PortSpec>& declaredPorts,
                      const std::set<std::string>& usedPorts,
                      const std::string& messageType,
                      const std::string& ownerNode,
                      const std::string& peerNode,
                      const std::string& peerPort,
                      const std::string& direction) {
    auto candidates = MatchingPorts(declaredPorts, messageType);
    if (candidates.empty()) {
        throw std::runtime_error("Mermaid cannot infer " + direction +
                                 " port for node '" + ownerNode +
                                 "': no port accepts type " + messageType);
    }

    std::vector<PortSpec> unused;
    for (const auto& candidate : candidates) {
        if (usedPorts.find(candidate.name) == usedPorts.end()) {
            unused.push_back(candidate);
        }
    }
    if (!unused.empty()) {
        candidates = unused;
    }

    if (candidates.size() == 1) {
        return candidates.front().name;
    }

    std::vector<PortSpec> peerPortMatches;
    if (!peerPort.empty()) {
        for (const auto& candidate : candidates) {
            if (candidate.name == peerPort) {
                peerPortMatches.push_back(candidate);
            }
        }
        if (peerPortMatches.size() == 1) {
            return peerPortMatches.front().name;
        }
    }

    std::vector<PortSpec> peerNameMatches;
    for (const auto& candidate : candidates) {
        if (PeerNameMatchesPort(peerNode, candidate.name)) {
            peerNameMatches.push_back(candidate);
        }
    }
    if (peerNameMatches.size() == 1) {
        return peerNameMatches.front().name;
    }

    throw std::runtime_error("Mermaid cannot infer " + direction +
                             " port for node '" + ownerNode +
                             "': specify from/to or make the graph unambiguous");
}

void InferMissingPorts(EdgeConfig& edge,
                       const RuntimeGraphConfig& config,
                       const std::map<std::string, std::size_t>& taskIndexByName,
                       const Registry& registry,
                       const std::map<std::string, std::set<std::string>>& usedOutputs,
                       const std::map<std::string, std::set<std::string>>& usedInputs) {
    const auto& fromTask = config.tasks.at(taskIndexByName.at(edge.from.node));
    const auto& toTask = config.tasks.at(taskIndexByName.at(edge.to.node));
    const auto& fromType = FindTaskTypeOrThrow(registry, fromTask);
    const auto& toType = FindTaskTypeOrThrow(registry, toTask);

    const auto fromUsedIt = usedOutputs.find(edge.from.node);
    const auto toUsedIt = usedInputs.find(edge.to.node);
    const std::set<std::string> noUsedPorts;
    const auto& fromUsed = fromUsedIt == usedOutputs.end() ? noUsedPorts : fromUsedIt->second;
    const auto& toUsed = toUsedIt == usedInputs.end() ? noUsedPorts : toUsedIt->second;

    if (edge.from.port.empty()) {
        edge.from.port = InferPort(fromType.outputs,
                                   fromUsed,
                                   edge.queue.type,
                                   edge.from.node,
                                   edge.to.node,
                                   edge.to.port,
                                   "output");
    }
    if (edge.to.port.empty()) {
        edge.to.port = InferPort(toType.inputs,
                                 toUsed,
                                 edge.queue.type,
                                 edge.to.node,
                                 edge.from.node,
                                 edge.from.port,
                                 "input");
    }

    if (edge.queue.name.empty()) {
        edge.queue.name = AutoQueueName(edge.from, edge.to);
    }
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
            throw std::runtime_error("Mermaid trigger_queues references non-input queue or type: " +
                                     task.name + " -> " + ref);
        }
    }
    return queues;
}

bool IsHeaderLine(const std::string& line) {
    return line.find("flowchart") == 0 || line.find("graph") == 0;
}

bool IsSubgraphLine(const std::string& line) {
    return line.find("subgraph") == 0;
}

bool IsEndLine(const std::string& line) {
    return line == "end";
}

std::string ExtractMermaidBlock(const std::string& text) {
    const std::string fence = "```";
    const std::string mermaidFence = "```mermaid";
    const auto begin = text.find(mermaidFence);
    if (begin == std::string::npos) {
        return text;
    }

    const auto contentBegin = text.find('\n', begin + mermaidFence.size());
    if (contentBegin == std::string::npos) {
        return {};
    }
    const auto end = text.find(fence, contentBegin + 1);
    if (end == std::string::npos) {
        return text.substr(contentBegin + 1);
    }
    return text.substr(contentBegin + 1, end - contentBegin - 1);
}

std::string StripMermaidComment(const std::string& line) {
    const auto comment = line.find("%%");
    return comment == std::string::npos ? line : line.substr(0, comment);
}

bool SubgraphLineMatches(const std::string& line, const std::string& subgraphName) {
    if (!IsSubgraphLine(line)) {
        return false;
    }
    std::istringstream input(line);
    std::string keyword;
    std::string id;
    input >> keyword >> id;
    const auto labelInId = id.find('[');
    if (labelInId != std::string::npos) {
        id = id.substr(0, labelInId);
    }
    if (id == subgraphName) {
        return true;
    }

    const auto labelBegin = line.find("[\"");
    if (labelBegin == std::string::npos) {
        return false;
    }
    const auto labelEnd = line.find("\"]", labelBegin + 2);
    if (labelEnd == std::string::npos) {
        return false;
    }
    return line.substr(labelBegin + 2, labelEnd - labelBegin - 2) == subgraphName;
}

std::string ExtractMermaidSubgraphBlock(const std::string& mermaidText,
                                        const std::string& subgraphName) {
    std::istringstream input(ExtractMermaidBlock(mermaidText));
    std::ostringstream output;
    output << "flowchart LR\n";

    std::string rawLine;
    bool inTarget = false;
    int depth = 0;
    while (std::getline(input, rawLine)) {
        const auto line = Trim(StripMermaidComment(rawLine));
        if (line.empty() || IsHeaderLine(line)) {
            continue;
        }
        if (!inTarget) {
            if (SubgraphLineMatches(line, subgraphName)) {
                inTarget = true;
                depth = 1;
            }
            continue;
        }

        if (IsSubgraphLine(line)) {
            ++depth;
            continue;
        }
        if (IsEndLine(line)) {
            --depth;
            if (depth == 0) {
                return output.str();
            }
            continue;
        }
        if (depth == 1) {
            output << line << '\n';
        }
    }

    throw std::runtime_error("Mermaid subgraph not found or not closed: " + subgraphName);
}

} // namespace

RuntimeGraphConfig ParseRuntimeGraphConfigMermaidInternal(const std::string& mermaidText,
                                                          const Registry* registry) {
    RuntimeGraphConfig config;
    std::map<std::string, std::size_t> taskIndexByName;
    std::set<std::string> queueNames;
    std::map<std::string, std::set<std::string>> usedOutputs;
    std::map<std::string, std::set<std::string>> usedInputs;
    std::map<std::string, QueueConfig> queueByName;

    std::istringstream input(ExtractMermaidBlock(mermaidText));
    std::string rawLine;
    while (std::getline(input, rawLine)) {
        const auto line = Trim(StripMermaidComment(rawLine));
        if (line.empty() || IsHeaderLine(line)) {
            continue;
        }
        if (IsSubgraphLine(line) || IsEndLine(line)) {
            continue;
        }

        if (line.find("-->") != std::string::npos) {
            auto edge = ParseEdgeLine(line);
            const auto fromTask = taskIndexByName.find(edge.from.node);
            const auto toTask = taskIndexByName.find(edge.to.node);
            if (fromTask == taskIndexByName.end()) {
                throw std::runtime_error("Mermaid edge references undefined source node: " + edge.from.node);
            }
            if (toTask == taskIndexByName.end()) {
                throw std::runtime_error("Mermaid edge references undefined target node: " + edge.to.node);
            }

            if (registry) {
                InferMissingPorts(edge, config, taskIndexByName, *registry, usedOutputs, usedInputs);
            } else {
                if (edge.from.port.empty()) {
                    throw std::runtime_error("missing Mermaid field 'from' on " + edge.from.node);
                }
                if (edge.to.port.empty()) {
                    throw std::runtime_error("missing Mermaid field 'to' on " + edge.to.node);
                }
                if (edge.queue.name.empty()) {
                    edge.queue.name = AutoQueueName(edge.from, edge.to);
                }
            }

            if (!queueNames.insert(edge.queue.name).second) {
                throw std::runtime_error("duplicate Mermaid queue name: " + edge.queue.name);
            }

            config.queues.push_back(edge.queue);
            queueByName[edge.queue.name] = edge.queue;
            auto& from = config.tasks[fromTask->second];
            auto& to = config.tasks[toTask->second];
            if (!from.outputs.emplace(edge.from.port, edge.queue.name).second) {
                throw std::runtime_error("Mermaid output port is connected more than once: " +
                                         edge.from.node + "." + edge.from.port);
            }
            if (!to.inputs.emplace(edge.to.port, edge.queue.name).second) {
                throw std::runtime_error("Mermaid input port is connected more than once: " +
                                         edge.to.node + "." + edge.to.port);
            }
            usedOutputs[edge.from.node].insert(edge.from.port);
            usedInputs[edge.to.node].insert(edge.to.port);
            continue;
        }

        auto task = ParseNodeLine(line);
        if (taskIndexByName.find(task.name) != taskIndexByName.end()) {
            throw std::runtime_error("duplicate Mermaid node id: " + task.name);
        }
        taskIndexByName[task.name] = config.tasks.size();
        config.tasks.push_back(std::move(task));
    }

    for (auto& task : config.tasks) {
        task.trigger.queues = ResolveTriggerQueues(task, queueByName);
    }

    return config;
}

RuntimeGraphConfig ParseRuntimeGraphConfigMermaid(const std::string& mermaidText) {
    return ParseRuntimeGraphConfigMermaidInternal(mermaidText, nullptr);
}

RuntimeGraphConfig ParseRuntimeGraphConfigMermaid(const std::string& mermaidText,
                                                  const Registry& registry) {
    return ParseRuntimeGraphConfigMermaidInternal(mermaidText, &registry);
}

RuntimeGraphConfig ParseRuntimeGraphConfigMermaidFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open runtime graph Mermaid file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ParseRuntimeGraphConfigMermaid(buffer.str());
}

RuntimeGraphConfig ParseRuntimeGraphConfigMermaidFile(const std::string& path,
                                                      const Registry& registry) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open runtime graph Mermaid file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ParseRuntimeGraphConfigMermaid(buffer.str(), registry);
}

RuntimeGraphConfig ParseRuntimeGraphConfigMermaidSubgraph(const std::string& mermaidText,
                                                          const std::string& subgraphName,
                                                          const Registry& registry) {
    return ParseRuntimeGraphConfigMermaid(ExtractMermaidSubgraphBlock(mermaidText, subgraphName), registry);
}

RuntimeGraphConfig ParseRuntimeGraphConfigMermaidSubgraphFile(const std::string& path,
                                                              const std::string& subgraphName,
                                                              const Registry& registry) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open runtime graph Mermaid file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ParseRuntimeGraphConfigMermaidSubgraph(buffer.str(), subgraphName, registry);
}

} // namespace runtime_graph
} // namespace smartdrone
