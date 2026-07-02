#include "core/application/planning/path_planner_plugin.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace SmartDrone::Core::Application {
namespace {

struct GridIndex {
    int x{0};
    int y{0};
    int z{0};
};

struct GridIndexHash {
    size_t operator()(const GridIndex &value) const
    {
        size_t seed = static_cast<size_t>(value.x) * 73856093U;
        seed ^= static_cast<size_t>(value.y) * 19349663U;
        seed ^= static_cast<size_t>(value.z) * 83492791U;
        return seed;
    }
};

struct GridIndexEqual {
    bool operator()(const GridIndex &left, const GridIndex &right) const
    {
        return left.x == right.x && left.y == right.y &&
               left.z == right.z;
    }
};

struct SearchQueueEntry {
    GridIndex index{};
    float gScore{0.0f};
    float fScore{0.0f};
};

struct SearchQueueCompare {
    bool operator()(const SearchQueueEntry &left,
                    const SearchQueueEntry &right) const
    {
        return left.fScore > right.fScore;
    }
};

struct SearchBounds {
    float minX{0.0f};
    float maxX{0.0f};
    float minY{0.0f};
    float maxY{0.0f};
    float minZ{0.0f};
    float maxZ{0.0f};
};

struct PlanningGrid {
    float voxelSizeM{0.3f};
    SearchBounds bounds{};
    std::unordered_set<GridIndex, GridIndexHash, GridIndexEqual>
        occupied;
};

struct SearchNode {
    float gScore{std::numeric_limits<float>::infinity()};
    GridIndex parent{};
    bool hasParent{false};
    bool closed{false};
};

struct GridSearchResult {
    bool success{false};
    std::vector<GridIndex> cells;
    int expandedNodes{0};
};

struct SearchState {
    std::priority_queue<SearchQueueEntry, std::vector<SearchQueueEntry>,
                        SearchQueueCompare>
        openSet;
    std::unordered_map<GridIndex, SearchNode, GridIndexHash, GridIndexEqual>
        nodes;
    int expandedNodes{0};
};

bool SameIndex(const GridIndex &left, const GridIndex &right)
{
    return GridIndexEqual{}(left, right);
}

float ClampValue(float value, float minValue, float maxValue)
{
    if (!std::isfinite(value)) {
        return minValue;
    }
    return std::max(minValue, std::min(maxValue, value));
}

PathPlannerConfig SanitizeConfig(const PathPlannerConfig &config)
{
    PathPlannerConfig out{};
    out.voxelSizeM = ClampValue(config.voxelSizeM, 0.1f, 1.0f);
    out.clearanceM = ClampValue(config.clearanceM, 0.0f, 3.0f);
    out.searchPaddingM = ClampValue(config.searchPaddingM, 0.5f, 10.0f);
    out.maxExpandedNodes = std::max(100, std::min(200000, config.maxExpandedNodes));
    return out;
}

bool IsFinitePoint(const PathPlannerPoint &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

GridIndex CellForPoint(const PathPlannerPoint &point, float voxelSize)
{
    return {static_cast<int>(std::floor(point.x / voxelSize)),
            static_cast<int>(std::floor(point.y / voxelSize)),
            static_cast<int>(std::floor(point.z / voxelSize))};
}

PathPlannerPoint PointForVoxel(const OccupiedVoxel &voxel)
{
    return {voxel.x, voxel.y, voxel.z};
}

PathPlannerPoint CellCenter(const GridIndex &index, float voxelSize)
{
    return {(static_cast<float>(index.x) + 0.5f) * voxelSize,
            (static_cast<float>(index.y) + 0.5f) * voxelSize,
            (static_cast<float>(index.z) + 0.5f) * voxelSize};
}

float Distance(const PathPlannerPoint &left, const PathPlannerPoint &right)
{
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float Heuristic(const GridIndex &left, const GridIndex &right,
                float voxelSize)
{
    const float dx = static_cast<float>(left.x - right.x);
    const float dy = static_cast<float>(left.y - right.y);
    const float dz = static_cast<float>(left.z - right.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz) * voxelSize;
}

SearchBounds BuildBounds(const PathPlannerRequest &request,
                         const PathPlannerConfig &config)
{
    const float padding = config.searchPaddingM + config.clearanceM;
    SearchBounds bounds{};
    bounds.minX = std::min(request.start.x, request.goal.x) - padding;
    bounds.maxX = std::max(request.start.x, request.goal.x) + padding;
    bounds.minY = std::min(request.start.y, request.goal.y) - padding;
    bounds.maxY = std::max(request.start.y, request.goal.y) + padding;
    bounds.minZ = std::min(request.start.z, request.goal.z) - padding;
    bounds.maxZ = std::max(request.start.z, request.goal.z) + padding;
    return bounds;
}

bool InBounds(const GridIndex &index, const PlanningGrid &grid)
{
    const PathPlannerPoint point = CellCenter(index, grid.voxelSizeM);
    return point.x >= grid.bounds.minX && point.x <= grid.bounds.maxX &&
           point.y >= grid.bounds.minY && point.y <= grid.bounds.maxY &&
           point.z >= grid.bounds.minZ && point.z <= grid.bounds.maxZ;
}

std::vector<GridIndex> BuildInflationOffsets(float clearanceM,
                                             float voxelSize)
{
    const int radius = static_cast<int>(std::ceil(clearanceM / voxelSize));
    const int width = radius * 2 + 1;
    const int count = width * width * width;
    std::vector<GridIndex> offsets;
    offsets.reserve(static_cast<size_t>(count));
    for (int linear = 0; linear < count; ++linear) {
        const int dx = linear % width - radius;
        const int dy = (linear / width) % width - radius;
        const int dz = linear / (width * width) - radius;
        const float distance = std::sqrt(static_cast<float>(
            dx * dx + dy * dy + dz * dz)) * voxelSize;
        if (distance <= clearanceM + voxelSize * 0.5f) {
            offsets.push_back({dx, dy, dz});
        }
    }
    return offsets;
}

void AddInflatedVoxel(const OccupiedVoxel &voxel,
                      const std::vector<GridIndex> &offsets,
                      PlanningGrid &grid)
{
    const GridIndex center = CellForPoint(PointForVoxel(voxel), grid.voxelSizeM);
    for (const GridIndex &offset : offsets) {
        GridIndex cell{center.x + offset.x, center.y + offset.y,
                       center.z + offset.z};
        if (InBounds(cell, grid)) {
            grid.occupied.insert(cell);
        }
    }
}

PlanningGrid BuildPlanningGrid(const PathPlannerRequest &request,
                               const PathPlannerConfig &config)
{
    PlanningGrid grid{};
    grid.voxelSizeM = config.voxelSizeM;
    grid.bounds = BuildBounds(request, config);
    if (!request.occupancyMap) {
        return grid;
    }
    const auto offsets =
        BuildInflationOffsets(config.clearanceM, config.voxelSizeM);
    for (const OccupiedVoxel &voxel : request.occupancyMap->OccupiedVoxels()) {
        AddInflatedVoxel(voxel, offsets, grid);
    }
    return grid;
}

bool IsBlocked(const GridIndex &index, const PlanningGrid &grid)
{
    return grid.occupied.find(index) != grid.occupied.end();
}

std::vector<GridIndex> BuildNeighborOffsets()
{
    std::vector<GridIndex> offsets;
    offsets.reserve(26);
    for (int linear = 0; linear < 27; ++linear) {
        const int dx = linear % 3 - 1;
        const int dy = (linear / 3) % 3 - 1;
        const int dz = linear / 9 - 1;
        if (dx != 0 || dy != 0 || dz != 0) {
            offsets.push_back({dx, dy, dz});
        }
    }
    return offsets;
}

GridIndex AddIndex(const GridIndex &left, const GridIndex &right)
{
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

float StepCost(const GridIndex &offset, float voxelSize)
{
    const float dx = static_cast<float>(offset.x);
    const float dy = static_cast<float>(offset.y);
    const float dz = static_cast<float>(offset.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz) * voxelSize;
}

void PushSearchNode(SearchState &state, const GridIndex &index,
                    const SearchNode &node, float fScore)
{
    state.nodes[index] = node;
    state.openSet.push({index, node.gScore, fScore});
}

bool ShouldSkipEntry(const SearchQueueEntry &entry,
                     const SearchNode &node)
{
    return node.closed || entry.gScore > node.gScore + 0.0001f;
}

void TryOpenNeighbor(const GridIndex &current, const GridIndex &offset,
                     const GridIndex &goal, const PlanningGrid &grid,
                     SearchState &state)
{
    const GridIndex next = AddIndex(current, offset);
    if (!InBounds(next, grid) || IsBlocked(next, grid)) {
        return;
    }
    const SearchNode &currentNode = state.nodes[current];
    const float tentativeG =
        currentNode.gScore + StepCost(offset, grid.voxelSizeM);
    auto iter = state.nodes.find(next);
    if (iter != state.nodes.end() && iter->second.closed) {
        return;
    }
    if (iter != state.nodes.end() && tentativeG >= iter->second.gScore) {
        return;
    }
    SearchNode nextNode{};
    nextNode.gScore = tentativeG;
    nextNode.parent = current;
    nextNode.hasParent = true;
    PushSearchNode(state, next, nextNode,
                   tentativeG + Heuristic(next, goal, grid.voxelSizeM));
}

std::vector<GridIndex> ReconstructPath(
    const GridIndex &start, const GridIndex &goal,
    const std::unordered_map<GridIndex, SearchNode, GridIndexHash,
                             GridIndexEqual> &nodes)
{
    std::vector<GridIndex> cells;
    GridIndex current = goal;
    cells.push_back(current);
    while (!SameIndex(current, start)) {
        auto iter = nodes.find(current);
        if (iter == nodes.end() || !iter->second.hasParent) {
            return {};
        }
        current = iter->second.parent;
        cells.push_back(current);
    }
    std::reverse(cells.begin(), cells.end());
    return cells;
}

GridSearchResult MakeGridSearchResult(const GridIndex &start,
                                      const GridIndex &goal,
                                      const SearchState &state)
{
    GridSearchResult result{};
    result.success = true;
    result.expandedNodes = state.expandedNodes;
    result.cells = ReconstructPath(start, goal, state.nodes);
    result.success = !result.cells.empty();
    return result;
}

GridSearchResult SearchGrid(const GridIndex &start, const GridIndex &goal,
                            const PlanningGrid &grid, int maxExpandedNodes)
{
    SearchState state{};
    SearchNode startNode{};
    startNode.gScore = 0.0f;
    PushSearchNode(state, start, startNode,
                   Heuristic(start, goal, grid.voxelSizeM));
    const std::vector<GridIndex> neighborOffsets = BuildNeighborOffsets();
    while (!state.openSet.empty()) {
        const SearchQueueEntry entry = state.openSet.top();
        state.openSet.pop();
        SearchNode &node = state.nodes[entry.index];
        if (ShouldSkipEntry(entry, node)) {
            continue;
        }
        if (SameIndex(entry.index, goal)) {
            return MakeGridSearchResult(start, goal, state);
        }
        if (state.expandedNodes >= maxExpandedNodes) {
            return {};
        }
        node.closed = true;
        ++state.expandedNodes;
        for (const GridIndex &offset : neighborOffsets) {
            TryOpenNeighbor(entry.index, offset, goal, grid, state);
        }
    }
    return {};
}

std::vector<PathPlannerPoint> CellsToWaypoints(
    const std::vector<GridIndex> &cells, const PathPlannerRequest &request,
    const PlanningGrid &grid)
{
    std::vector<PathPlannerPoint> waypoints;
    waypoints.reserve(cells.size());
    for (const GridIndex &cell : cells) {
        waypoints.push_back(CellCenter(cell, grid.voxelSizeM));
    }
    if (!waypoints.empty()) {
        waypoints.front() = request.start;
        waypoints.back() = request.goal;
    }
    return waypoints;
}

PathPlannerPoint Interpolate(const PathPlannerPoint &start,
                             const PathPlannerPoint &end, float t)
{
    return {start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t};
}

bool SegmentClear(const PathPlannerPoint &start, const PathPlannerPoint &end,
                  const PlanningGrid &grid)
{
    const float stepM = std::max(0.05f, grid.voxelSizeM * 0.5f);
    const int steps = std::max(1, static_cast<int>(
                                      std::ceil(Distance(start, end) / stepM)));
    for (int index = 0; index <= steps; ++index) {
        const float t = static_cast<float>(index) / static_cast<float>(steps);
        if (IsBlocked(CellForPoint(Interpolate(start, end, t),
                                   grid.voxelSizeM),
                      grid)) {
            return false;
        }
    }
    return true;
}

size_t FindFarthestVisible(size_t anchor,
                           const std::vector<PathPlannerPoint> &points,
                           const PlanningGrid &grid)
{
    for (size_t candidate = points.size() - 1; candidate > anchor;
         --candidate) {
        if (SegmentClear(points[anchor], points[candidate], grid)) {
            return candidate;
        }
    }
    return anchor + 1;
}

std::vector<PathPlannerPoint> SmoothWaypoints(
    const std::vector<PathPlannerPoint> &points, const PlanningGrid &grid)
{
    if (points.size() < 3) {
        return points;
    }
    std::vector<PathPlannerPoint> smoothed;
    smoothed.push_back(points.front());
    size_t anchor = 0;
    while (anchor + 1 < points.size()) {
        const size_t next = FindFarthestVisible(anchor, points, grid);
        smoothed.push_back(points[next]);
        anchor = next;
    }
    return smoothed;
}

bool ValidateRequest(const PathPlannerRequest &request,
                     PathPlannerResult &result)
{
    if (!IsFinitePoint(request.start) || !IsFinitePoint(request.goal)) {
        result.reason = "path planning failed: non-finite start or goal";
        return false;
    }
    if (Distance(request.start, request.goal) < 0.05f) {
        result.success = true;
        result.reason = "path planning skipped: goal already reached";
        result.waypoints = {request.start};
        return false;
    }
    return true;
}

PathPlannerResult BlockedResult(const char *reason)
{
    PathPlannerResult result{};
    result.reason = reason;
    return result;
}

} // namespace

PathPlannerResult FastPlannerGridPathPlannerPlugin::PlanPath(
    const PathPlannerRequest &request) const
{
    PathPlannerResult result{};
    if (!ValidateRequest(request, result)) {
        return result;
    }
    const PathPlannerConfig config = SanitizeConfig(request.config);
    const PlanningGrid grid = BuildPlanningGrid(request, config);
    const GridIndex start = CellForPoint(request.start, grid.voxelSizeM);
    const GridIndex goal = CellForPoint(request.goal, grid.voxelSizeM);
    if (IsBlocked(start, grid)) {
        return BlockedResult("path planning failed: start occupied");
    }
    if (IsBlocked(goal, grid)) {
        return BlockedResult("path planning failed: goal occupied");
    }
    const GridSearchResult search =
        SearchGrid(start, goal, grid, config.maxExpandedNodes);
    if (!search.success) {
        return BlockedResult("path planning failed: no grid path");
    }
    result.success = true;
    result.reason = "planned with fast-planner grid plugin";
    result.expandedNodes = search.expandedNodes;
    result.waypoints = SmoothWaypoints(
        CellsToWaypoints(search.cells, request, grid), grid);
    return result;
}

} // namespace SmartDrone::Core::Application
