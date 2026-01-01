const std = @import("std");
const types = @import("types.zig");
const grid = @import("grid.zig");
const map = @import("map.zig");
const pathfinding = @import("pathfinding.zig");

const rl = grid.rl;
const ScreenPos = types.ScreenPos;
const GridCoord = types.GridCoord;
const Agent = types.Agent;
const AgentList = types.AgentList;
const ObstacleGrid = map.ObstacleGrid;
const PathNode = types.PathNode;
const TimedPath = types.TimedPath;
const Reservation = types.Reservation;

const AGENT_RADIUS: i32 = 10;
pub const AGENT_SELECT_RADIUS: i32 = 12;

pub fn initAgents(allocator: std.mem.Allocator, count: usize, startPos: ScreenPos, gridSize: i32) !AgentList {
    var agents = std.ArrayList(Agent).empty;
    try agents.ensureTotalCapacityPrecise(allocator, count);
    errdefer agents.deinit(allocator);

    for (0..count) |i| {
        const offsetRow = @as(i32, @intCast(@divFloor(i, 3))) - 1;
        const offsetCol = @as(i32, @intCast(i % 3)) - 1;

        const centerGrid = grid.getSquareInGrid(gridSize, startPos);
        const centerCoord = GridCoord{
            .x = @divFloor(centerGrid.x, gridSize),
            .y = @divFloor(centerGrid.y, gridSize),
        };

        const agentCoord = GridCoord{
            .x = centerCoord.x + offsetCol,
            .y = centerCoord.y + offsetRow,
        };

        const gridPos = ScreenPos{ .x = agentCoord.x * gridSize, .y = agentCoord.y * gridSize };
        const agentPos = grid.getSquareCenter(gridSize, gridPos);

        const agent = Agent{
            .id = i,
            .pos = agentPos,
            .selected = false,
            .path = null,
            .destination = null,
        };
        try agents.append(allocator, agent);
    }

    return AgentList{
        .agents = agents,
        .nextId = count,
    };
}

pub fn updateAgentPositions(agentList: *AgentList, deltaTime: f32, speed: f32, gridSize: i32) void {
    const timeStepDuration: f32 = @as(f32, @floatFromInt(gridSize)) / speed;

    for (agentList.agents.items) |*agent| {
        if (agent.path == null or agent.path.?.completed) continue;

        const path = &agent.path.?;

        if (path.currentWaitRemaining > 0) {
            path.currentWaitRemaining -= deltaTime;
            continue;
        }

        if (path.currentNodeIndex >= path.nodes.items.len) {
            path.completed = true;
            continue;
        }

        const nextNode = path.nodes.items[path.currentNodeIndex];
        const currentPos = agent.pos;

        const dx: f32 = @floatFromInt(nextNode.pos.x - currentPos.x);
        const dy: f32 = @floatFromInt(nextNode.pos.y - currentPos.y);
        const distance = std.math.sqrt(dx * dx + dy * dy);

        if (distance <= 1.0) {
            agent.pos = nextNode.pos;
            path.currentNodeIndex += 1;

            if (path.currentNodeIndex >= path.nodes.items.len) {
                path.completed = true;
            } else {
                path.currentWaitRemaining = timeStepDuration;
            }
        } else {
            const moveAmount = speed * deltaTime;
            if (moveAmount >= distance) {
                agent.pos = nextNode.pos;
                path.currentWaitRemaining = timeStepDuration;
            } else {
                const ratio = moveAmount / distance;
                agent.pos.x += @as(i32, @intFromFloat(dx * ratio));
                agent.pos.y += @as(i32, @intFromFloat(dy * ratio));
            }
        }
    }
}

pub fn drawAgents(agentList: *const AgentList) void {
    for (agentList.agents.items) |*agent| {
        if (agent.selected) {
            rl.DrawCircleLines(agent.pos.x, agent.pos.y, AGENT_SELECT_RADIUS + 2, rl.GREEN);
        }
        rl.DrawCircle(agent.pos.x, agent.pos.y, AGENT_RADIUS, rl.RED);
    }
}

pub fn cleanup(agentList: *AgentList, _allocator: std.mem.Allocator) void {
    for (agentList.agents.items) |*agent| {
        clearAgentPath(agent, _allocator);
    }
    agentList.agents.deinit(_allocator);
}

pub fn getAgentAtPosition(agentList: *const AgentList, pos: ScreenPos, radius: i32) ?usize {
    for (agentList.agents.items, 0..) |agent, i| {
        const dx = agent.pos.x - pos.x;
        const dy = agent.pos.y - pos.y;
        const distSq = dx * dx + dy * dy;
        if (distSq <= radius * radius) {
            return i;
        }
    }
    return null;
}

pub fn setAgentPath(agent: *Agent, path: TimedPath, allocator: std.mem.Allocator) void {
    clearAgentPath(agent, allocator);
    agent.path = path;
}

pub fn clearAgentPath(agent: *Agent, allocator: std.mem.Allocator) void {
    if (agent.path) |*path| {
        path.nodes.deinit(allocator);
    }
    agent.path = null;
}

pub fn clearAllPaths(agentList: *AgentList, allocator: std.mem.Allocator) void {
    for (agentList.agents.items) |*agent| {
        clearAgentPath(agent, allocator);
    }
}

pub fn toggleSelection(agent: *Agent) void {
    agent.selected = !agent.selected;
}

pub fn selectAll(agentList: *AgentList) void {
    for (agentList.agents.items) |*agent| {
        agent.selected = true;
    }
}

pub fn deselectAll(agentList: *AgentList) void {
    for (agentList.agents.items) |*agent| {
        agent.selected = false;
    }
}

pub fn getSelectedCount(agentList: *const AgentList) usize {
    var count: usize = 0;
    for (agentList.agents.items) |agent| {
        if (agent.selected) count += 1;
    }
    return count;
}

pub fn selectAgentsInRect(agentList: *AgentList, x: i32, y: i32, width: i32, height: i32, _radius: i32) void {
    _ = _radius;
    for (agentList.agents.items) |*agent| {
        const agentX = agent.pos.x;
        const agentY = agent.pos.y;
        const inRect = agentX >= x and agentX <= x + width and agentY >= y and agentY <= y + height;
        if (inRect) {
            agent.selected = true;
        }
    }
}

pub fn getSelectedAgents(agentList: *const AgentList) std.ArrayList(usize) {
    var selected = std.ArrayList(usize).empty;
    for (agentList.agents.items, 0..) |agent, i| {
        if (agent.selected) {
            selected.append(std.heap.page_allocator, i) catch {};
        }
    }
    return selected;
}

pub fn drawSelectionBox(start: ScreenPos, end: ScreenPos) void {
    const x = @min(start.x, end.x);
    const y = @min(start.y, end.y);
    const width = @abs(start.x - end.x);
    const height = @abs(start.y - end.y);

    if (width > 0 and height > 0) {
        rl.DrawRectangle(x, y, @as(c_int, @intCast(width)), @as(c_int, @intCast(height)), rl.Fade(rl.BLUE, 0.3));
        rl.DrawRectangleLines(x, y, @as(c_int, @intCast(width)), @as(c_int, @intCast(height)), rl.BLUE);
    }
}

pub fn generateFormationPositions(center: ScreenPos, count: usize, gridSize: i32, obstacleGrid: *const ObstacleGrid, _allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
    _ = _allocator;
    var positions = std.ArrayList(ScreenPos).empty;
    try positions.ensureTotalCapacityPrecise(std.heap.page_allocator, count);
    errdefer positions.deinit(std.heap.page_allocator);

    const centerGrid = grid.getSquareInGrid(gridSize, center);
    const centerCoord = GridCoord{
        .x = @divFloor(centerGrid.x, gridSize),
        .y = @divFloor(centerGrid.y, gridSize),
    };

    var ring: i32 = 0;
    while (positions.items.len < count and ring < 10) : (ring += 1) {
        const sideLen = 2 * ring + 1;

        if (ring == 0) {
            const centerPos = grid.getSquareCenter(gridSize, centerGrid);
            if (!map.isObstacle(obstacleGrid, centerPos, gridSize)) {
                var alreadyExists = false;
                for (positions.items) |pos| {
                    if (std.meta.eql(pos, centerPos)) {
                        alreadyExists = true;
                        break;
                    }
                }
                if (!alreadyExists) {
                    try positions.append(std.heap.page_allocator, centerPos);
                }
            }
        } else {
            var side: usize = 0;
            while (side < 4 and positions.items.len < count) : (side += 1) {
                var i: i32 = 0;
                while (i < sideLen and positions.items.len < count) : (i += 1) {
                    var coord = centerCoord;

                    if (side == 0) {
                        coord.x += ring;
                        coord.y -= ring + i;
                    } else if (side == 1) {
                        coord.x += ring - i;
                        coord.y += ring;
                    } else if (side == 2) {
                        coord.x -= ring;
                        coord.y += ring - i;
                    } else {
                        coord.x -= ring + i;
                        coord.y -= ring;
                    }

                    const gridPos = ScreenPos{ .x = coord.x * gridSize, .y = coord.y * gridSize };
                    const gridCenter = grid.getSquareCenter(gridSize, gridPos);

                    if (!map.isObstacle(obstacleGrid, gridCenter, gridSize)) {
                        var alreadyExists = false;
                        for (positions.items) |pos| {
                            if (std.meta.eql(pos, gridCenter)) {
                                alreadyExists = true;
                                break;
                            }
                        }
                        if (!alreadyExists) {
                            try positions.append(std.heap.page_allocator, gridCenter);
                        }
                    }
                }
            }
        }
    }

    return positions;
}

const PriorityAgent = struct {
    agentIdx: usize,
    priority: u32,
};

fn planAgentPaths(agentList: *AgentList, gridSize: i32, obstacleGrid: *ObstacleGrid, allocator: std.mem.Allocator) !void {
    var reservations = std.AutoHashMap(GridCoord, std.ArrayList(Reservation)).init(allocator);
    defer {
        var iter = reservations.iterator();
        while (iter.next()) |entry| {
            entry.value_ptr.*.deinit(allocator);
        }
        reservations.deinit();
    }

    var movingAgents = std.ArrayList(PriorityAgent).empty;
    defer movingAgents.deinit(allocator);

    for (agentList.agents.items, 0..) |*agent, i| {
        if (agent.destination != null) {
            const startPos = grid.getSquareInGrid(gridSize, agent.pos);
            const startCoord = GridCoord{
                .x = @divFloor(startPos.x, gridSize),
                .y = @divFloor(startPos.y, gridSize),
            };
            const endPos = grid.getSquareInGrid(gridSize, agent.destination.?);
            const endCoord = GridCoord{
                .x = @divFloor(endPos.x, gridSize),
                .y = @divFloor(endPos.y, gridSize),
            };
            const heuristic = @abs(startCoord.x - endCoord.x) + @abs(startCoord.y - endCoord.y);

            try movingAgents.append(allocator, .{
                .agentIdx = i,
                .priority = heuristic,
            });
        }
    }

    std.sort.insertion(PriorityAgent, movingAgents.items, {}, struct {
        fn lessThan(_: void, a: PriorityAgent, b: PriorityAgent) bool {
            return a.priority < b.priority;
        }
    }.lessThan);

    for (movingAgents.items) |pAgent| {
        const agent = &agentList.agents.items[pAgent.agentIdx];

        const maybePath = try pathfinding.findPathWithReservations(
            agent.pos,
            agent.destination.?,
            gridSize,
            obstacleGrid,
            &reservations,
            allocator,
            agent.id,
        );

        if (maybePath) |path| {
            try reservePath(&reservations, path, agent.id, allocator);
            setAgentPath(agent, path, allocator);
        } else {
            clearAgentPath(agent, allocator);
        }
    }
}

pub fn reservePath(reservations: *std.AutoHashMap(GridCoord, std.ArrayList(Reservation)), path: TimedPath, agentId: usize, allocator: std.mem.Allocator) !void {
    for (path.nodes.items) |node| {
        const reservation = Reservation{
            .agentId = agentId,
            .startTime = node.timeStep,
            .endTime = node.timeStep + 1,
        };

        const gop = try reservations.getOrPut(node.gridCoord);
        if (!gop.found_existing) {
            gop.value_ptr.* = std.ArrayList(Reservation).empty;
        }
        try gop.value_ptr.append(allocator, reservation);
    }
}

pub fn assignDestinationsToSelected(agentList: *AgentList, targetCenter: ScreenPos, gridSize: i32, obstacleGrid: *ObstacleGrid, allocator: std.mem.Allocator) !void {
    var selected = getSelectedAgents(agentList);
    defer selected.deinit(std.heap.page_allocator);

    if (selected.items.len == 0) return;

    var candidates = try generateFormationPositions(targetCenter, selected.items.len, gridSize, obstacleGrid, allocator);
    defer candidates.deinit(std.heap.page_allocator);

    for (selected.items, 0..) |agentIdx, i| {
        if (i >= candidates.items.len) break;
        agentList.agents.items[agentIdx].destination = candidates.items[i];
    }

    try planAgentPaths(agentList, gridSize, obstacleGrid, allocator);
}
