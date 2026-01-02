const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
pub const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

const ScreenPos = types.ScreenPos;
const ObstacleGrid = map.ObstacleGrid;

const Agent = types.Agent;

pub const crossMovement: [4][2]i32 = .{ .{ 0, 1 }, .{ 1, 0 }, .{ 0, -1 }, .{ -1, 0 } };
pub const crossDiagonalMovement: [8][2]i32 = .{ .{ 1, 1 }, .{ -1, -1 }, .{ 1, -1 }, .{ -1, 1 }, .{ 0, 1 }, .{ 0, -1 }, .{ 1, 0 }, .{ -1, 0 } };

/// Returns true if an agent can stand on this cell (height < 3)
pub fn isValidStandingCell(pos: ScreenPos, gridSize: i32, obstacleGrid: *const ObstacleGrid) bool {
    const height = obstacleGrid.obstacles.get(map.screenToGridCoord(pos, gridSize)) orelse 0;
    return height < 3;
}

/// Returns true if an agent can move from `from` to `to`
/// Checks: destination is valid standing cell, height difference <= 1, diagonal movement rules
pub fn isValidMove(from: ScreenPos, to: ScreenPos, gridSize: i32, obstacleGrid: *const ObstacleGrid) bool {
    const fromHeight = obstacleGrid.obstacles.get(map.screenToGridCoord(from, gridSize)) orelse 0;
    const toHeight = obstacleGrid.obstacles.get(map.screenToGridCoord(to, gridSize)) orelse 0;

    // Can't stand on height >= 3
    if (toHeight >= 3) return false;

    // Can't climb/descend more than 1 height level
    if (@abs(fromHeight - toHeight) > 1) return false;

    // Check diagonal movement rules
    if (!map.canMoveDiagonal(from, to, gridSize, obstacleGrid)) return false;

    return true;
}

/// Compute angle in radians from dx, dy (-pi to pi)
fn computeAngle(dx: i32, dy: i32) f32 {
    return std.math.atan2(@as(f32, @floatFromInt(dy)), @as(f32, @floatFromInt(dx)));
}

/// Returns smallest absolute difference between two angles (handles wrap-around)
fn angleDifference(a: f32, b: f32) f32 {
    var diff = a - b;
    while (diff > std.math.pi) diff -= 2.0 * std.math.pi;
    while (diff < -std.math.pi) diff += 2.0 * std.math.pi;
    return @abs(diff);
}

const Node = struct {
    pos: ScreenPos,
    g_score: u32,
    f_score: u32,
    parent: ?*Node,
};

pub fn getScore(start: ScreenPos, end: ScreenPos) u32 {
    return @abs(start.x - end.x) + @abs(start.y - end.y);
}

pub fn getPathGreedy(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
    std.debug.assert(@mod(start.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(start.y, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.y, gridSize) == @divFloor(gridSize, 2));

    var pathList = std.ArrayList(ScreenPos).empty;
    var current = start;
    var next = start;
    const maxLen = 20;

    while (!std.meta.eql(current, end)) {
        if (pathList.items.len > maxLen) {
            break;
        }
        try pathList.append(allocator, current);

        var bestMovement = movement[0];
        var bestScore = getScore(current, end);
        for (movement) |move| {
            const movedPoint: ScreenPos = .{ .x = current.x + move[0] * gridSize, .y = current.y + move[1] * gridSize };
            const score = getScore(movedPoint, end);
            if (score < bestScore) {
                bestMovement = move;
                bestScore = score;
            }
        }

        next.x += bestMovement[0] * gridSize;
        next.y += bestMovement[1] * gridSize;
        current = next;
    }
    try pathList.append(allocator, end);

    return pathList;
}

pub fn getPathAstar(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, obstacleGrid: *const ObstacleGrid, allocator: std.mem.Allocator, maxPathLen: usize) !std.ArrayList(ScreenPos) {
    std.debug.assert(@mod(start.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(start.y, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.y, gridSize) == @divFloor(gridSize, 2));

    var openSet = std.ArrayList(ScreenPos).empty;
    defer openSet.deinit(allocator);

    var cameFrom = std.AutoHashMap(ScreenPos, ScreenPos).init(allocator);
    defer cameFrom.deinit();

    var gScore = std.AutoHashMap(ScreenPos, u32).init(allocator);
    defer gScore.deinit();

    var fScore = std.AutoHashMap(ScreenPos, u32).init(allocator);
    defer fScore.deinit();

    try openSet.append(allocator, start);
    try gScore.put(start, 0);
    try fScore.put(start, getScore(start, end));

    const max_iters: usize = 10000;
    var iters: usize = 0;

    while (openSet.items.len > 0 and iters < max_iters) {
        iters += 1;
        var currentIdx: usize = 0;
        var minF: u32 = std.math.maxInt(u32);

        for (openSet.items, 0..) |pos, i| {
            const f = fScore.get(pos) orelse minF;
            if (f < minF) {
                minF = f;
                currentIdx = i;
            }
        }

        const current = openSet.orderedRemove(currentIdx);

        if (std.meta.eql(current, end)) {
            var path = try std.ArrayList(ScreenPos).initCapacity(allocator, maxPathLen);
            errdefer path.deinit(allocator);
            var reconstructCurrent = current;
            while (cameFrom.get(reconstructCurrent)) |parent| {
                path.appendAssumeCapacity(reconstructCurrent);
                reconstructCurrent = parent;
            }
            path.appendAssumeCapacity(start);

            std.mem.reverse(ScreenPos, path.items);
            return path;
        }

        for (movement) |move| {
            const neighbor: ScreenPos = .{
                .x = current.x + move[0] * gridSize,
                .y = current.y + move[1] * gridSize,
            };

            if (!isValidMove(current, neighbor, gridSize, obstacleGrid)) continue;

            const neighborSquare = grid.getSquareInGrid(gridSize, neighbor);
            if (neighborSquare.x < 0 or neighborSquare.y < 0) continue;

            const tentativeG = (gScore.get(current) orelse std.math.maxInt(u32)) + 1;

            if (tentativeG < (gScore.get(neighbor) orelse std.math.maxInt(u32))) {
                try cameFrom.put(neighbor, current);
                try gScore.put(neighbor, tentativeG);
                try fScore.put(neighbor, tentativeG + getScore(neighbor, end));

                var found = false;
                for (openSet.items) |pos| {
                    if (std.meta.eql(pos, neighbor)) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    try openSet.append(allocator, neighbor);
                }
            }
        }
    }

    return error.NoPathFound;
}

pub fn drawPathLines(path: []ScreenPos) void {
    if (path.len < 2) return;

    for (path, 0..) |pos, i| {
        if (i == 0) {
            rl.DrawLine(pos.x, pos.y, path[i + 1].x, path[i + 1].y, rl.BLUE);
        } else {
            rl.DrawLine(pos.x, pos.y, path[i - 1].x, path[i - 1].y, rl.BLUE);
        }
    }
}

/// BFS from target to collect up to `count` valid standing cells.
/// Returns cells in order of distance from target.
fn gatherAvailableCells(
    targetPos: ScreenPos,
    count: usize,
    gridSize: i32,
    obstacleGrid: *const ObstacleGrid,
    maxSearchRadius: i32,
    allocator: std.mem.Allocator,
) !std.ArrayList(ScreenPos) {
    var result = std.ArrayList(ScreenPos).empty;
    errdefer result.deinit(allocator);

    var queue = std.ArrayList(ScreenPos).empty;
    defer queue.deinit(allocator);

    var visited = std.AutoHashMap(ScreenPos, void).init(allocator);
    defer visited.deinit();

    // Snap target to grid center
    const targetSquare = grid.getSquareInGrid(gridSize, targetPos);
    const targetCenter = grid.getSquareCenter(gridSize, targetSquare);

    try queue.append(allocator, targetCenter);
    try visited.put(targetCenter, {});

    const maxIters: usize = 1000;
    var iters: usize = 0;

    while (queue.items.len > 0 and iters < maxIters and result.items.len < count) {
        iters += 1;
        const current = queue.orderedRemove(0);

        // Check distance from target (in grid cells)
        const distX: i32 = @intCast(@divFloor(@abs(current.x - targetCenter.x), @as(u32, @intCast(gridSize))));
        const distY: i32 = @intCast(@divFloor(@abs(current.y - targetCenter.y), @as(u32, @intCast(gridSize))));
        if (distX > maxSearchRadius or distY > maxSearchRadius) {
            continue;
        }

        // If valid standing cell, add to result
        if (current.x >= 0 and current.y >= 0 and
            isValidStandingCell(current, gridSize, obstacleGrid))
        {
            try result.append(allocator, current);
        }

        // Add neighbors to queue
        for (crossMovement) |move| {
            const neighbor: ScreenPos = .{
                .x = current.x + move[0] * gridSize,
                .y = current.y + move[1] * gridSize,
            };

            if (!visited.contains(neighbor)) {
                try visited.put(neighbor, {});
                try queue.append(allocator, neighbor);
            }
        }
    }

    return result;
}

/// Given a target position and a slice of agents, compute a goal position for each
/// selected agent using direction-aware packing.
///
/// Agents are packed as tightly as possible around the target, but cells are assigned
/// based on relative direction: an agent that is "north" of the group's centroid will
/// get a cell that is "north" of the target (if available).
///
/// Returns an ArrayList of ScreenPos with exactly one entry per selected agent,
/// in the order they appear in the agents slice.
pub fn findGroupGoals(
    targetPos: ScreenPos,
    agents: []const Agent,
    gridSize: i32,
    obstacleGrid: *const ObstacleGrid,
    allocator: std.mem.Allocator,
    maxSearchRadius: i32,
) !std.ArrayList(ScreenPos) {
    var result = std.ArrayList(ScreenPos).empty;
    errdefer result.deinit(allocator);

    // Count selected agents and calculate centroid
    var selectedCount: usize = 0;
    var centroidX: i64 = 0;
    var centroidY: i64 = 0;

    for (agents) |agent| {
        if (agent.selected) {
            selectedCount += 1;
            centroidX += agent.pos.x;
            centroidY += agent.pos.y;
        }
    }

    if (selectedCount == 0) {
        return result;
    }

    centroidX = @divFloor(centroidX, @as(i64, @intCast(selectedCount)));
    centroidY = @divFloor(centroidY, @as(i64, @intCast(selectedCount)));

    // Snap target to grid center
    const targetSquare = grid.getSquareInGrid(gridSize, targetPos);
    const targetCenter = grid.getSquareCenter(gridSize, targetSquare);

    // Gather available cells near target (more than needed for flexibility)
    var availableCells = try gatherAvailableCells(
        targetCenter,
        selectedCount * 4,
        gridSize,
        obstacleGrid,
        maxSearchRadius,
        allocator,
    );
    defer availableCells.deinit(allocator);

    // Track which cells have been assigned
    var assignedCells = std.AutoHashMap(ScreenPos, void).init(allocator);
    defer assignedCells.deinit();

    // For each selected agent, find the best matching cell
    for (agents) |agent| {
        if (!agent.selected) continue;

        // Compute agent's direction from centroid
        const agentDx = agent.pos.x - @as(i32, @intCast(centroidX));
        const agentDy = agent.pos.y - @as(i32, @intCast(centroidY));

        // Check if agent is at centroid (or very close)
        const atCentroid = (@abs(agentDx) < gridSize and @abs(agentDy) < gridSize);
        const agentAngle = computeAngle(agentDx, agentDy);

        // Find best matching cell
        var bestCell: ?ScreenPos = null;
        var bestScore: f32 = std.math.inf(f32);

        for (availableCells.items) |cell| {
            if (assignedCells.contains(cell)) continue;

            // Compute cell's direction from target
            const cellDx = cell.x - targetCenter.x;
            const cellDy = cell.y - targetCenter.y;
            const cellAtTarget = (cellDx == 0 and cellDy == 0);

            // Score = angular difference + small distance penalty for tie-breaking
            var score: f32 = 0;

            if (atCentroid) {
                // Agent at centroid prefers cells closest to target
                const dist = @as(f32, @floatFromInt(@abs(cellDx) + @abs(cellDy)));
                score = dist;
            } else if (cellAtTarget) {
                // Target cell has no angle - give it a moderate score
                // (center-most agents will claim it due to their low scores)
                score = std.math.pi;
            } else {
                const cellAngle = computeAngle(cellDx, cellDy);
                const angularDiff = angleDifference(agentAngle, cellAngle);

                // Distance from target as tie-breaker (normalized)
                const dist = @as(f32, @floatFromInt(@abs(cellDx) + @abs(cellDy)));
                const distPenalty = dist / @as(f32, @floatFromInt(gridSize * 100));

                score = angularDiff + distPenalty;
            }

            if (score < bestScore) {
                bestScore = score;
                bestCell = cell;
            }
        }

        // Assign cell (or fallback to agent's current position)
        if (bestCell) |cell| {
            try assignedCells.put(cell, {});
            try result.append(allocator, cell);
        } else {
            // Fallback: use agent's current position
            const agentSquare = grid.getSquareInGrid(gridSize, agent.pos);
            const agentCenter = grid.getSquareCenter(gridSize, agentSquare);
            try result.append(allocator, agentCenter);
        }
    }

    return result;
}
