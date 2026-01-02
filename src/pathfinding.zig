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

// Collision avoidance constants
pub const COLLISION_LOOK_AHEAD: f32 = 24.0; // pixels to look ahead for collisions
pub const DEFAULT_MAX_WAIT_TIME: i64 = 2; // seconds before replanning (configurable)

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

// ============================================================================
// Collision Avoidance Functions
// ============================================================================

/// Check if two positions would cause a collision (distance < minDistance)
pub fn wouldCollide(pos1: rl.Vector2, pos2: rl.Vector2, minDistance: f32) bool {
    const dx = pos1.x - pos2.x;
    const dy = pos1.y - pos2.y;
    return (dx * dx + dy * dy) < (minDistance * minDistance);
}

/// Check if a floating-point position is walkable (not in an obstacle)
pub fn isPositionWalkable(pos: rl.Vector2, gridSize: i32, obstacleGrid: *const ObstacleGrid) bool {
    const screenPos = ScreenPos{
        .x = @intFromFloat(pos.x),
        .y = @intFromFloat(pos.y),
    };
    return isValidStandingCell(screenPos, gridSize, obstacleGrid);
}

/// Rotate a 2D vector by the given angle in degrees
fn rotateVector(v: rl.Vector2, angleDegrees: f32) rl.Vector2 {
    const rad = angleDegrees * std.math.pi / 180.0;
    const cos_val = @cos(rad);
    const sin_val = @sin(rad);
    return .{
        .x = v.x * cos_val - v.y * sin_val,
        .y = v.x * sin_val + v.y * cos_val,
    };
}

/// Calculate the minimum distance between two line segments (p1-p2 and p3-p4)
/// Used to check if two moving agents would cross paths
fn segmentDistance(p1: rl.Vector2, p2: rl.Vector2, p3: rl.Vector2, p4: rl.Vector2) f32 {
    // Check distance at multiple points along both segments
    const steps: usize = 4;
    var minDist: f32 = std.math.inf(f32);

    for (0..steps + 1) |i| {
        const t1 = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(steps));
        const point1 = rl.Vector2{
            .x = p1.x + (p2.x - p1.x) * t1,
            .y = p1.y + (p2.y - p1.y) * t1,
        };

        for (0..steps + 1) |j| {
            const t2 = @as(f32, @floatFromInt(j)) / @as(f32, @floatFromInt(steps));
            const point2 = rl.Vector2{
                .x = p3.x + (p4.x - p3.x) * t2,
                .y = p3.y + (p4.y - p3.y) * t2,
            };

            const dx = point1.x - point2.x;
            const dy = point1.y - point2.y;
            const dist = @sqrt(dx * dx + dy * dy);
            if (dist < minDist) {
                minDist = dist;
            }
        }
    }

    return minDist;
}

/// Predict where an agent will be after deltaTime seconds
fn predictAgentPosition(agent: *const Agent, deltaTime: f32, speed: f32) rl.Vector2 {
    const currentPos = rl.Vector2{
        .x = @floatFromInt(agent.pos.x),
        .y = @floatFromInt(agent.pos.y),
    };

    // If no path, agent stays in place
    if (agent.path == null) return currentPos;
    const path = &agent.path.?;
    if (path.items.len == 0) return currentPos;

    const nextPoint = path.items[0];
    const targetPos = rl.Vector2{
        .x = @floatFromInt(nextPoint.x),
        .y = @floatFromInt(nextPoint.y),
    };

    const dx = targetPos.x - currentPos.x;
    const dy = targetPos.y - currentPos.y;
    const distance = @sqrt(dx * dx + dy * dy);

    if (distance < 0.001) return currentPos;

    const moveAmount = speed * deltaTime;
    if (moveAmount >= distance) {
        return targetPos;
    }

    const ratio = moveAmount / distance;
    return rl.Vector2{
        .x = currentPos.x + dx * ratio,
        .y = currentPos.y + dy * ratio,
    };
}

/// Find a safe movement vector for an agent, avoiding collisions with higher-priority agents and walls.
/// Returns the movement vector to apply, or null if the agent should wait.
///
/// Parameters:
/// - agentIndex: index of the agent being moved in the agents slice
/// - agents: all agents
/// - agentOrder: indices sorted by priority (earlier movers first)
/// - orderPosition: this agent's position in the priority order
/// - desiredDir: normalized direction agent wants to move
/// - speed: agent movement speed
/// - deltaTime: frame time
/// - agentRadius: collision radius
/// - obstacleGrid: for wall collision checking
/// - gridSize: grid cell size
pub fn findSafeMovement(
    agentIndex: usize,
    agents: []const Agent,
    agentOrder: []const usize,
    orderPosition: usize,
    desiredDir: rl.Vector2,
    speed: f32,
    deltaTime: f32,
    agentRadius: f32,
    obstacleGrid: *const ObstacleGrid,
    gridSize: i32,
) ?rl.Vector2 {
    const agent = &agents[agentIndex];
    const currentPos = rl.Vector2{
        .x = @floatFromInt(agent.pos.x),
        .y = @floatFromInt(agent.pos.y),
    };

    const moveDistance = speed * deltaTime;
    const minSeparation = agentRadius * 2.0;

    // Try the desired direction first, then alternative angles
    const angles = [_]f32{ 0, 30, -30, 45, -45, 60, -60, 90, -90, 120, -120, 150, -150 };

    for (angles) |angle| {
        const testDir = rotateVector(desiredDir, angle);
        const testMove = rl.Vector2{
            .x = testDir.x * moveDistance,
            .y = testDir.y * moveDistance,
        };
        const predictedPos = rl.Vector2{
            .x = currentPos.x + testMove.x,
            .y = currentPos.y + testMove.y,
        };

        // Check wall collision
        if (!isPositionWalkable(predictedPos, gridSize, obstacleGrid)) {
            continue;
        }

        // Check collision with higher-priority agents (those earlier in agentOrder)
        var collision = false;
        for (0..orderPosition) |i| {
            const otherIdx = agentOrder[i];
            if (otherIdx == agentIndex) continue;

            const other = &agents[otherIdx];
            const otherPos = rl.Vector2{
                .x = @floatFromInt(other.pos.x),
                .y = @floatFromInt(other.pos.y),
            };

            // Predict where the other agent will be
            const otherPredicted = predictAgentPosition(other, deltaTime, speed);

            // Check collision at predicted positions
            if (wouldCollide(predictedPos, otherPredicted, minSeparation)) {
                collision = true;
                break;
            }

            // Check if paths would cross (segment intersection)
            if (segmentDistance(currentPos, predictedPos, otherPos, otherPredicted) < minSeparation) {
                collision = true;
                break;
            }
        }

        if (!collision) {
            return testMove;
        }
    }

    // No safe movement found
    return null;
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

/// A* pathfinding with additional blocked cells (for replanning around other agents)
/// blockedCells contains positions that should be treated as impassable obstacles
pub fn getPathAstarWithBlockedCells(
    start: ScreenPos,
    end: ScreenPos,
    gridSize: i32,
    movement: []const [2]i32,
    obstacleGrid: *const ObstacleGrid,
    blockedCells: []const ScreenPos,
    allocator: std.mem.Allocator,
    maxPathLen: usize,
) !std.ArrayList(ScreenPos) {
    std.debug.assert(@mod(start.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(start.y, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.y, gridSize) == @divFloor(gridSize, 2));

    // Build a set of blocked cells for O(1) lookup
    var blockedSet = std.AutoHashMap(ScreenPos, void).init(allocator);
    defer blockedSet.deinit();
    for (blockedCells) |cell| {
        try blockedSet.put(cell, {});
    }

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

            // Check if neighbor is in blocked cells
            if (blockedSet.contains(neighbor)) continue;

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
