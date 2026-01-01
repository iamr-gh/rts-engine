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

pub const crossMovement: [4][2]i32 = .{ .{ 0, 1 }, .{ 1, 0 }, .{ 0, -1 }, .{ -1, 0 } };
pub const crossDiagonalMovement: [8][2]i32 = .{ .{ 1, 1 }, .{ -1, -1 }, .{ 1, -1 }, .{ -1, 1 }, .{ 0, 1 }, .{ 0, -1 }, .{ 1, 0 }, .{ -1, 0 } };

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

pub fn getPathAstar(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, obstacleGrid: *const ObstacleGrid, allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
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
            var path = try std.ArrayList(ScreenPos).initCapacity(allocator, 100);
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

            const neigh_height = obstacleGrid.obstacles.get(map.screenToGridCoord(neighbor, gridSize)) orelse 0;
            if (neigh_height >= 3) continue;
            const height = obstacleGrid.obstacles.get(map.screenToGridCoord(current, gridSize)) orelse 0;
            if (@abs(height - neigh_height) > 1) continue;

            if (!map.canMoveDiagonal(current, neighbor, gridSize, obstacleGrid)) continue;

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
