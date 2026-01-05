const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
const utils = @import("utils.zig");
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

pub const FinalPosition = struct {
    arrival_time: i32,
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

pub fn isValidMove(curr: ScreenPos, next: ScreenPos, obstacleGrid: *const ObstacleGrid, gridSize: i32) bool {
    const neigh_height = obstacleGrid.obstacles.get(map.screenToGridCoord(next, gridSize)) orelse 0;
    const too_high = neigh_height >= 4;

    const height = obstacleGrid.obstacles.get(map.screenToGridCoord(curr, gridSize)) orelse 0;
    const too_steep = @abs(height - neigh_height) > 1;

    const good_diagonal = map.canMoveDiagonal(curr, next, gridSize, obstacleGrid);

    const neighborSquare = grid.getSquareInGrid(gridSize, next);
    const out_of_bounds = neighborSquare.x < 0 or neighborSquare.y < 0; // could be expanded

    return !too_high and !too_steep and good_diagonal and !out_of_bounds;
}

// occupiedMap could likely switch to a more efficient datastructure
pub fn getPathAstar(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, obstacleGrid: *const ObstacleGrid, allocator: std.mem.Allocator, maxPathLen: usize, occupiedMap: std.AutoHashMap(ScreenPos, std.AutoHashMap(i32, void)), finalPositions: std.AutoHashMap(ScreenPos, FinalPosition)) !std.ArrayList(ScreenPos) {
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

    var timeLookup = std.AutoHashMap(ScreenPos, i32).init(allocator);
    defer timeLookup.deinit();

    try timeLookup.put(start, 0);

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

            if (!isValidMove(current, neighbor, obstacleGrid, gridSize)) continue;

            // in order to collide with occupied map, need to be at same place same time as another
            if (occupiedMap.get(neighbor)) |times| {
                if (timeLookup.get(current)) |time| {
                    if (times.get(time + 1)) |_| {
                        continue;
                    }
                }
            }

            // check finalPositions for permanent occupancy (agents that have reached their destination)
            if (finalPositions.get(neighbor)) |final| {
                if (timeLookup.get(current)) |time| {
                    if (time + 1 >= final.arrival_time) {
                        continue;
                    }
                }
            }
            const tentativeG = (gScore.get(current) orelse std.math.maxInt(u32)) + 1;

            if (tentativeG < (gScore.get(neighbor) orelse std.math.maxInt(u32))) {
                try cameFrom.put(neighbor, current);

                try timeLookup.put(neighbor, timeLookup.get(current).? + 1);

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

// can clean up constants
pub fn getGroupGoals(obstacles: *ObstacleGrid, goal: ScreenPos, count: i32, gridSize: i32, allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
    var goals = std.ArrayList(ScreenPos).empty;

    var taken = std.AutoHashMap(ScreenPos, void).init(allocator);

    // should be a bfs, so a queue not a stack
    var toVisit = utils.Queue(ScreenPos).init(allocator);
    defer toVisit.deinit();

    try toVisit.enqueue(goal);
    try taken.put(goal, {});

    // bfs to grab from the nearby
    while (goals.items.len < count) {
        const curr = toVisit.dequeue().?;

        // use movement iteration
        for (crossDiagonalMovement) |move| {
            const movedPoint: ScreenPos = .{ .x = curr.x + move[0] * gridSize, .y = curr.y + move[1] * gridSize };

            // no limits right now
            if (taken.get(movedPoint)) |_| {} else {
                try taken.put(movedPoint, {});
                try toVisit.enqueue(movedPoint);
            }
        }

        if (obstacles.obstacles.get(map.screenToGridCoord(curr, gridSize))) |height| {
            if (height >= 4) {
                continue;
            }
        }

        try goals.append(allocator, curr);
    }

    return goals;
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
