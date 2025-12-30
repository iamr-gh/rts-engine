//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.

pub const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

const std = @import("std");

const ScreenPos = struct {
    x: c_int,
    y: c_int,
};

const GridCoord = struct {
    x: i32,
    y: i32,
};

const ObstacleGrid = struct {
    obstacles: std.AutoHashMap(GridCoord, void),

    fn init(allocator: std.mem.Allocator) ObstacleGrid {
        return .{
            .obstacles = std.AutoHashMap(GridCoord, void).init(allocator),
        };
    }

    fn deinit(self: *ObstacleGrid) void {
        self.obstacles.deinit();
    }

    fn add(self: *ObstacleGrid, coord: GridCoord) !void {
        try self.obstacles.put(coord, {});
    }

    fn remove(self: *ObstacleGrid, coord: GridCoord) void {
        _ = self.obstacles.remove(coord);
    }

    fn toggle(self: *ObstacleGrid, coord: GridCoord) !void {
        if (self.obstacles.fetchRemove(coord)) |_| {
            // was there, now removed
        } else {
            try self.obstacles.put(coord, {});
        }
    }

    fn contains(self: *const ObstacleGrid, coord: GridCoord) bool {
        return self.obstacles.contains(coord);
    }
};

fn generateRandomObstacles(allocator: std.mem.Allocator, gridSize: i32, screenWidth: i32, screenHeight: i32, excludePos: ScreenPos) !ObstacleGrid {
    var rng = std.Random.DefaultPrng.init(@intCast(std.time.timestamp()));
    const random = rng.random();

    var obstacleGrid = ObstacleGrid.init(allocator);
    errdefer obstacleGrid.deinit();

    const numCols = @divFloor(screenWidth, gridSize);
    const numRows = @divFloor(screenHeight, gridSize);

    const numColsUsize: usize = @intCast(numCols);
    const numRowsUsize: usize = @intCast(numRows);

    for (0..numColsUsize) |col| {
        for (0..numRowsUsize) |row| {
            const gridX = @as(i32, @intCast(col)) * gridSize;
            const gridY = @as(i32, @intCast(row)) * gridSize;
            const center = getSquareCenter(gridSize, .{ .x = gridX, .y = gridY });

            if (std.meta.eql(center, excludePos)) continue;

            if (random.float(f32) < 0.2) {
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
            }
        }
    }

    return obstacleGrid;
}

fn isObstacle(obstacleGrid: *const ObstacleGrid, screenPos: ScreenPos, gridSize: i32) bool {
    const gridSquare = getSquareInGrid(gridSize, screenPos);
    const coord = GridCoord{
        .x = @divFloor(gridSquare.x, gridSize),
        .y = @divFloor(gridSquare.y, gridSize),
    };
    return obstacleGrid.contains(coord);
}

fn canMoveDiagonal(from: ScreenPos, to: ScreenPos, gridSize: i32, obstacleGrid: *const ObstacleGrid) bool {
    const dx = to.x - from.x;
    const dy = to.y - from.y;

    const dxGrid = @divFloor(dx, gridSize);
    const dyGrid = @divFloor(dy, gridSize);

    if (dxGrid == 0 or dyGrid == 0) {
        return true;
    }

    const intermediate1 = ScreenPos{
        .x = from.x + dxGrid * gridSize,
        .y = from.y,
    };
    const intermediate2 = ScreenPos{
        .x = from.x,
        .y = from.y + dyGrid * gridSize,
    };

    return !isObstacle(obstacleGrid, intermediate1, gridSize) and !isObstacle(obstacleGrid, intermediate2, gridSize);
}

fn drawObstacles(obstacleGrid: *const ObstacleGrid, gridSize: i32) void {
    var iter = obstacleGrid.obstacles.iterator();
    while (iter.next()) |entry| {
        const coord = entry.key_ptr.*;
        const x = coord.x * gridSize;
        const y = coord.y * gridSize;
        rl.DrawRectangle(x, y, gridSize, gridSize, rl.DARKGRAY);
    }
}

fn printGrid(inc: i32, start: i32, end: i32) void {
    var i: i32 = start;

    // only works with square resolution rn
    while (i < end) : (i += inc) {
        // vertical line
        rl.DrawLine(i, start, i, end, rl.GRAY);
        // horizontal line
        rl.DrawLine(start, i, end, i, rl.GRAY);
    }
}

fn getSquareInGrid(gridSize: i32, pos: ScreenPos) ScreenPos {
    const leftX = @divFloor(pos.x, gridSize) * gridSize;
    const topY = @divFloor(pos.y, gridSize) * gridSize;
    return .{ .x = leftX, .y = topY };
}

fn getSquareCenter(gridSize: i32, pos: ScreenPos) ScreenPos {
    return .{ .x = pos.x + @divFloor(gridSize, 2), .y = pos.y + @divFloor(gridSize, 2) };
}

fn drawPathLines(path: []ScreenPos) void {
    if (path.len < 2) return;

    for (path, 0..) |pos, i| {
        if (i == 0) {
            rl.DrawLine(pos.x, pos.y, path[i + 1].x, path[i + 1].y, rl.BLUE);
        } else {
            rl.DrawLine(pos.x, pos.y, path[i - 1].x, path[i - 1].y, rl.BLUE);
        }
    }
}

// need to construct graph representation of grid points
// it's about imposing a graph upon the grid
// I think graph is formed from adj list

const crossMovement: [4][2]i32 = .{ .{ 0, 1 }, .{ 1, 0 }, .{ 0, -1 }, .{ -1, 0 } };
const crossDiagonalMovement: [8][2]i32 = .{ .{ 1, 1 }, .{ -1, -1 }, .{ 1, -1 }, .{ -1, 1 }, .{ 0, 1 }, .{ 0, -1 }, .{ 1, 0 }, .{ -1, 0 } };

const Node = struct {
    pos: ScreenPos,
    g_score: u32,
    f_score: u32,
    parent: ?*Node,
};

fn getScore(start: ScreenPos, end: ScreenPos) u32 {
    return @abs(start.x - end.x) + @abs(start.y - end.y);
}

fn getPathGreedy(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
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

fn getPathAstar(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, obstacleGrid: *const ObstacleGrid, allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
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

    while (openSet.items.len > 0) {
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

            if (isObstacle(obstacleGrid, neighbor, gridSize)) continue;

            if (!canMoveDiagonal(current, neighbor, gridSize, obstacleGrid)) continue;

            const neighborSquare = getSquareInGrid(gridSize, neighbor);
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

pub fn main() !void {
    const allocator = std.heap.page_allocator; // consider changing to continguous
    rl.SetTraceLogLevel(rl.LOG_DEBUG);

    // important on macOS
    rl.SetConfigFlags(rl.FLAG_WINDOW_HIGHDPI);
    rl.SetConfigFlags(rl.FLAG_WINDOW_RESIZABLE);

    const screenWidth: i32 = 1000;
    const screenHeight: i32 = 1000;
    rl.InitWindow(screenWidth, screenHeight, "rts engine testing");
    defer rl.CloseWindow();

    const fps: i32 = 60;
    rl.SetTargetFPS(fps);
    rl.SetWindowPosition(0, 0);

    // const vel: i32 = 2; // pixels per frame

    const centerX: i32 = screenWidth / 2;
    const centerY: i32 = screenHeight / 2;

    const agentSpeed: f32 = 200;

    var clickedPt: ScreenPos = .{ .x = centerX, .y = centerY };
    var agentPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 50;
    const gridChange: i32 = 10;

    agentPt = getSquareCenter(gridSize, getSquareInGrid(gridSize, clickedPt));

    var path: ?std.ArrayList(ScreenPos) = null;
    var previousTarget = agentPt;

    var obstacleGrid = try generateRandomObstacles(allocator, gridSize, screenWidth, screenHeight, agentPt);
    var prevGridSize = gridSize;

    std.debug.assert(@mod(gridSize, 2) == 0);
    std.debug.assert(@mod(gridChange, 2) == 0);

    var end = false;
    while (!end and !rl.WindowShouldClose()) {
        if (rl.IsKeyDown(rl.KEY_W) and rl.IsKeyDown(rl.KEY_LEFT_SUPER)) {
            end = true;
        }

        rl.BeginDrawing();
        defer rl.EndDrawing();

        rl.ClearBackground(rl.RAYWHITE);

        // scale grid
        if (rl.IsKeyPressed(rl.KEY_LEFT)) {
            gridSize -= gridChange;
        } else if (rl.IsKeyPressed(rl.KEY_RIGHT))
            gridSize += gridChange;

        gridSize = @max(gridSize, 5);
        gridSize = @min(gridSize, 100);

        if (gridSize != prevGridSize) {
            obstacleGrid.deinit();
            obstacleGrid = try generateRandomObstacles(allocator, gridSize, screenWidth, screenHeight, agentPt);
            if (path) |*p| p.deinit(allocator);
            path = null;
            prevGridSize = gridSize;
        }

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            clickedPt.x = rl.GetMouseX();
            clickedPt.y = rl.GetMouseY();
        }

        // highlight square of grid containing target
        const gridSquare = getSquareInGrid(gridSize, clickedPt);
        const gridSquareCenter = getSquareCenter(gridSize, gridSquare);

        if (rl.IsMouseButtonPressed(rl.MOUSE_RIGHT_BUTTON)) {
            const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };
            const rightGridSquare = getSquareInGrid(gridSize, mousePos);
            const coord = GridCoord{
                .x = @divFloor(rightGridSquare.x, gridSize),
                .y = @divFloor(rightGridSquare.y, gridSize),
            };
            try obstacleGrid.toggle(coord);
            if (!isObstacle(&obstacleGrid, gridSquareCenter, gridSize)) {
                if (path) |*p| p.deinit(allocator);
                path = getPathAstar(agentPt, gridSquareCenter, gridSize, &crossDiagonalMovement, &obstacleGrid, allocator) catch null;
                if (path) |*p| {
                    if (p.items.len > 0) _ = p.orderedRemove(0);
                }
            }
        }

        if (isObstacle(&obstacleGrid, gridSquareCenter, gridSize)) {
            rl.DrawRectangle(gridSquare.x, gridSquare.y, gridSize, gridSize, rl.RED);
        } else {
            rl.DrawRectangle(gridSquare.x, gridSquare.y, gridSize, gridSize, rl.GREEN);
        }

        if (!std.meta.eql(gridSquareCenter, previousTarget)) {
            if (path) |*p| p.deinit(allocator);
            const agentSquare = getSquareInGrid(gridSize, agentPt);
            agentPt = getSquareCenter(gridSize, agentSquare);
            if (isObstacle(&obstacleGrid, gridSquareCenter, gridSize)) {
                path = null;
            } else {
                path = getPathAstar(agentPt, gridSquareCenter, gridSize, &crossDiagonalMovement, &obstacleGrid, allocator) catch null;
                if (path) |*p| {
                    if (p.items.len > 0) _ = p.orderedRemove(0);
                }
            }
            previousTarget = gridSquareCenter;
        }

        if (path) |*p| {
            if (p.items.len > 0) {
                const deltaTime = rl.GetFrameTime();
                const nextPoint = p.items[0];
                const dx: f32 = @floatFromInt(nextPoint.x - agentPt.x);
                const dy: f32 = @floatFromInt(nextPoint.y - agentPt.y);
                const distance = std.math.sqrt(dx * dx + dy * dy);

                if (distance > 0) {
                    const moveAmount = agentSpeed * deltaTime;
                    if (moveAmount >= distance) {
                        agentPt = nextPoint;
                        _ = p.orderedRemove(0);
                        if (p.items.len == 0) {
                            p.deinit(allocator);
                            path = null;
                        }
                    } else {
                        const ratio = moveAmount / distance;
                        agentPt.x += @as(i32, @intFromFloat(dx * ratio));
                        agentPt.y += @as(i32, @intFromFloat(dy * ratio));
                    }
                }
            }
        }

        printGrid(gridSize, 0, @max(screenWidth, screenHeight));

        drawObstacles(&obstacleGrid, gridSize);

        if (path) |*p| {
            drawPathLines(p.items);
        }

        rl.DrawCircle(agentPt.x, agentPt.y, 10, rl.RED);
    }

    obstacleGrid.deinit();
    if (path) |*p| p.deinit(allocator);
}
