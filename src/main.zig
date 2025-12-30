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

// manhattan distance
fn getScore(start: ScreenPos, end: ScreenPos) u32 {
    return @abs(start.x - end.x) + @abs(start.y - end.y);
}

// abstraction on pathing algo
fn getPath(start: ScreenPos, end: ScreenPos, gridSize: i32, movement: []const [2]i32, allocator: std.mem.Allocator) !std.ArrayList(ScreenPos) {
    // verify start and end are on grid centers
    std.debug.assert(@mod(start.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(start.y, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.x, gridSize) == @divFloor(gridSize, 2));
    std.debug.assert(@mod(end.y, gridSize) == @divFloor(gridSize, 2));

    var pathList = std.ArrayList(ScreenPos).empty;
    var current = start;
    var next = start;
    const maxLen = 20; // good idea long term, one method over retiling

    // greedy algorithm, can get stuck in local minima with obstacles
    while (!std.meta.eql(current, end)) {
        if (pathList.items.len > maxLen) {
            break;
        }
        try pathList.append(allocator, current);

        // pick the best movement
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

    // consider creating a struct for this
    var clickedPt: ScreenPos = .{ .x = centerX, .y = centerY };
    var agentPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 50;
    const gridChange: i32 = 10;

    agentPt = getSquareCenter(gridSize, getSquareInGrid(gridSize, clickedPt));

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

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            clickedPt.x = rl.GetMouseX();
            clickedPt.y = rl.GetMouseY();
        }

        // highlight square of grid containing target
        const gridSquare = getSquareInGrid(gridSize, clickedPt);
        const gridSquareCenter = getSquareCenter(gridSize, gridSquare);

        var path = try getPath(agentPt, gridSquareCenter, gridSize, &crossDiagonalMovement, allocator);

        rl.DrawRectangle(gridSquare.x, gridSquare.y, gridSize, gridSize, rl.GREEN);

        printGrid(gridSize, 0, @max(screenWidth, screenHeight));
        drawPathLines(path.items);
        path.deinit(allocator);

        // rl.DrawCircle(clickedPt.x, clickedPt.y, 3, rl.LIGHTGRAY);
        rl.DrawCircle(agentPt.x, agentPt.y, 10, rl.RED);
    }
}
