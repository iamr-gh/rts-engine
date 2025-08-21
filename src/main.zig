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

fn getSquareInGrid(gridSize: i32, x: i32, y: i32) ScreenPos {
    const leftX = @divFloor(x, gridSize) * gridSize;
    const topY = @divFloor(y, gridSize) * gridSize;
    return .{ .x = leftX, .y = topY };
}

pub fn main() !void {
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

    const vel: i32 = 2; // pixels per frame

    const centerX: i32 = screenWidth / 2;
    const centerY: i32 = screenHeight / 2;

    // consider creating a struct for this
    var clickedPt: ScreenPos = .{ .x = centerX, .y = centerY };
    var agentPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 50;
    const gridChange: i32 = 10;

    std.debug.assert(@mod(gridSize, 2) == 0);
    std.debug.assert(@mod(gridChange, 2) == 0);

    while (!rl.WindowShouldClose()) {
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
        const gridSquare = getSquareInGrid(gridSize, clickedPt.x, clickedPt.y);
        const gridSquareCenter = .{ .x = gridSquare.x + @divFloor(gridSize, 2), .y = gridSquare.y + @divFloor(gridSize, 2) };

        const freezeThreshold: i32 = 2;

        // bad naive pathing
        if (@abs(agentPt.x - gridSquareCenter.x) > freezeThreshold) {
            if (agentPt.x < gridSquareCenter.x) {
                agentPt.x += vel;
            } else if (agentPt.x > gridSquareCenter.x) {
                agentPt.x -= vel;
            }
        }

        if (@abs(agentPt.y - gridSquareCenter.y) > freezeThreshold) {
            if (agentPt.y < gridSquareCenter.y) {
                agentPt.y += vel;
            } else if (agentPt.y > gridSquareCenter.y) {
                agentPt.y -= vel;
            }
        }

        rl.DrawRectangle(gridSquare.x, gridSquare.y, gridSize, gridSize, rl.GREEN);

        printGrid(gridSize, 0, @max(screenWidth, screenHeight));

        // rl.DrawCircle(clickedPt.x, clickedPt.y, 3, rl.LIGHTGRAY);
        rl.DrawCircle(agentPt.x, agentPt.y, 10, rl.RED);
    }
}
