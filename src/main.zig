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

    while (i < end) : (i += inc) {
        // vertical line
        rl.DrawLine(i, start, i, end, rl.GRAY);
        // horizontal line
        rl.DrawLine(start, i, end, i, rl.GRAY);
    }
}

pub fn main() !void {
    rl.SetTraceLogLevel(rl.LOG_DEBUG);

    // important on macOS
    rl.SetConfigFlags(rl.FLAG_WINDOW_HIGHDPI);

    const screenWidth: i32 = 800;
    const screenHeight: i32 = 450;
    rl.InitWindow(screenWidth, screenHeight, "rts engine testing");
    defer rl.CloseWindow();

    const fps: i32 = 60;
    rl.SetTargetFPS(fps);
    rl.SetWindowPosition(0, 0);

    const vel: i32 = 2; // pixels per frame
    // integer pixel issues
    // need to be careful with the diagonal speed issue

    const centerX: i32 = screenWidth / 2;
    const centerY: i32 = screenHeight / 2;

    // consider creating a struct for this
    var targetPt: ScreenPos = .{ .x = centerX, .y = centerY };
    var agentPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 10;
    const gridChange: i32 = 10;

    while (!rl.WindowShouldClose()) {
        rl.BeginDrawing();
        defer rl.EndDrawing();
        rl.ClearBackground(rl.RAYWHITE);

        // scroll grid scaling is not great
        // if (rl.GetMouseWheelMove() > 0) {
        //     if (rl.GetMouseWheelMoveV().y > 0) {
        //         gridSize += @intFromFloat(rl.GetMouseWheelMoveV().y);
        //     }
        //     else {
        //         gridSize -= @intFromFloat(rl.GetMouseWheelMoveV().y);
        //     }
        // }
        if (rl.IsKeyPressed(rl.KEY_LEFT)) {
            gridSize -= gridChange;
        } else if (rl.IsKeyPressed(rl.KEY_RIGHT))
            gridSize += gridChange;

        gridSize = @max(gridSize, 5);
        gridSize = @min(gridSize, 100);

        printGrid(gridSize, gridSize, screenWidth);

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            targetPt.x = rl.GetMouseX();
            targetPt.y = rl.GetMouseY();
        }

        if (agentPt.x < targetPt.x) {
            agentPt.x += vel;
        } else if (agentPt.x > targetPt.x) {
            agentPt.x -= vel;
        }

        if (agentPt.y < targetPt.y) {
            agentPt.y += vel;
        } else if (agentPt.y > targetPt.y) {
            agentPt.y -= vel;
        }

        rl.DrawCircle(targetPt.x, targetPt.y, 5, rl.LIGHTGRAY);
        rl.DrawCircle(agentPt.x, agentPt.y, 10, rl.RED);
    }
}
