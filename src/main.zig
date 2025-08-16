//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.

pub const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

const std = @import("std");

pub fn main() !void {
    rl.SetTraceLogLevel(rl.LOG_DEBUG);
    rl.SetConfigFlags(rl.FLAG_WINDOW_HIGHDPI);
    const screenWidth: i32 = 800;
    const screenHeight: i32 = 450;
    rl.InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");
    defer rl.CloseWindow();

    rl.SetTargetFPS(60);

    rl.SetWindowPosition(0, 0);

    const centerX: i32 = screenWidth / 2;
    const centerY: i32 = screenHeight / 2;

    // consider creating a struct for this
    var circleX: i32 = centerX;
    var circleY: i32 = centerY;

    while (!rl.WindowShouldClose()) {
        rl.BeginDrawing();
        defer rl.EndDrawing();
        rl.ShowCursor();

        rl.ClearBackground(rl.RAYWHITE);
        rl.DrawText("Congrats! You created your first window!", 190, 200, 20, rl.LIGHTGRAY);

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            circleX = rl.GetMouseX();
            circleY = rl.GetMouseY();
        }

        rl.DrawCircle(circleX, circleY, 10, rl.DARKBLUE);
    }
}
