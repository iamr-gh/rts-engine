const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
const pathfinding = @import("pathfinding.zig");
const selection = @import("selection.zig");

const rl = grid.rl;
const ScreenPos = types.ScreenPos;
const GridCoord = types.GridCoord;

pub fn main() !void {
    const allocator = std.heap.page_allocator;
    rl.SetTraceLogLevel(rl.LOG_DEBUG);

    rl.SetConfigFlags(rl.FLAG_WINDOW_HIGHDPI);
    rl.SetConfigFlags(rl.FLAG_WINDOW_RESIZABLE);

    const screenWidth: i32 = 1000;
    const screenHeight: i32 = 1000;
    rl.InitWindow(screenWidth, screenHeight, "rts engine testing");
    defer rl.CloseWindow();

    const fps: i32 = 60;
    rl.SetTargetFPS(fps);
    rl.SetWindowPosition(0, 0);

    const centerX: i32 = screenWidth / 2;
    const centerY: i32 = screenHeight / 2;

    // pixels per second
    const agentSpeed: f32 = 150;

    // const NUM_AGENTS: i32 = 2;

    var goalPt: ScreenPos = .{ .x = centerX, .y = centerY };
    var agentPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 6;
    const gridChange: i32 = 10;

    agentPt = grid.getSquareCenter(gridSize, grid.getSquareInGrid(gridSize, goalPt));

    var path: ?std.ArrayList(ScreenPos) = null;
    var previousTarget = agentPt;

    var obstacleGrid = try map.generateTerrainObstacles(allocator, gridSize, screenWidth, screenHeight, agentPt);
    var prevGridSize = gridSize;

    std.debug.assert(@mod(gridSize, 2) == 0);
    std.debug.assert(@mod(gridChange, 2) == 0);

    var end = false;
    while (!end and !rl.WindowShouldClose()) {
        rl.BeginDrawing();
        defer rl.EndDrawing();

        if (rl.IsKeyDown(rl.KEY_W) and rl.IsKeyDown(rl.KEY_LEFT_SUPER)) {
            end = true;
        }

        rl.ClearBackground(rl.RAYWHITE);

        if (rl.IsKeyPressed(rl.KEY_LEFT)) {
            gridSize -= gridChange;
        } else if (rl.IsKeyPressed(rl.KEY_RIGHT))
            gridSize += gridChange;

        gridSize = @max(gridSize, 5);
        gridSize = @min(gridSize, 100);

        if (gridSize != prevGridSize) {
            obstacleGrid.deinit();
            obstacleGrid = try map.generateTerrainObstacles(allocator, gridSize, screenWidth, screenHeight, agentPt);
            if (path) |*p| p.deinit(allocator);
            path = null;
            prevGridSize = gridSize;
        }

        // pathing to goal
        if (rl.IsMouseButtonPressed(rl.MOUSE_RIGHT_BUTTON)) {
            goalPt.x = rl.GetMouseX();
            goalPt.y = rl.GetMouseY();
        }

        const goalSquare = grid.getSquareInGrid(gridSize, goalPt);
        const goalSquareCenter = grid.getSquareCenter(gridSize, goalSquare);

        if (map.isObstacle(&obstacleGrid, goalSquareCenter, gridSize)) {
            rl.DrawRectangle(goalSquare.x, goalSquare.y, gridSize, gridSize, rl.RED);
        } else {
            rl.DrawRectangle(goalSquare.x, goalSquare.y, gridSize, gridSize, rl.GREEN);
        }

        if (!std.meta.eql(goalSquareCenter, previousTarget)) {
            if (path) |*p| p.deinit(allocator);
            const agentSquare = grid.getSquareInGrid(gridSize, agentPt);
            agentPt = grid.getSquareCenter(gridSize, agentSquare);
            path = pathfinding.getPathAstar(agentPt, goalSquareCenter, gridSize, &pathfinding.crossDiagonalMovement, &obstacleGrid, allocator) catch null;
            if (path) |*p| {
                if (p.items.len > 0) _ = p.orderedRemove(0);
            }
            previousTarget = goalSquareCenter;
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

        grid.printGrid(gridSize, 0, @max(screenWidth, screenHeight));

        map.drawObstacles(&obstacleGrid, gridSize);

        if (path) |*p| {
            pathfinding.drawPathLines(p.items);
        }

        rl.DrawCircle(agentPt.x, agentPt.y, 5, rl.RED);
    }

    obstacleGrid.deinit();
    if (path) |*p| p.deinit(allocator);
}
