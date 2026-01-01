const std = @import("std");
const types = @import("types.zig");
const grid = @import("grid.zig");
const map = @import("map.zig");
const pathfinding = @import("pathfinding.zig");
const agents = @import("agents.zig");

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

    const agentSpeed: f32 = 200;
    const INITIAL_AGENT_COUNT: usize = 2;

    var clickedPt: ScreenPos = .{ .x = centerX, .y = centerY };
    var agentPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 30;
    const gridChange: i32 = 10;

    agentPt = grid.getSquareCenter(gridSize, grid.getSquareInGrid(gridSize, clickedPt));

    const obstacleChance: [2]f32 = .{ 0.2, 0.2 };

    var obstacleGrid = try map.generateRandomObstacles(allocator, gridSize, screenWidth, screenHeight, agentPt, obstacleChance);
    var prevGridSize = gridSize;

    std.debug.assert(@mod(gridSize, 2) == 0);
    std.debug.assert(@mod(gridChange, 2) == 0);

    var agentList = try agents.initAgents(allocator, INITIAL_AGENT_COUNT, agentPt, gridSize);
    defer agents.cleanup(&agentList, allocator);

    var selectionDragStart: ?ScreenPos = null;
    var isDraggingSelection = false;
    var leftButtonPressedThisFrame = false;
    var leftButtonPressHandled = false;

    var end = false;
    while (!end and !rl.WindowShouldClose()) {
        if (rl.IsKeyDown(rl.KEY_W) and rl.IsKeyDown(rl.KEY_LEFT_SUPER)) {
            end = true;
        }

        rl.BeginDrawing();
        defer rl.EndDrawing();

        rl.ClearBackground(rl.RAYWHITE);

        if (rl.IsKeyPressed(rl.KEY_LEFT)) {
            gridSize -= gridChange;
        } else if (rl.IsKeyPressed(rl.KEY_RIGHT))
            gridSize += gridChange;

        gridSize = @max(gridSize, 5);
        gridSize = @min(gridSize, 100);

        if (gridSize != prevGridSize) {
            obstacleGrid.deinit();
            obstacleGrid = try map.generateRandomObstacles(allocator, gridSize, screenWidth, screenHeight, agentPt, obstacleChance);
            agents.clearAllPaths(&agentList, allocator);
            prevGridSize = gridSize;
        }

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            leftButtonPressedThisFrame = true;
            leftButtonPressHandled = false;
        } else {
            leftButtonPressedThisFrame = false;
        }

        if (rl.IsMouseButtonDown(rl.MOUSE_LEFT_BUTTON)) {
            if (leftButtonPressedThisFrame and !leftButtonPressHandled) {
                const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };
                if (agents.getAgentAtPosition(&agentList, mousePos, agents.AGENT_SELECT_RADIUS)) |agentIdx| {
                    leftButtonPressHandled = true;
                    const additive = rl.IsKeyDown(rl.KEY_LEFT_CONTROL) or rl.IsKeyDown(rl.KEY_RIGHT_CONTROL);
                    if (!additive) {
                        agents.deselectAll(&agentList);
                    }
                    agents.toggleSelection(&agentList.agents.items[agentIdx]);
                } else {
                    isDraggingSelection = true;
                    selectionDragStart = mousePos;
                    leftButtonPressHandled = true;
                }
            }
        } else {
            if (isDraggingSelection) {
                const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };
                if (selectionDragStart) |start| {
                    const x = @min(start.x, mousePos.x);
                    const y = @min(start.y, mousePos.y);
                    const width = @abs(start.x - mousePos.x);
                    const height = @abs(start.y - mousePos.y);

                    if (width > 2 or height > 2) {
                        agents.selectAgentsInRect(&agentList, @as(i32, @intCast(x)), @as(i32, @intCast(y)), @as(i32, @intCast(width)), @as(i32, @intCast(height)), agents.AGENT_SELECT_RADIUS);
                    }
                }
                isDraggingSelection = false;
                selectionDragStart = null;
            }
        }

        const gridSquare = grid.getSquareInGrid(gridSize, clickedPt);
        const gridSquareCenter = grid.getSquareCenter(gridSize, gridSquare);

        if (rl.IsMouseButtonPressed(rl.MOUSE_RIGHT_BUTTON)) {
            const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };

            if (agents.getSelectedCount(&agentList) > 0) {
                const targetPos = grid.getSquareCenter(gridSize, grid.getSquareInGrid(gridSize, mousePos));
                agents.assignDestinationsToSelected(&agentList, targetPos, gridSize, &obstacleGrid, allocator) catch {};
            } else {
                const rightGridSquare = grid.getSquareInGrid(gridSize, mousePos);
                const coord = GridCoord{
                    .x = @divFloor(rightGridSquare.x, gridSize),
                    .y = @divFloor(rightGridSquare.y, gridSize),
                };
                try obstacleGrid.toggle(coord);
            }
        }

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            clickedPt.x = rl.GetMouseX();
            clickedPt.y = rl.GetMouseY();
        }

        if (map.isObstacle(&obstacleGrid, gridSquareCenter, gridSize)) {
            rl.DrawRectangle(gridSquare.x, gridSquare.y, gridSize, gridSize, rl.RED);
        } else {
            rl.DrawRectangle(gridSquare.x, gridSquare.y, gridSize, gridSize, rl.GREEN);
        }

        grid.printGrid(gridSize, 0, @max(screenWidth, screenHeight));

        map.drawObstacles(&obstacleGrid, gridSize);

        const deltaTime = rl.GetFrameTime();
        agents.updateAgentPositions(&agentList, deltaTime, agentSpeed, gridSize);

        agents.drawAgents(&agentList);

        for (agentList.agents.items) |*agent| {
            if (agent.selected and agent.path != null) {
                const path = agent.path.?;
                var pathPositions = std.ArrayList(ScreenPos).empty;
                defer pathPositions.deinit(allocator);
                for (path.nodes.items) |node| {
                    pathPositions.append(allocator, node.pos) catch {};
                }
                pathfinding.drawPathLines(pathPositions.items);
            }
        }

        if (isDraggingSelection) {
            if (selectionDragStart) |start| {
                const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };
                agents.drawSelectionBox(start, mousePos);
            }
        }
    }
}
