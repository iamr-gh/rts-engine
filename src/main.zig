const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
const pathfinding = @import("pathfinding.zig");
const selection = @import("selection.zig");

const rl = pathfinding.rl;
const ScreenPos = types.ScreenPos;
const GridCoord = types.GridCoord;
const Agent = types.Agent;

const AGENT_RADIUS: f32 = 8.0;
const CLICK_THRESHOLD: f32 = 10.0;
const MAX_WAIT_TIME: i64 = 2; // seconds before agent replans path (configurable)

fn isAgentClicked(agentPos: ScreenPos, mousePos: ScreenPos) bool {
    const dx: f32 = @floatFromInt(agentPos.x - mousePos.x);
    const dy: f32 = @floatFromInt(agentPos.y - mousePos.y);
    return std.math.sqrt(dx * dx + dy * dy) <= AGENT_RADIUS + CLICK_THRESHOLD;
}

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

    const agentSpeed: f32 = 150;

    const NUM_AGENTS: i32 = 5;

    var goalPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 6;
    const gridChange: i32 = 10;

    var obstacleGrid = try map.generateTerrainObstaclesWithConfig(allocator, gridSize, screenWidth, screenHeight, goalPt, map.terrain.PLAINS);
    var prevGridSize = gridSize;

    const colors = [_]rl.Color{ rl.RED, rl.GREEN, rl.BLUE, rl.ORANGE, rl.PURPLE, rl.GOLD, rl.VIOLET, rl.MAROON, rl.SKYBLUE, rl.DARKGRAY };

    var agents = try std.ArrayList(Agent).initCapacity(allocator, NUM_AGENTS);
    errdefer {
        for (agents.items) |*agent| {
            if (agent.path) |*p| p.deinit(allocator);
        }
        agents.deinit(allocator);
    }

    var rng = std.Random.DefaultPrng.init(@intCast(std.time.timestamp()));
    const random = rng.random();

    var agentIdx: i32 = 0;
    while (agentIdx < NUM_AGENTS) : (agentIdx += 1) {
        var agentPos: ScreenPos = undefined;
        var valid = false;
        var attempts: i32 = 0;

        while (!valid and attempts < 100) : (attempts += 1) {
            agentPos.x = random.intRangeAtMost(i32, 0, screenWidth - gridSize);
            agentPos.y = random.intRangeAtMost(i32, 0, screenHeight - gridSize);
            const square = grid.getSquareInGrid(gridSize, agentPos);
            agentPos = grid.getSquareCenter(gridSize, square);

            if (map.isObstacle(&obstacleGrid, agentPos, gridSize)) {
                continue;
            }

            valid = true;
            for (agents.items) |other| {
                const dx: f32 = @floatFromInt(agentPos.x - other.pos.x);
                const dy: f32 = @floatFromInt(agentPos.y - other.pos.y);
                if (std.math.sqrt(dx * dx + dy * dy) < @as(f32, @floatFromInt(gridSize)) * 2) {
                    valid = false;
                    break;
                }
            }
        }

        const colorIndex = @mod(@as(usize, @intCast(agentIdx)), colors.len);
        try agents.append(allocator, .{
            .pos = agentPos,
            .path = null,
            .selected = false,
            .color_index = colorIndex,
        });
    }

    var previousTarget: ?ScreenPos = null;

    std.debug.assert(@mod(gridSize, 2) == 0);
    std.debug.assert(@mod(gridChange, 2) == 0);

    var box = selection.Box{};
    var prevBox = selection.Box{};
    var boxCompleted = false;

    var end = false;
    while (!end and !rl.WindowShouldClose()) {
        rl.BeginDrawing();
        defer rl.EndDrawing();

        if (rl.IsKeyDown(rl.KEY_W) and rl.IsKeyDown(rl.KEY_LEFT_SUPER)) {
            end = true;
        }

        rl.ClearBackground(rl.RAYWHITE);

        prevBox = box;
        selection.updateBox(&box);

        boxCompleted = false;
        if (prevBox.active and !box.active) {
            boxCompleted = true;
        }

        if (rl.IsKeyPressed(rl.KEY_LEFT)) {
            gridSize -= gridChange;
        } else if (rl.IsKeyPressed(rl.KEY_RIGHT))
            gridSize += gridChange;

        gridSize = @max(gridSize, 5);
        gridSize = @min(gridSize, 100);

        if (gridSize != prevGridSize) {
            obstacleGrid.deinit();
            obstacleGrid = try map.generateTerrainObstaclesWithConfig(allocator, gridSize, screenWidth, screenHeight, goalPt, map.terrain.PLAINS);
            for (agents.items) |*agent| {
                const square = grid.getSquareInGrid(gridSize, agent.pos);
                agent.pos = grid.getSquareCenter(gridSize, square);
                if (agent.path) |*p| {
                    p.deinit(allocator);
                    agent.path = null;
                }
            }
            prevGridSize = gridSize;
        }

        const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };

        if (boxCompleted) {
            const boxWidth = @abs(box.dst.x - box.src.x);
            const boxHeight = @abs(box.dst.y - box.src.y);
            if (boxWidth > 2 and boxHeight > 2) {
                for (agents.items) |*agent| {
                    agent.selected = selection.insideBox(agent.pos, box);
                }
            } else {
                var clickedAgent: bool = false;
                for (agents.items) |*agent| {
                    if (isAgentClicked(agent.pos, mousePos)) {
                        for (agents.items) |*a| a.selected = false;
                        agent.selected = true;
                        clickedAgent = true;
                        break;
                    }
                }
                if (!clickedAgent) {
                    for (agents.items) |*a| a.selected = false;
                }
            }
        }

        if (rl.IsMouseButtonPressed(rl.MOUSE_RIGHT_BUTTON)) {
            goalPt.x = rl.GetMouseX();
            goalPt.y = rl.GetMouseY();

            const goalSquare = grid.getSquareInGrid(gridSize, goalPt);
            const goalSquareCenter = grid.getSquareCenter(gridSize, goalSquare);

            // Get group goals preserving formation
            const maxSearchRadius: i32 = 20;
            var groupGoals = pathfinding.findGroupGoals(
                goalSquareCenter,
                agents.items,
                gridSize,
                &obstacleGrid,
                allocator,
                maxSearchRadius,
            ) catch null;

            if (groupGoals) |_| {
                defer groupGoals.?.deinit(allocator);

                // Assign paths to each selected agent
                var goalIdx: usize = 0;
                const currentTime = std.time.timestamp();
                for (agents.items) |*agent| {
                    if (agent.selected) {
                        if (agent.path) |*p| p.deinit(allocator);
                        const agentSquare = grid.getSquareInGrid(gridSize, agent.pos);
                        const agentPosCenter = grid.getSquareCenter(gridSize, agentSquare);
                        const maxPathLen: usize = @intCast(@divTrunc(screenHeight, gridSize) + @divTrunc(screenWidth, gridSize));

                        const agentGoal = groupGoals.?.items[goalIdx];
                        agent.path = pathfinding.getPathAstar(
                            agentPosCenter,
                            agentGoal,
                            gridSize,
                            &pathfinding.crossDiagonalMovement,
                            &obstacleGrid,
                            allocator,
                            maxPathLen,
                        ) catch null;

                        if (agent.path) |*p| {
                            if (p.items.len > 0) _ = p.orderedRemove(0);
                        }

                        // Set movement priority timestamp and reset wait timer
                        agent.moveStartTime = currentTime;
                        agent.waitStartTime = null;

                        goalIdx += 1;
                    }
                }
            }
            previousTarget = goalSquareCenter;
        }

        const goalSquare = grid.getSquareInGrid(gridSize, goalPt);
        const goalSquareCenter = grid.getSquareCenter(gridSize, goalSquare);

        if (map.isObstacle(&obstacleGrid, goalSquareCenter, gridSize)) {
            rl.DrawRectangle(goalSquare.x, goalSquare.y, gridSize, gridSize, rl.RED);
        } else {
            rl.DrawRectangle(goalSquare.x, goalSquare.y, gridSize, gridSize, rl.GREEN);
        }

        const deltaTime = rl.GetFrameTime();
        const currentTime = std.time.timestamp();

        // Build array of agent indices sorted by moveStartTime (priority order)
        var agentOrder: [100]usize = undefined; // Support up to 100 agents
        const numAgents = agents.items.len;
        for (0..numAgents) |i| {
            agentOrder[i] = i;
        }

        // Simple insertion sort by moveStartTime (earlier = higher priority)
        for (1..numAgents) |i| {
            const key = agentOrder[i];
            var j: usize = i;
            while (j > 0 and agents.items[agentOrder[j - 1]].moveStartTime > agents.items[key].moveStartTime) {
                agentOrder[j] = agentOrder[j - 1];
                j -= 1;
            }
            agentOrder[j] = key;
        }

        // Process agents in priority order
        for (0..numAgents) |orderPos| {
            const currentAgentIdx = agentOrder[orderPos];
            var agent = &agents.items[currentAgentIdx];

            if (agent.path == null) continue;
            var p = &agent.path.?;
            if (p.items.len == 0) continue;

            const nextPoint = p.items[0];
            const currentPos = rl.Vector2{
                .x = @floatFromInt(agent.pos.x),
                .y = @floatFromInt(agent.pos.y),
            };
            const targetPos = rl.Vector2{
                .x = @floatFromInt(nextPoint.x),
                .y = @floatFromInt(nextPoint.y),
            };

            const dx = targetPos.x - currentPos.x;
            const dy = targetPos.y - currentPos.y;
            const distance = @sqrt(dx * dx + dy * dy);

            // Check if reached waypoint
            if (distance < 1.0) {
                _ = p.orderedRemove(0);
                agent.waitStartTime = null;
                if (p.items.len == 0) {
                    p.deinit(allocator);
                    agent.path = null;
                }
                continue;
            }

            // Calculate desired direction
            const desiredDir = rl.Vector2{
                .x = dx / distance,
                .y = dy / distance,
            };

            // Try to find safe movement
            const safeMove = pathfinding.findSafeMovement(
                currentAgentIdx,
                agents.items,
                agentOrder[0..numAgents],
                orderPos,
                desiredDir,
                agentSpeed,
                deltaTime,
                AGENT_RADIUS,
                &obstacleGrid,
                gridSize,
            );

            if (safeMove) |move| {
                // Apply movement
                const newX = currentPos.x + move.x;
                const newY = currentPos.y + move.y;
                agent.pos.x = @intFromFloat(newX);
                agent.pos.y = @intFromFloat(newY);
                agent.waitStartTime = null;
            } else {
                // Can't move - start or continue waiting
                if (agent.waitStartTime == null) {
                    agent.waitStartTime = currentTime;
                } else if (currentTime - agent.waitStartTime.? >= MAX_WAIT_TIME) {
                    // Waited too long - recompute path treating other agents as obstacles
                    const finalGoal = p.items[p.items.len - 1];
                    p.deinit(allocator);
                    agent.path = null;
                    agent.waitStartTime = null;

                    // Collect positions of other agents as blocked cells
                    var blockedCells: [100]ScreenPos = undefined;
                    var blockedCount: usize = 0;
                    for (agents.items, 0..) |other, otherIdx| {
                        if (otherIdx == currentAgentIdx) continue;
                        const otherSquare = grid.getSquareInGrid(gridSize, other.pos);
                        const otherCenter = grid.getSquareCenter(gridSize, otherSquare);
                        if (blockedCount < 100) {
                            blockedCells[blockedCount] = otherCenter;
                            blockedCount += 1;
                        }
                    }

                    // Recompute path avoiding other agents
                    const agentSquare = grid.getSquareInGrid(gridSize, agent.pos);
                    const agentPosCenter = grid.getSquareCenter(gridSize, agentSquare);
                    const maxPathLen: usize = @intCast(@divTrunc(screenHeight, gridSize) + @divTrunc(screenWidth, gridSize));

                    agent.path = pathfinding.getPathAstarWithBlockedCells(
                        agentPosCenter,
                        finalGoal,
                        gridSize,
                        &pathfinding.crossDiagonalMovement,
                        &obstacleGrid,
                        blockedCells[0..blockedCount],
                        allocator,
                        maxPathLen,
                    ) catch null;

                    if (agent.path) |*newPath| {
                        if (newPath.items.len > 0) _ = newPath.orderedRemove(0);
                    }

                    // Reset move start time for new path
                    agent.moveStartTime = currentTime;
                }
                // else: keep waiting
            }
        }

        grid.printGrid(gridSize, 0, @max(screenWidth, screenHeight));

        map.drawObstacles(&obstacleGrid, gridSize);
        selection.drawBox(box);

        for (agents.items) |agent| {
            if (agent.path) |*p| {
                pathfinding.drawPathLines(p.items);
            }
        }

        for (agents.items) |agent| {
            if (agent.selected) {
                rl.DrawCircleLines(agent.pos.x, agent.pos.y, AGENT_RADIUS + 4, rl.WHITE);
            }
            rl.DrawCircle(agent.pos.x, agent.pos.y, AGENT_RADIUS, colors[agent.color_index]);
        }
    }

    obstacleGrid.deinit();
    for (agents.items) |*agent| {
        if (agent.path) |*p| p.deinit(allocator);
    }
    agents.deinit(allocator);
}
