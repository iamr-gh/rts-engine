const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
const pathfinding = @import("pathfinding.zig");
const selection = @import("selection.zig");
const utils = @import("utils.zig");

const rl = grid.rl;
const ScreenPos = types.ScreenPos;
const GridCoord = types.GridCoord;
const Agent = types.Agent;

const AGENT_RADIUS: f32 = 8.0;
const CLICK_THRESHOLD: f32 = 10.0;

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

    const NUM_AGENTS: i32 = 100;

    var goalPt: ScreenPos = .{ .x = centerX, .y = centerY };

    var gridSize: i32 = 20;
    const gridChange: i32 = 10;

    var obstacleGrid = try map.generateTerrainObstaclesWithConfig(allocator, gridSize, screenWidth, screenHeight, goalPt, map.terrain.MOUNTAINOUS);
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

        const hexWidth: f32 = @as(f32, @floatFromInt(gridSize)) * 0.75;
        const hexHeight: f32 = @as(f32, @floatFromInt(gridSize)) * std.math.sqrt(3.0) / 2.0;
        const numCols: i32 = @intFromFloat(@as(f32, @floatFromInt(screenWidth)) / hexWidth);
        const numRows: i32 = @intFromFloat(@as(f32, @floatFromInt(screenHeight)) / hexHeight);

        while (!valid and attempts < 100) : (attempts += 1) {
            const col = random.intRangeAtMost(i32, 0, numCols - 1);
            const row = random.intRangeAtMost(i32, 0, numRows - 1);
            const axial = grid.offsetToAxial(col, row);
            agentPos = grid.axialToScreen(axial[0], axial[1], gridSize);

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
                const axial = grid.getHexContainingPos(agent.pos, gridSize);
                agent.pos = grid.axialToScreen(axial[0], axial[1], gridSize);
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
        }

        const axial = grid.getHexContainingPos(goalPt, gridSize);
        const goalHexCenter = grid.axialToScreen(axial[0], axial[1], gridSize);

        if (rl.IsMouseButtonPressed(rl.MOUSE_RIGHT_BUTTON)) {
            var num_selected: i32 = 0;

            for (agents.items) |*agent| {
                if (agent.selected) {
                    num_selected += 1;
                }
            }

            // goal allocation
            const goals = try pathfinding.getGroupGoals(&obstacleGrid, goalHexCenter, num_selected, gridSize, allocator);

            // path assignment
            var idx: usize = 0;

            // could switch everything to arena allocators eventually
            var arena = std.heap.ArenaAllocator.init(allocator);
            defer arena.deinit();
            const map_alloc = arena.allocator();

            var occupiedMap = std.AutoHashMap(ScreenPos, std.AutoHashMap(i32, void)).init(map_alloc);
            var finalPositions = std.AutoHashMap(ScreenPos, pathfinding.FinalPosition).init(map_alloc);

            for (agents.items) |*agent| {
                if (agent.selected) {
                    if (agent.path) |*p| p.deinit(allocator);
                    const goal = goals.items[idx];
                    _ = goal;

                    const agentAxial = grid.getHexContainingPos(agent.pos, gridSize);
                    var maxPathLen: usize = @intCast(@divTrunc(screenHeight, gridSize) + @divTrunc(screenWidth, gridSize));
                    maxPathLen *= 2; // hotfix, I thought previous was a good enough bound

                    agent.path = pathfinding.getPathAstar(grid.axialToScreen(agentAxial[0], agentAxial[1], gridSize), goalHexCenter, gridSize, &pathfinding.hexMovement, &obstacleGrid, allocator, maxPathLen, occupiedMap, finalPositions) catch null;
                    if (agent.path) |*p| {
                        var node_idx: i32 = 0;
                        // bug with end of paths
                        while (node_idx < p.items.len) : (node_idx += 1) {
                            const node = p.items[@intCast(node_idx)];
                            if (occupiedMap.getPtr(node)) |times| {
                                try times.put(node_idx, {});
                            } else {
                                var newMap = std.AutoHashMap(i32, void).init(map_alloc);
                                try newMap.put(node_idx, {});
                                try occupiedMap.put(node, newMap);
                            }
                        }

                        // mark final position as permanently occupied from arrival time onwards
                        if (p.items.len > 0) {
                            const final_pos = p.items[p.items.len - 1];
                            const arrival_time: i32 = @intCast(p.items.len - 1);
                            try finalPositions.put(final_pos, pathfinding.FinalPosition{ .arrival_time = arrival_time });
                        }

                        if (p.items.len > 0) _ = p.orderedRemove(0);
                    }
                    idx += 1;
                }
            }
            previousTarget = goalHexCenter;
        }

        if (map.isObstacle(&obstacleGrid, goalHexCenter, gridSize)) {
            grid.drawHex(goalHexCenter, gridSize, .{ .r = 255, .g = 0, .b = 0, .a = 255 });
        } else {
            grid.drawHex(goalHexCenter, gridSize, .{ .r = 0, .g = 255, .b = 0, .a = 255 });
        }

        const deltaTime = rl.GetFrameTime();

        // path following code
        for (agents.items) |*agent| {
            if (agent.path) |*p| {
                if (p.items.len > 0) {
                    const nextPoint = p.items[0];
                    const dx: f32 = @floatFromInt(nextPoint.x - agent.pos.x);
                    const dy: f32 = @floatFromInt(nextPoint.y - agent.pos.y);
                    const distance = std.math.sqrt(dx * dx + dy * dy);

                    if (distance > 0) {
                        const moveAmount = agentSpeed * deltaTime;
                        if (moveAmount >= distance) {
                            agent.pos = nextPoint;
                            _ = p.orderedRemove(0);
                            if (p.items.len == 0) {
                                p.deinit(allocator);
                                agent.path = null;
                            }
                        } else {
                            const ratio = moveAmount / distance;
                            const next_pos_x = agent.pos.x + @as(i32, @intFromFloat(dx * ratio));
                            const next_pos_y = agent.pos.y + @as(i32, @intFromFloat(dy * ratio));

                            const safeDistance = 2 * moveAmount;

                            // check if other agent is there
                            var tooClose = false;
                            for (agents.items) |*other_agent| {
                                if (other_agent != agent) {
                                    const agent_dx: f32 = @floatFromInt(other_agent.pos.x - agent.pos.x);
                                    const agent_dy: f32 = @floatFromInt(other_agent.pos.y - agent.pos.y);
                                    const agent_distance = std.math.sqrt(agent_dx * agent_dx + agent_dy * agent_dy);

                                    // deadlock
                                    if (agent_distance <= safeDistance) {
                                        tooClose = true;
                                    }
                                }
                            }

                            // if so, wait
                            // need to refine this local planner
                            // if (tooClose) {
                            //     continue;
                            // }

                            agent.pos.x = next_pos_x;
                            agent.pos.y = next_pos_y;
                        }
                    }
                }
            }
        }

        grid.printHexGrid(gridSize, screenWidth, screenHeight);

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
