const types = @import("types.zig");
const std = @import("std");

pub const local_info = struct {
    agents: *std.ArrayList(types.Agent),
    agent_idx: usize,
    agentSpeed: f32,
    deltaTime: f32,
    allocator: std.mem.Allocator,
};

// might make generic eventually
pub fn plan_agents(
    agents: *std.ArrayList(types.Agent),
    agentSpeed: f32,
    deltaTime: f32,
    planner: fn (info: local_info) void,
    allocator: std.mem.Allocator,
) void {
    var agentIdx: usize = 0;
    while (agentIdx < agents.items.len) : (agentIdx += 1) {
        const info: local_info = .{
            .agents = agents,
            .agent_idx = agentIdx,
            .agentSpeed = agentSpeed,
            .deltaTime = deltaTime,
            .allocator = allocator,
        };
        planner(info);
    }
}

pub fn basic_planner(
    info: local_info,
) void {
    const agent = &info.agents.items[info.agent_idx];
    if (agent.path) |*p| {
        if (p.items.len > 0) {
            const nextPoint = p.items[0];
            const dx: f32 = @floatFromInt(nextPoint.x - agent.pos.x);
            const dy: f32 = @floatFromInt(nextPoint.y - agent.pos.y);
            const distance = std.math.sqrt(dx * dx + dy * dy);

            if (distance > 0) {
                const moveAmount = info.agentSpeed * info.deltaTime;
                if (moveAmount >= distance) {
                    agent.pos = nextPoint;
                    _ = p.orderedRemove(0);
                    if (p.items.len == 0) {
                        p.deinit(info.allocator);
                        agent.path = null;
                    }
                } else {
                    const ratio = moveAmount / distance;
                    const next_pos_x = agent.pos.x + @as(i32, @intFromFloat(dx * ratio));
                    const next_pos_y = agent.pos.y + @as(i32, @intFromFloat(dy * ratio));

                    const safeDistance = 2 * moveAmount;

                    // check if other agent is there
                    var tooClose = false;
                    for (info.agents.items) |*other_agent| {
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

                    agent.pos.x = next_pos_x;
                    agent.pos.y = next_pos_y;
                }
            }
        }
    }
}

// oh these things are going to need memory...
pub fn momentum_planner(
    momentum: f32,
    info: local_info,
) void {
    _ = momentum;
    const agent = &info.agents.items[info.agent_idx];
    if (agent.path) |*p| {
        if (p.items.len > 0) {
            const nextPoint = p.items[0];
            const dx: f32 = @floatFromInt(nextPoint.x - agent.pos.x);
            const dy: f32 = @floatFromInt(nextPoint.y - agent.pos.y);
            const distance = std.math.sqrt(dx * dx + dy * dy);

            if (distance > 0) {
                const moveAmount = info.agentSpeed * info.deltaTime;
                if (moveAmount >= distance) {
                    agent.pos = nextPoint;
                    _ = p.orderedRemove(0);
                    if (p.items.len == 0) {
                        p.deinit(info.allocator);
                        agent.path = null;
                    }
                } else {
                    const ratio = moveAmount / distance;
                    const next_pos_x = agent.pos.x + @as(i32, @intFromFloat(dx * ratio));
                    const next_pos_y = agent.pos.y + @as(i32, @intFromFloat(dy * ratio));

                    const safeDistance = 2 * moveAmount;

                    // check if other agent is there
                    var tooClose = false;
                    for (info.agents.items) |*other_agent| {
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

                    agent.pos.x = next_pos_x;
                    agent.pos.y = next_pos_y;
                }
            }
        }
    }
}
