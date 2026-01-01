pub const ScreenPos = struct {
    x: c_int,
    y: c_int,
};

pub const GridCoord = struct {
    x: i32,
    y: i32,
};

pub const PathNode = struct {
    pos: ScreenPos,
    gridCoord: GridCoord,
    timeStep: u32,
};

pub const TimedPath = struct {
    nodes: std.ArrayList(PathNode),
    currentNodeIndex: u32,
    currentWaitRemaining: f32,
    completed: bool,
};

pub const Reservation = struct {
    agentId: usize,
    startTime: u32,
    endTime: u32,
};

pub const Agent = struct {
    id: usize,
    pos: ScreenPos,
    selected: bool,
    path: ?TimedPath,
    destination: ?ScreenPos,
};

pub const AgentList = struct {
    agents: std.ArrayList(Agent),
    nextId: usize,
};

const std = @import("std");
