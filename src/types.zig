const std = @import("std");

pub const ScreenPos = struct {
    x: c_int = 0,
    y: c_int = 0,
};

pub const GridCoord = struct {
    x: i32 = 0,
    y: i32 = 0,
};

pub const Agent = struct {
    pos: ScreenPos,
    path: ?std.ArrayList(ScreenPos),
    selected: bool,
    color_index: usize,
    moveStartTime: i64 = 0, // timestamp when agent started current path (for priority)
    waitStartTime: ?i64 = null, // timestamp when agent started waiting (null if not waiting)
};
