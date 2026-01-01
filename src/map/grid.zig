pub const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

const types = @import("../types.zig");
const ScreenPos = types.ScreenPos;

pub fn printGrid(inc: i32, start: i32, end: i32) void {
    var i: i32 = start;

    while (i < end) : (i += inc) {
        rl.DrawLine(i, start, i, end, rl.GRAY);
        rl.DrawLine(start, i, end, i, rl.GRAY);
    }
}

pub fn getSquareInGrid(gridSize: i32, pos: ScreenPos) ScreenPos {
    const leftX = @divFloor(pos.x, gridSize) * gridSize;
    const topY = @divFloor(pos.y, gridSize) * gridSize;
    return .{ .x = leftX, .y = topY };
}

pub fn getSquareCenter(gridSize: i32, pos: ScreenPos) ScreenPos {
    return .{ .x = pos.x + @divFloor(gridSize, 2), .y = pos.y + @divFloor(gridSize, 2) };
}
