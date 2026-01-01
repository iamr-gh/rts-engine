const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
const pathfinding = @import("pathfinding.zig");
const rl = grid.rl;

pub const Box = struct {
    topLeft: types.ScreenPos,
    bottomRight: types.ScreenPos,
};

// pub const

pub fn insideBox(pos: types.ScreenPos, box: Box) bool {
    return pos.x >= box.topLeft.x and pos.x <= box.bottomRight.x and pos.y >= box.topLeft.y and pos.y <= box.bottomRight.y;
}

pub fn updateBox(box: *Box) void {
    if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
        if (box.topLeft.x == 0 and box.topLeft.y == 0) {
            box.topLeft.x = rl.GetMouseX();
            box.topLeft.y = rl.GetMouseY();
        } else {
            box.bottomRight.x = rl.GetMouseX();
            box.bottomRight.y = rl.GetMouseY();
        }
    } else {
        box.topLeft = Box{};
    }
}

pub fn drawBox(box: Box) void {
    if (box.topLeft.x == 0) {
        return;
    }
    // transparent box
    rl.DrawRectangle(box.topLeft.x, box.topLeft.y, box.bottomRight.x - box.topLeft.x, box.bottomRight.y - box.topLeft.y, rl.Fade(rl.BLUE, 0.5));
}

// draw transparent selection box
// if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
//     const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };
// } else {}
