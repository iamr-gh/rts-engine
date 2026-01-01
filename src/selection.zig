const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const map = @import("map/map.zig");
const pathfinding = @import("pathfinding.zig");
const rl = grid.rl;

pub const Box = struct {
    src: types.ScreenPos = types.ScreenPos{},
    dst: types.ScreenPos = types.ScreenPos{},
};

pub fn insideBox(pos: types.ScreenPos, box: Box) bool {
    return pos.x >= box.src.x and pos.x <= box.dst.x and pos.y >= box.src.y and pos.y <= box.dst.y;
}

pub fn updateBox(box: *Box) void {
    if (rl.IsMouseButtonDown(rl.MOUSE_LEFT_BUTTON)) {
        if (box.src.x == 0 and box.src.y == 0) {
            box.src.x = rl.GetMouseX();
            box.src.y = rl.GetMouseY();
        } else {
            box.dst.x = rl.GetMouseX();
            box.dst.y = rl.GetMouseY();
        }
    } else {
        // box.topLeft = types.ScreenPos{};
        // box.bottomRight = types.ScreenPos{};
    }
}

pub fn drawBox(box: Box) void {
    if (box.src.x == box.dst.x or box.src.y == box.dst.y) {
        return;
    }

    // transparent box
    rl.DrawRectangle(box.src.x, box.src.y, box.dst.x - box.src.x, box.dst.y - box.src.y, rl.Fade(rl.BLUE, 0.5));
}

// draw transparent selection box
// if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
//     const mousePos: ScreenPos = .{ .x = rl.GetMouseX(), .y = rl.GetMouseY() };
// } else {}
