const std = @import("std");
const types = @import("types.zig");
const grid = @import("map/grid.zig");
const rl = grid.rl;

pub const Box = struct {
    src: types.ScreenPos = types.ScreenPos{},
    dst: types.ScreenPos = types.ScreenPos{},
    active: bool = false,
};

pub fn insideBox(pos: types.ScreenPos, box: Box) bool {
    const minX = @min(box.src.x, box.dst.x);
    const maxX = @max(box.src.x, box.dst.x);
    const minY = @min(box.src.y, box.dst.y);
    const maxY = @max(box.src.y, box.dst.y);

    return pos.x >= minX and pos.x <= maxX and pos.y >= minY and pos.y <= maxY;
}

pub fn updateBox(box: *Box) void {
    if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
        box.src.x = rl.GetMouseX();
        box.src.y = rl.GetMouseY();
        box.dst = box.src;
        box.active = true;
    } else if (rl.IsMouseButtonDown(rl.MOUSE_LEFT_BUTTON) and box.active) {
        box.dst.x = rl.GetMouseX();
        box.dst.y = rl.GetMouseY();
    } else {
        box.active = false;
    }
}

pub fn drawBox(box: Box) void {
    if (!box.active) {
        return;
    }

    const x = @min(box.src.x, box.dst.x);
    const y = @min(box.src.y, box.dst.y);
    const width = @max(box.src.x, box.dst.x) - @min(box.src.x, box.dst.x);
    const height = @max(box.src.y, box.dst.y) - @min(box.src.y, box.dst.y);

    if (width == 0 or height == 0) {
        return;
    }

    rl.DrawRectangle(x, y, width, height, rl.Fade(rl.BLUE, 0.25));
}
