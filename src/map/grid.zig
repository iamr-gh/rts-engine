const std = @import("std");

pub const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

const types = @import("../types.zig");
const ScreenPos = types.ScreenPos;

pub const HEX_DIRECTIONS = [6][2]i32{
    .{ 1, 0 }, // E
    .{ 1, -1 }, // SE
    .{ 0, -1 }, // SW
    .{ -1, 0 }, // W
    .{ -1, 1 }, // NW
    .{ 0, 1 }, // NE
};

pub const HEX_WIDTH_SIZE_RATIO: f32 = 1.0;
pub const HEX_HEIGHT_RATIO: f32 = std.math.sqrt(3.0) / 2.0;

pub fn axialToScreen(q: i32, r: i32, size: i32) ScreenPos {
    const qf: f32 = @floatFromInt(q);
    const rf: f32 = @floatFromInt(r);
    const sizef: f32 = @floatFromInt(size);

    const x = sizef * 3.0 / 2.0 * qf;
    const y = sizef * std.math.sqrt(3.0) * (rf + qf / 2.0);

    return .{ .x = @intFromFloat(x), .y = @intFromFloat(y) };
}

pub fn screenToAxial(pos: ScreenPos, size: i32) [2]i32 {
    const posf_x: f32 = @floatFromInt(pos.x);
    const posf_y: f32 = @floatFromInt(pos.y);
    const sizef: f32 = @floatFromInt(size);

    const q = (2.0 / 3.0 * posf_x) / sizef;
    const r = (-1.0 / 3.0 * posf_x + std.math.sqrt(3.0) / 3.0 * posf_y) / sizef;

    return axialRound(q, r);
}

fn axialRound(qf: f32, rf: f32) [2]i32 {
    const q = qf;
    const r = rf;
    const s = -q - r;

    var q_round = @round(q);
    var r_round = @round(r);
    var s_round = @round(s);

    const q_diff = @abs(q_round - q);
    const r_diff = @abs(r_round - r);
    const s_diff = @abs(s_round - s);

    if (q_diff > r_diff and q_diff > s_diff) {
        q_round = -r_round - s_round;
    } else if (r_diff > s_diff) {
        r_round = -q_round - s_round;
    } else {
        s_round = -q_round - r_round;
    }

    return .{ @as(i32, @intFromFloat(q_round)), @as(i32, @intFromFloat(r_round)) };
}

pub fn axialToOffset(q: i32, r: i32) [2]i32 {
    const col = q;
    const row = r + @divFloor((q - (q & 1)), 2);
    return .{ col, row };
}

pub fn offsetToAxial(col: i32, row: i32) [2]i32 {
    const q = col;
    const r = row - @divFloor((col - (col & 1)), 2);
    return .{ q, r };
}

pub fn getHexCorners(center: ScreenPos, size: i32) [6]ScreenPos {
    const cx: f32 = @floatFromInt(center.x);
    const cy: f32 = @floatFromInt(center.y);
    const sizef: f32 = @floatFromInt(size);

    var corners: [6]ScreenPos = undefined;
    for (0..6) |i| {
        const angle_deg = @as(f32, @floatFromInt(i * 60));
        const angle_rad = angle_deg * std.math.pi / 180.0;
        corners[i] = .{
            .x = @intFromFloat(cx + sizef / 2.0 * @cos(angle_rad)),
            .y = @intFromFloat(cy + sizef / 2.0 * @sin(angle_rad)),
        };
    }
    return corners;
}

pub fn drawHex(center: ScreenPos, size: i32, color: rl.Color) void {
    rl.DrawPoly(rl.Vector2{ .x = @floatFromInt(center.x), .y = @floatFromInt(center.y) }, 6, @as(f32, @floatFromInt(size)), 0, color);
}

pub fn drawHexOutline(center: ScreenPos, size: i32, color: rl.Color) void {
    const hexRadius: f32 = @as(f32, @floatFromInt(size)) / 2.0;
    rl.DrawPolyLines(rl.Vector2{ .x = @floatFromInt(center.x), .y = @floatFromInt(center.y) }, 6, hexRadius, 0, color);
}

pub fn hexDistance(a: [2]i32, b: [2]i32) i32 {
    const aq = a[0];
    const ar = a[1];
    const as = -aq - ar;

    const bq = b[0];
    const br = b[1];
    const bs = -bq - br;

    const abs_q = @abs(aq - bq);
    const abs_r = @abs(ar - br);
    const abs_s = @abs(as - bs);

    if (abs_q >= abs_r and abs_q >= abs_s) return @intCast(abs_q);
    if (abs_r >= abs_q and abs_r >= abs_s) return @intCast(abs_r);
    return @intCast(abs_s);
}

pub fn getHexNeighbors(q: i32, r: i32) [6][2]i32 {
    var neighbors: [6][2]i32 = undefined;
    for (0..6) |i| {
        neighbors[i] = .{ q + HEX_DIRECTIONS[i][0], r + HEX_DIRECTIONS[i][1] };
    }
    return neighbors;
}

pub fn getHexCenter(q: i32, r: i32, size: i32) ScreenPos {
    return axialToScreen(q, r, size);
}

pub fn getHexContainingPos(pos: ScreenPos, size: i32) [2]i32 {
    return screenToAxial(pos, size);
}

pub fn printHexGrid(gridSize: i32, screenWidth: i32, screenHeight: i32) void {
    const hexWidth = @as(f32, @floatFromInt(gridSize)) * 0.75;
    const hexHeight = @as(f32, @floatFromInt(gridSize)) * std.math.sqrt(3.0) / 2.0;

    const numCols: i32 = @intFromFloat(@as(f32, @floatFromInt(screenWidth)) / hexWidth);
    const numRows: i32 = @intFromFloat(@as(f32, @floatFromInt(screenHeight)) / hexHeight);

    var col: i32 = 0;
    while (col < numCols) : (col += 1) {
        var row: i32 = 0;
        while (row < numRows) : (row += 1) {
            const axial = offsetToAxial(col, row);
            const center = axialToScreen(axial[0], axial[1], gridSize);
            drawHexOutline(center, gridSize, rl.LIGHTGRAY);
            row += 1;
        }
        col += 1;
    }
}
