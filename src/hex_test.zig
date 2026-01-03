const std = @import("std");
const types = @import("types.zig");
const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

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

fn axialToScreen(q: i32, r: i32, size: i32) ScreenPos {
    const qf: f32 = @floatFromInt(q);
    const rf: f32 = @floatFromInt(r);
    const sizef: f32 = @floatFromInt(size);

    const x = sizef * 3.0 / 2.0 * qf;
    const y = sizef * std.math.sqrt(3.0) * (rf + qf / 2.0);

    return .{ .x = @intFromFloat(x), .y = @intFromFloat(y) };
}

fn screenToAxial(pos: ScreenPos, size: i32) [2]i32 {
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

fn axialToOffset(q: i32, r: i32) [2]i32 {
    const col = q;
    const row = r + @divFloor((q - (q & 1)), 2);
    return .{ col, row };
}

fn offsetToAxial(col: i32, row: i32) [2]i32 {
    const q = col;
    const r = row - @divFloor((col - (col & 1)), 2);
    return .{ q, r };
}

fn getHexCorners(center: ScreenPos, size: i32) [6]ScreenPos {
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

fn drawHex(center: ScreenPos, size: i32, color: rl.Color) void {
    rl.DrawPoly(rl.Vector2{ .x = @floatFromInt(center.x), .y = @floatFromInt(center.y) }, 6, @as(f32, @floatFromInt(size)), 0, color);
}

fn drawHexOutline(center: ScreenPos, size: i32, color: rl.Color) void {
    const hexRadius: f32 = @as(f32, @floatFromInt(size)) / 2.0;
    rl.DrawPolyLines(rl.Vector2{ .x = @floatFromInt(center.x), .y = @floatFromInt(center.y) }, 6, hexRadius, 0, color);
}

fn hexDistance(a: [2]i32, b: [2]i32) i32 {
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

fn getHexNeighbors(q: i32, r: i32) [6][2]i32 {
    var neighbors: [6][2]i32 = undefined;
    for (0..6) |i| {
        neighbors[i] = .{ q + HEX_DIRECTIONS[i][0], r + HEX_DIRECTIONS[i][1] };
    }
    return neighbors;
}

pub fn main() !void {
    const screenWidth: i32 = 1000;
    const screenHeight: i32 = 1000;
    rl.InitWindow(screenWidth, screenHeight, "Hex Grid Test");
    defer rl.CloseWindow();

    const fps: i32 = 60;
    rl.SetTargetFPS(fps);

    const gridSize: i32 = 40;

    const offsetX: i32 = screenWidth / 2;
    const offsetY: i32 = screenHeight / 2;

    var selectedHex: ?[2]i32 = null;

    while (!rl.WindowShouldClose()) {
        rl.BeginDrawing();
        defer rl.EndDrawing();

        rl.ClearBackground(rl.RAYWHITE);

        const mousePos: ScreenPos = .{ .x = rl.GetMouseX() - offsetX, .y = rl.GetMouseY() - offsetY };

        const hoveredAxial = screenToAxial(mousePos, gridSize);
        const hoveredOffset = axialToOffset(hoveredAxial[0], hoveredAxial[1]);

        const neighbors = getHexNeighbors(hoveredAxial[0], hoveredAxial[1]);
        const distanceFromOrigin = hexDistance(hoveredAxial, .{ 0, 0 });

        const hexWidthFloat: f32 = @as(f32, @floatFromInt(gridSize)) * 0.75;
        const hexHeightFloat: f32 = @as(f32, @floatFromInt(gridSize)) * std.math.sqrt(3.0) / 2.0;
        const numCols: i32 = @intFromFloat(@as(f32, @floatFromInt(screenWidth)) / hexWidthFloat);
        const numRows: i32 = @intFromFloat(@as(f32, @floatFromInt(screenHeight)) / hexHeightFloat);
        const halfCols = @divFloor(numCols, 2);
        const halfRows = @divFloor(numRows, 2);

        var col_i: i32 = -halfCols;
        while (col_i <= halfCols) : (col_i += 1) {
            var row_i: i32 = -halfRows;
            while (row_i <= halfRows) : (row_i += 1) {
                const axial = offsetToAxial(col_i, row_i);
                var center = axialToScreen(axial[0], axial[1], gridSize);
                center.x += offsetX;
                center.y += offsetY;

                const isHovered = std.meta.eql(axial, hoveredAxial);
                const isSelected = if (selectedHex) |sel| std.meta.eql(axial, sel) else false;
                const isNeighbor = blk: {
                    var found = false;
                    for (neighbors) |n| {
                        if (std.meta.eql(axial, n)) {
                            found = true;
                            break;
                        }
                    }
                    break :blk found;
                };

                var color = rl.LIGHTGRAY;
                if (isSelected) {
                    color = rl.BLUE;
                } else if (isHovered) {
                    color = rl.YELLOW;
                } else if (isNeighbor) {
                    color = rl.GREEN;
                } else {
                    const dist = hexDistance(axial, .{ 0, 0 });
                    const intensity: u8 = @intCast(@max(0, 255 - dist * 10));
                    color = rl.Color{
                        .r = intensity,
                        .g = intensity,
                        .b = intensity,
                        .a = 255,
                    };
                }

                drawHex(center, gridSize, color);
                drawHexOutline(center, gridSize, rl.DARKGRAY);
            }
        }

        if (rl.IsMouseButtonPressed(rl.MOUSE_LEFT_BUTTON)) {
            selectedHex = hoveredAxial;
        }

        var text1_buf: [100]u8 = undefined;
        const text1 = std.fmt.bufPrintZ(&text1_buf, "Axial: ({d}, {d})", .{ hoveredAxial[0], hoveredAxial[1] }) catch unreachable;
        rl.DrawText(text1, 10, 10, 20, rl.BLACK);

        var text2_buf: [100]u8 = undefined;
        const text2 = std.fmt.bufPrintZ(&text2_buf, "Offset: ({d}, {d})", .{ hoveredOffset[0], hoveredOffset[1] }) catch unreachable;
        rl.DrawText(text2, 10, 40, 20, rl.BLACK);

        var text3_buf: [100]u8 = undefined;
        const text3 = std.fmt.bufPrintZ(&text3_buf, "Distance from origin: {d}", .{distanceFromOrigin}) catch unreachable;
        rl.DrawText(text3, 10, 70, 20, rl.BLACK);

        var text4_buf: [100]u8 = undefined;
        const text4 = std.fmt.bufPrintZ(&text4_buf, "Selected: {s}", .{if (selectedHex != null) "Yes" else "No (click to select)"}) catch unreachable;
        rl.DrawText(text4, 10, 100, 20, rl.BLACK);
    }
}
