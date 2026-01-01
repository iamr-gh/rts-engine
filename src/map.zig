const std = @import("std");
const types = @import("types.zig");
const grid = @import("grid.zig");
pub const rl = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("rlgl.h");
});

const ScreenPos = types.ScreenPos;
const GridCoord = types.GridCoord;

pub const ObstacleGrid = struct {
    obstacles: std.AutoHashMap(GridCoord, i32),

    pub fn init(allocator: std.mem.Allocator) ObstacleGrid {
        return .{
            .obstacles = std.AutoHashMap(GridCoord, i32).init(allocator),
        };
    }

    pub fn deinit(self: *ObstacleGrid) void {
        self.obstacles.deinit();
    }

    pub fn add(self: *ObstacleGrid, coord: GridCoord) !void {
        const val = (self.obstacles.get(coord) orelse 0) + 1;
        try self.obstacles.put(coord, val);
    }

    pub fn remove(self: *ObstacleGrid, coord: GridCoord) void {
        _ = self.obstacles.remove(coord);
    }

    pub fn toggle(self: *ObstacleGrid, coord: GridCoord) !void {
        const maxHeight: i32 = 2;

        if (self.obstacles.get(coord)) |height| {
            if (height == 1) {
                _ = self.obstacles.remove(coord);
            } else {
                try self.obstacles.put(coord, height - 1);
            }
        } else {
            try self.obstacles.put(coord, maxHeight);
        }
    }

    pub fn contains(self: *const ObstacleGrid, coord: GridCoord) bool {
        return self.obstacles.contains(coord);
    }
};

pub fn generateRandomObstacles(allocator: std.mem.Allocator, gridSize: i32, screenWidth: i32, screenHeight: i32, excludePos: ScreenPos, chance: [2]f32) !ObstacleGrid {
    var rng = std.Random.DefaultPrng.init(@intCast(std.time.timestamp()));
    const random = rng.random();

    var obstacleGrid = ObstacleGrid.init(allocator);
    errdefer obstacleGrid.deinit();

    const numCols = @divFloor(screenWidth, gridSize);
    const numRows = @divFloor(screenHeight, gridSize);

    const numColsUsize: usize = @intCast(numCols);
    const numRowsUsize: usize = @intCast(numRows);

    for (0..numColsUsize) |col| {
        for (0..numRowsUsize) |row| {
            const gridX = @as(i32, @intCast(col)) * gridSize;
            const gridY = @as(i32, @intCast(row)) * gridSize;
            const center = grid.getSquareCenter(gridSize, .{ .x = gridX, .y = gridY });

            if (std.meta.eql(center, excludePos)) continue;

            if (random.float(f32) < chance[0]) {
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
            } else if (random.float(f32) < chance[1]) {
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
            }
        }
    }

    return obstacleGrid;
}

pub fn screenToGridCoord(screenPos: ScreenPos, gridSize: i32) GridCoord {
    const gridSquare = grid.getSquareInGrid(gridSize, screenPos);
    const coord = GridCoord{
        .x = @divFloor(gridSquare.x, gridSize),
        .y = @divFloor(gridSquare.y, gridSize),
    };
    return coord;
}

pub fn isObstacle(obstacleGrid: *const ObstacleGrid, screenPos: ScreenPos, gridSize: i32) bool {
    const gridSquare = grid.getSquareInGrid(gridSize, screenPos);
    const coord = GridCoord{
        .x = @divFloor(gridSquare.x, gridSize),
        .y = @divFloor(gridSquare.y, gridSize),
    };
    return obstacleGrid.contains(coord);
}

pub fn canMoveDiagonal(from: ScreenPos, to: ScreenPos, gridSize: i32, obstacleGrid: *const ObstacleGrid) bool {
    const dx = to.x - from.x;
    const dy = to.y - from.y;

    const dxGrid = @divFloor(dx, gridSize);
    const dyGrid = @divFloor(dy, gridSize);

    if (dxGrid == 0 or dyGrid == 0) {
        return true;
    }

    const intermediate1 = ScreenPos{
        .x = from.x + dxGrid * gridSize,
        .y = from.y,
    };
    const intermediate2 = ScreenPos{
        .x = from.x,
        .y = from.y + dyGrid * gridSize,
    };

    return !isObstacle(obstacleGrid, intermediate1, gridSize) and !isObstacle(obstacleGrid, intermediate2, gridSize);
}

pub fn drawObstacles(obstacleGrid: *const ObstacleGrid, gridSize: i32) void {
    var iter = obstacleGrid.obstacles.iterator();
    while (iter.next()) |entry| {
        const coord = entry.key_ptr.*;
        const x = coord.x * gridSize;
        const y = coord.y * gridSize;
        if (entry.value_ptr.* == 1) {
            rl.DrawRectangle(x, y, gridSize, gridSize, rl.GRAY);
        } else if (entry.value_ptr.* == 2) {
            rl.DrawRectangle(x, y, gridSize, gridSize, rl.DARKGRAY);
        }
    }
}
