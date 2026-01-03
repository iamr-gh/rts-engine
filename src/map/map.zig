const std = @import("std");
const types = @import("../types.zig");
const grid = @import("grid.zig");
pub const terrain = @import("terrain.zig");
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

    pub fn contains(self: *const ObstacleGrid, coord: GridCoord) bool {
        return self.obstacles.contains(coord);
    }
};

pub fn generateRandomObstacles(allocator: std.mem.Allocator, gridSize: i32, screenWidth: i32, screenHeight: i32, excludePos: ScreenPos) !ObstacleGrid {
    var rng = std.Random.DefaultPrng.init(@intCast(std.time.timestamp()));
    const random = rng.random();

    var obstacleGrid = ObstacleGrid.init(allocator);
    errdefer obstacleGrid.deinit();

    const hexWidth: f32 = @as(f32, @floatFromInt(gridSize)) * 0.75;
    const hexHeight: f32 = @as(f32, @floatFromInt(gridSize)) * std.math.sqrt(3.0) / 2.0;
    const numCols: i32 = @intFromFloat(@as(f32, @floatFromInt(screenWidth)) / hexWidth);
    const numRows: i32 = @intFromFloat(@as(f32, @floatFromInt(screenHeight)) / hexHeight);

    const numColsUsize: usize = @intCast(numCols);
    const numRowsUsize: usize = @intCast(numRows);

    for (0..numColsUsize) |col| {
        for (0..numRowsUsize) |row| {
            const axial = grid.offsetToAxial(@intCast(col), @intCast(row));
            const center = grid.axialToScreen(axial[0], axial[1], gridSize);

            if (std.meta.eql(center, excludePos)) continue;

            if (random.float(f32) < 0.3) {
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
            } else if (random.float(f32) < 0.3) {
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
                try obstacleGrid.add(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) });
            }
        }
    }

    return obstacleGrid;
}

pub fn generateTerrainObstacles(allocator: std.mem.Allocator, gridSize: i32, screenWidth: i32, screenHeight: i32, excludePos: ScreenPos) !ObstacleGrid {
    return generateTerrainObstaclesWithConfig(allocator, gridSize, screenWidth, screenHeight, excludePos, terrain.DEFAULT_TERRAIN_CONFIG);
}

pub fn generateTerrainObstaclesWithConfig(allocator: std.mem.Allocator, gridSize: i32, screenWidth: i32, screenHeight: i32, excludePos: ScreenPos, config: terrain.TerrainConfig) !ObstacleGrid {
    const hexWidth: f32 = @as(f32, @floatFromInt(gridSize)) * 0.75;
    const hexHeight: f32 = @as(f32, @floatFromInt(gridSize)) * std.math.sqrt(3.0) / 2.0;
    const numCols: i32 = @intFromFloat(@as(f32, @floatFromInt(screenWidth)) / hexWidth);
    const numRows: i32 = @intFromFloat(@as(f32, @floatFromInt(screenHeight)) / hexHeight);

    const numColsUsize: usize = @intCast(numCols);
    const numRowsUsize: usize = @intCast(numRows);

    const heightmap = try terrain.generateHeightmap(allocator, numColsUsize, numRowsUsize, config);
    defer terrain.freeHeightmap(allocator, heightmap);

    const features = terrain.getCanyonMountainRangeFeatures();
    terrain.applyFeaturesToHeightmap(heightmap, features);

    var obstacleGrid = ObstacleGrid.init(allocator);
    errdefer obstacleGrid.deinit();

    for (0..numColsUsize) |col| {
        for (0..numRowsUsize) |row| {
            const axial = grid.offsetToAxial(@intCast(col), @intCast(row));
            const center = grid.axialToScreen(axial[0], axial[1], gridSize);

            if (std.meta.eql(center, excludePos)) continue;

            const height = heightmap[row][col];
            const obstacleHeight = terrain.getMountainHeight(height, config);

            if (obstacleHeight == 0) {
                continue;
            }

            try obstacleGrid.obstacles.put(.{ .x = @as(i32, @intCast(col)), .y = @as(i32, @intCast(row)) }, obstacleHeight);
        }
    }

    return obstacleGrid;
}

pub fn screenToGridCoord(screenPos: ScreenPos, gridSize: i32) GridCoord {
    const axial = grid.getHexContainingPos(screenPos, gridSize);
    const offset = grid.axialToOffset(axial[0], axial[1]);
    return .{ .x = offset[0], .y = offset[1] };
}

pub fn isObstacle(obstacleGrid: *const ObstacleGrid, screenPos: ScreenPos, gridSize: i32) bool {
    const axial = grid.getHexContainingPos(screenPos, gridSize);
    const offset = grid.axialToOffset(axial[0], axial[1]);
    const coord = GridCoord{ .x = offset[0], .y = offset[1] };
    return obstacleGrid.contains(coord);
}

pub fn drawObstacles(obstacleGrid: *const ObstacleGrid, gridSize: i32) void {
    var iter = obstacleGrid.obstacles.iterator();
    while (iter.next()) |entry| {
        const coord = entry.key_ptr.*;
        const axial = grid.offsetToAxial(coord.x, coord.y);
        const center = grid.axialToScreen(axial[0], axial[1], gridSize);

        switch (entry.value_ptr.*) {
            1 => grid.drawHex(center, gridSize, .{ .r = 200, .g = 200, .b = 200, .a = 255 }),
            2 => grid.drawHex(center, gridSize, .{ .r = 150, .g = 150, .b = 150, .a = 255 }),
            3 => grid.drawHex(center, gridSize, .{ .r = 100, .g = 100, .b = 100, .a = 255 }),
            4 => grid.drawHex(center, gridSize, .{ .r = 50, .g = 50, .b = 50, .a = 255 }),
            else => {},
        }
    }
}
