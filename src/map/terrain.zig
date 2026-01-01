const std = @import("std");
const types = @import("../types.zig");
const noise = @import("noise.zig");

pub const TerrainType = enum {
    flat,
    hill,
    mountain,
    peak,
};

pub const TerrainConfig = struct {
    seed: u64,
    scale: f32 = 50.0,
    octaves: u32 = 4,
    persistence: f32 = 0.5,
    lacunarity: f32 = 2.0,
    ridgeStrength: f32 = 0.7,
    baseHeight: f32 = 0.3,
    mountainThreshold: f32 = 0.5,
    peakThreshold: f32 = 0.7,
};

pub const DEFAULT_TERRAIN_CONFIG: TerrainConfig = .{
    .seed = 12345,
    .scale = 50.0,
    .octaves = 4,
    .persistence = 0.5,
    .lacunarity = 2.0,
    .ridgeStrength = 0.7,
    .baseHeight = 0.3,
    .mountainThreshold = 0.5,
    .peakThreshold = 0.7,
};

pub const MOUNTAINOUS: TerrainConfig = .{
    .seed = 12345,
    .scale = 60.0,
    .octaves = 5,
    .persistence = 0.6,
    .lacunarity = 2.0,
    .ridgeStrength = 0.8,
    .baseHeight = 0.2,
    .mountainThreshold = 0.4,
    .peakThreshold = 0.65,
};

pub const ROLLING_HILLS: TerrainConfig = .{
    .seed = 12345,
    .octaves = 3,
    .persistence = 0.4,
    .lacunarity = 2.0,
    .ridgeStrength = 0.4,
    .baseHeight = 0.4,
    .mountainThreshold = 0.6,
    .peakThreshold = 0.8,
};

pub fn generateHeightmap(allocator: std.mem.Allocator, width: usize, height: usize, config: TerrainConfig) ![][]f32 {
    var perlin = noise.PerlinNoise.init(config.seed);

    var heightmap = try allocator.alloc([]f32, height);
    errdefer {
        for (heightmap) |row| allocator.free(row);
        allocator.free(heightmap);
    }

    for (0..height) |y| {
        heightmap[y] = try allocator.alloc(f32, width);

        for (0..width) |x| {
            const fx: f32 = @floatFromInt(x);
            const fy: f32 = @floatFromInt(y);

            const sx = fx / config.scale;
            const sy = fy / config.scale;

            const baseNoise = perlin.noise2D(sx, sy);

            const ridgeNoiseValue = perlin.noise2D(sx * 0.5, sy * 0.5);
            const ridge = noise.PerlinNoise.ridgeNoise(ridgeNoiseValue);

            const detail = perlin.fractalBrownianMotion(sx * 2.0, sy * 2.0, config.octaves, config.persistence, config.lacunarity);

            var heightValue = config.baseHeight + baseNoise * 0.2 + ridge * config.ridgeStrength * 0.5 + detail * 0.1;

            heightValue = std.math.clamp(heightValue, 0.0, 1.0);
            heightmap[y][x] = heightValue;
        }
    }

    return heightmap;
}

pub fn getTerrainType(height: f32, config: TerrainConfig) TerrainType {
    if (height < config.mountainThreshold) {
        return .hill;
    } else if (height < config.peakThreshold) {
        return .mountain;
    } else {
        return .peak;
    }
}

pub fn freeHeightmap(allocator: std.mem.Allocator, heightmap: [][]f32) void {
    for (heightmap) |row| {
        allocator.free(row);
    }
    allocator.free(heightmap);
}
