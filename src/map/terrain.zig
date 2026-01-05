const std = @import("std");
const types = @import("../types.zig");
const noise = @import("noise.zig");

pub const TerrainType = enum {
    flat,
    low_hill,
    high_hill,
    mountain,
};

pub const TerrainConfig = struct {
    seed: u64,
    scale: f32 = 50.0,
    octaves: u32 = 4,
    persistence: f32 = 0.5,
    lacunarity: f32 = 2.0,
    ridgeStrength: f32 = 0.7,
    baseHeight: f32 = 0.3,
    flatThreshold: f32 = 0.3,
    lowHillThreshold: f32 = 0.5,
    highHillThreshold: f32 = 0.65,
};

pub const DEFAULT_TERRAIN_CONFIG: TerrainConfig = .{
    .seed = 12345,
    .scale = 50.0,
    .octaves = 4,
    .persistence = 0.5,
    .lacunarity = 2.0,
    .ridgeStrength = 0.7,
    .baseHeight = 0.3,
    .flatThreshold = 0.3,
    .lowHillThreshold = 0.5,
    .highHillThreshold = 0.65,
};

pub const MOUNTAINOUS: TerrainConfig = .{
    .seed = 12345,
    .scale = 60.0,
    .octaves = 5,
    .persistence = 0.6,
    .lacunarity = 2.0,
    .ridgeStrength = 0.8,
    .baseHeight = 0.2,
    .flatThreshold = 0.25,
    .lowHillThreshold = 0.4,
    .highHillThreshold = 0.55,
};

pub const ROLLING_HILLS: TerrainConfig = .{
    .seed = 12345,
    .octaves = 3,
    .persistence = 0.4,
    .lacunarity = 2.0,
    .ridgeStrength = 0.4,
    .baseHeight = 0.4,
    .flatThreshold = 0.35,
    .lowHillThreshold = 0.55,
    .highHillThreshold = 0.7,
};

pub const PLAINS: TerrainConfig = .{
    .seed = 12345,
    .scale = 100.0,
    .octaves = 2,
    .persistence = 0.3,
    .lacunarity = 2.0,
    .ridgeStrength = 0.25,
    .baseHeight = 0.55,
    .flatThreshold = 0.45,
    .lowHillThreshold = 0.65,
    .highHillThreshold = 0.8,
};

// mostly low hill, but some smattering of high peaks
pub const FOREST: TerrainConfig = .{
    .seed = 12345,
    .scale = 100.0,
    .octaves = 2,
    .persistence = 0.3,
    .lacunarity = 2.0,
    .ridgeStrength = 0.25,
    .baseHeight = 0.55,
    .flatThreshold = 0.45,
    .lowHillThreshold = 0.7,
    .highHillThreshold = 0.65,
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
    if (height < config.flatThreshold) {
        return .flat;
    } else if (height < config.lowHillThreshold) {
        return .low_hill;
    } else if (height < config.highHillThreshold) {
        return .high_hill;
    } else {
        return .mountain;
    }
}

pub fn getMountainHeight(height: f32, config: TerrainConfig) i32 {
    if (height < config.flatThreshold) {
        return 0;
    } else if (height < config.lowHillThreshold) {
        return 1;
    } else if (height < config.highHillThreshold) {
        return 2;
    } else {
        const mountainRange = 1.0 - config.highHillThreshold;
        if (mountainRange > 0.0) {
            const normalizedMountain = (height - config.highHillThreshold) / mountainRange;
            if (normalizedMountain < 0.33) {
                return 3;
            } else {
                return 4;
            }
        }
        return 4;
    }
}

pub const Canyon = struct {
    start: [2]f32,
    end: [2]f32,
    width: f32,
    depth: f32,
};

pub const MountainRange = struct {
    waypoints: []const [2]f32,
    width: f32,
    height: f32,
};

pub const TerrainFeatures = struct {
    canyons: []const Canyon,
    mountainRanges: []const MountainRange,
};

fn distanceToSegment(px: f32, py: f32, x1: f32, y1: f32, x2: f32, y2: f32) f32 {
    const A = px - x1;
    const B = py - y1;
    const C = x2 - x1;
    const D = y2 - y1;

    const dot = A * C + B * D;
    const len_sq = C * C + D * D;
    var param: f32 = -1.0;

    if (len_sq != 0.0) {
        param = dot / len_sq;
    }

    var xx: f32 = undefined;
    var yy: f32 = undefined;

    if (param < 0.0) {
        xx = x1;
        yy = y1;
    } else if (param > 1.0) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }

    const dx = px - xx;
    const dy = py - yy;
    return std.math.sqrt(dx * dx + dy * dy);
}

fn bezierPoint(t: f32, p0: [2]f32, p1: [2]f32, p2: [2]f32) [2]f32 {
    const mt = 1.0 - t;
    const x = mt * mt * p0[0] + 2.0 * mt * t * p1[0] + t * t * p2[0];
    const y = mt * mt * p0[1] + 2.0 * mt * t * p1[1] + t * t * p2[1];
    return .{ x, y };
}

fn applyCanyonToHeightmap(heightmap: [][]f32, canyon: Canyon) void {
    const height = @as(usize, @intCast(heightmap.len));
    const width = @as(usize, @intCast(heightmap[0].len));

    for (0..height) |y| {
        for (0..width) |x| {
            const px: f32 = @as(f32, @floatFromInt(x)) / @as(f32, @floatFromInt(width - 1));
            const py: f32 = @as(f32, @floatFromInt(y)) / @as(f32, @floatFromInt(height - 1));

            const dist = distanceToSegment(px, py, canyon.start[0], canyon.start[1], canyon.end[0], canyon.end[1]);
            const influence = std.math.clamp(1.0 - dist / (canyon.width * 0.5), 0.0, 1.0);

            if (influence > 0.0) {
                const currentHeight = heightmap[y][x];
                const targetHeight = @max(0.0, currentHeight - canyon.depth * influence);
                const newHeight = currentHeight * (1.0 - influence) + targetHeight * influence;
                heightmap[y][x] = newHeight;
            }
        }
    }
}

fn applyMountainRangeToHeightmap(heightmap: [][]f32, mountainRange: MountainRange) void {
    const height = @as(usize, @intCast(heightmap.len));
    const width = @as(usize, @intCast(heightmap[0].len));

    if (mountainRange.waypoints.len < 2) return;

    for (0..height) |y| {
        for (0..width) |x| {
            const px: f32 = @as(f32, @floatFromInt(x)) / @as(f32, @floatFromInt(width - 1));
            const py: f32 = @as(f32, @floatFromInt(y)) / @as(f32, @floatFromInt(height - 1));

            var maxInfluence: f32 = 0.0;

            for (0..mountainRange.waypoints.len - 1) |i| {
                const p0 = mountainRange.waypoints[i];
                const p1 = mountainRange.waypoints[i + 1];

                const midX = (p0[0] + p1[0]) * 0.5;
                const midY = (p0[1] + p1[1]) * 0.5;

                const dx = p1[0] - p0[0];
                const dy = p1[1] - p0[1];
                const perpX = -dy;
                const perpY = dx;
                const perpLen = std.math.sqrt(perpX * perpX + perpY * perpY);

                var cp0: [2]f32 = undefined;
                var cp1: [2]f32 = undefined;
                if (perpLen > 0.0) {
                    const curveStrength = 0.3;
                    cp0 = .{ midX + perpX / perpLen * curveStrength, midY + perpY / perpLen * curveStrength };
                    cp1 = .{ midX - perpX / perpLen * curveStrength, midY - perpY / perpLen * curveStrength };
                } else {
                    cp0 = .{ midX, midY };
                    cp1 = .{ midX, midY };
                }

                var dist: f32 = std.math.floatMax(f32);

                const steps: usize = 20;
                for (0..steps + 1) |t_idx| {
                    const t = @as(f32, @floatFromInt(t_idx)) / @as(f32, @floatFromInt(steps));
                    const curvePoint = bezierPoint(t, p0, cp0, p1);
                    const curveDist = std.math.sqrt(std.math.pow(f32, px - curvePoint[0], 2.0) + std.math.pow(f32, py - curvePoint[1], 2.0));
                    if (curveDist < dist) {
                        dist = curveDist;
                    }
                }

                const influence = std.math.clamp(1.0 - dist / (mountainRange.width * 0.5), 0.0, 1.0);
                maxInfluence = @max(maxInfluence, influence);
            }

            if (maxInfluence > 0.0) {
                const currentHeight = heightmap[y][x];
                const targetHeight = @min(1.0, currentHeight + mountainRange.height * maxInfluence);
                const newHeight = currentHeight * (1.0 - maxInfluence) + targetHeight * maxInfluence;
                heightmap[y][x] = newHeight;
            }
        }
    }
}

pub fn applyFeaturesToHeightmap(heightmap: [][]f32, features: TerrainFeatures) void {
    for (features.canyons) |canyon| {
        applyCanyonToHeightmap(heightmap, canyon);
    }
    for (features.mountainRanges) |mountainRange| {
        applyMountainRangeToHeightmap(heightmap, mountainRange);
    }
}

pub const CANYON_MOUNTAIN_RANGE: TerrainConfig = .{
    .seed = 12345,
    .scale = 60.0,
    .octaves = 5,
    .persistence = 0.6,
    .lacunarity = 2.0,
    .ridgeStrength = 0.8,
    .baseHeight = 0.25,
    .flatThreshold = 0.3,
    .lowHillThreshold = 0.5,
    .highHillThreshold = 0.65,
};

pub fn getCanyonMountainRangeFeatures() TerrainFeatures {
    const canyons = [_]Canyon{
        .{
            .start = [2]f32{ 0.0, 0.4 },
            .end = [2]f32{ 1.0, 0.6 },
            .width = 0.12,
            .depth = 0.35,
        },
        .{
            .start = [2]f32{ 0.3, 0.0 },
            .end = [2]f32{ 0.4, 1.0 },
            .width = 0.10,
            .depth = 0.35,
        },
    };

    const mountainRange1_waypoints = [_][2]f32{
        [2]f32{ 0.0, 0.2 },
        [2]f32{ 0.25, 0.15 },
        [2]f32{ 0.45, 0.25 },
        [2]f32{ 0.65, 0.15 },
        [2]f32{ 1.0, 0.3 },
    };

    const mountainRange2_waypoints = [_][2]f32{
        [2]f32{ 0.0, 0.8 },
        [2]f32{ 0.3, 0.85 },
        [2]f32{ 0.5, 0.75 },
        [2]f32{ 0.75, 0.85 },
        [2]f32{ 1.0, 0.7 },
    };

    const mountainRange3_waypoints = [_][2]f32{
        [2]f32{ 0.6, 0.0 },
        [2]f32{ 0.55, 0.25 },
        [2]f32{ 0.65, 0.5 },
        [2]f32{ 0.55, 0.75 },
        [2]f32{ 0.65, 1.0 },
    };

    const mountainRanges = [_]MountainRange{
        .{
            .waypoints = &mountainRange1_waypoints,
            .width = 0.10,
            .height = 0.6,
        },
        .{
            .waypoints = &mountainRange2_waypoints,
            .width = 0.10,
            .height = 0.6,
        },
        .{
            .waypoints = &mountainRange3_waypoints,
            .width = 0.09,
            .height = 0.6,
        },
    };

    return .{
        .canyons = &canyons,
        .mountainRanges = &mountainRanges,
    };
}

pub fn freeHeightmap(allocator: std.mem.Allocator, heightmap: [][]f32) void {
    for (heightmap) |row| {
        allocator.free(row);
    }
    allocator.free(heightmap);
}
