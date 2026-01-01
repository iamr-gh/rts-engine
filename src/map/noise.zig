const std = @import("std");

pub const PerlinNoise = struct {
    rng: std.Random.DefaultPrng,

    pub fn init(seed: u64) PerlinNoise {
        return .{ .rng = std.Random.DefaultPrng.init(seed) };
    }

    pub fn noise2D(self: *PerlinNoise, x: f32, y: f32) f32 {
        const X: i32 = @intFromFloat(@floor(x));
        const Y: i32 = @intFromFloat(@floor(y));
        const xf = x - @as(f32, @floatFromInt(X));
        const yf = y - @as(f32, @floatFromInt(Y));

        const u = self.fade(xf);
        const v = self.fade(yf);

        const h0 = self.hash(X, Y);
        const h1 = self.hash(X + 1, Y);
        const h2 = self.hash(X, Y + 1);
        const h3 = self.hash(X + 1, Y + 1);

        const grad00 = self.grad(h0, xf, yf);
        const grad10 = self.grad(h1, xf - 1.0, yf);
        const grad01 = self.grad(h2, xf, yf - 1.0);
        const grad11 = self.grad(h3, xf - 1.0, yf - 1.0);

        const x1 = self.lerp(grad00, grad10, u);
        const x2 = self.lerp(grad01, grad11, u);

        return self.lerp(x1, x2, v);
    }

    pub fn fractalBrownianMotion(self: *PerlinNoise, x: f32, y: f32, octaves: u32, persistence: f32, lacunarity: f32) f32 {
        var total: f32 = 0.0;
        var frequency: f32 = 1.0;
        var amplitude: f32 = 1.0;
        var maxValue: f32 = 0.0;

        var i: u32 = 0;
        while (i < octaves) : (i += 1) {
            total += self.noise2D(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= lacunarity;
        }

        return total / maxValue;
    }

    pub fn ridgeNoise(noiseValue: f32) f32 {
        return 1.0 - @abs(noiseValue);
    }

    fn fade(_: *PerlinNoise, t: f32) f32 {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    fn lerp(_: *PerlinNoise, a: f32, b: f32, t: f32) f32 {
        return a + t * (b - a);
    }

    fn hash(self: *PerlinNoise, x: i32, y: i32) i32 {
        const h = @as(u64, @intCast(@abs(x))) * 374761393 + @as(u64, @intCast(@abs(y))) * 668265263;
        const randVal = self.rng.random().int(u32);
        return @as(i32, @intCast((h ^ @as(u64, randVal)) % 256));
    }

    fn grad(_: *PerlinNoise, hashValue: i32, x: f32, y: f32) f32 {
        const h = @as(u32, @intCast(hashValue));
        const h8 = h & 7;
        const u: f32 = if (h8 < 4) x else y;
        const v: f32 = if (h8 < 4) y else x;
        return if ((h8 & 1) != 0) -u else u + if ((h8 & 2) != 0) -2.0 * v else 2.0 * v;
    }
};
