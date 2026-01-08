const std = @import("std");

/// Computes the function type after removing the first `n` parameters.
fn PartialFn(comptime F: type, comptime n: usize) type {
    const info = @typeInfo(F).@"fn";
    var new_params: [info.params.len - n]std.builtin.Type.Fn.Param = undefined;
    for (info.params[n..], 0..) |p, i| {
        new_params[i] = p;
    }
    return @Type(.{ .@"fn" = .{
        .calling_convention = info.calling_convention,
        .is_generic = false,
        .is_var_args = info.is_var_args,
        .return_type = info.return_type,
        .params = &new_params,
    } });
}

/// Partial function application - binds the first N arguments of a function at compile time.
/// Returns a concrete function pointer with the bound parameters removed from the signature.
///
/// Example:
///   fn add(a: i32, b: i32) i32 { return a + b; }
///   const addFive = partial(add, .{5});  // type: fn(i32) i32
///   addFive(3);  // returns 8
///
pub fn partial(comptime f: anytype, comptime bound_args: anytype) PartialFn(@TypeOf(f), bound_args.len) {
    const F = @TypeOf(f);
    const info = @typeInfo(F).@"fn";
    const bound_count = bound_args.len;
    const remaining_count = info.params.len - bound_count;
    const p = info.params;

    const Impl = struct {
        inline fn callWithArgs(remaining_tuple: std.meta.ArgsTuple(PartialFn(F, bound_count))) info.return_type.? {
            var full_args: std.meta.ArgsTuple(F) = undefined;
            inline for (0..bound_count) |i| {
                full_args[i] = bound_args[i];
            }
            inline for (0..remaining_count) |i| {
                full_args[bound_count + i] = remaining_tuple[i];
            }
            return @call(.auto, f, full_args);
        }
    };

    return switch (remaining_count) {
        0 => struct {
            fn call() info.return_type.? {
                return Impl.callWithArgs(.{});
            }
        }.call,
        1 => struct {
            fn call(a0: p[bound_count].type.?) info.return_type.? {
                return Impl.callWithArgs(.{a0});
            }
        }.call,
        2 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1 });
            }
        }.call,
        3 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?, a2: p[bound_count + 2].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1, a2 });
            }
        }.call,
        4 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?, a2: p[bound_count + 2].type.?, a3: p[bound_count + 3].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1, a2, a3 });
            }
        }.call,
        5 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?, a2: p[bound_count + 2].type.?, a3: p[bound_count + 3].type.?, a4: p[bound_count + 4].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1, a2, a3, a4 });
            }
        }.call,
        6 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?, a2: p[bound_count + 2].type.?, a3: p[bound_count + 3].type.?, a4: p[bound_count + 4].type.?, a5: p[bound_count + 5].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1, a2, a3, a4, a5 });
            }
        }.call,
        7 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?, a2: p[bound_count + 2].type.?, a3: p[bound_count + 3].type.?, a4: p[bound_count + 4].type.?, a5: p[bound_count + 5].type.?, a6: p[bound_count + 6].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1, a2, a3, a4, a5, a6 });
            }
        }.call,
        8 => struct {
            fn call(a0: p[bound_count].type.?, a1: p[bound_count + 1].type.?, a2: p[bound_count + 2].type.?, a3: p[bound_count + 3].type.?, a4: p[bound_count + 4].type.?, a5: p[bound_count + 5].type.?, a6: p[bound_count + 6].type.?, a7: p[bound_count + 7].type.?) info.return_type.? {
                return Impl.callWithArgs(.{ a0, a1, a2, a3, a4, a5, a6, a7 });
            }
        }.call,
        else => @compileError("partial: too many remaining arguments (max 8 supported)"),
    };
}

// linked list implementation of a queue
pub fn Queue(comptime T: type) type {
    return struct {
        const Self = @This();
        const Node = struct {
            data: T,
            next: ?*Node,
        };

        allocator: std.mem.Allocator,
        head: ?*Node,
        tail: ?*Node,

        pub fn init(allocator: std.mem.Allocator) Self {
            return .{
                .allocator = allocator,
                .head = null,
                .tail = null,
            };
        }

        pub fn enqueue(self: *Self, value: T) !void {
            const node = try self.allocator.create(Node);
            node.* = .{ .data = value, .next = null };

            if (self.tail) |tail| {
                tail.next = node;
            } else {
                self.head = node;
            }
            self.tail = node;
        }

        pub fn dequeue(self: *Self) ?T {
            const head = self.head orelse return null;
            defer self.allocator.destroy(head);

            self.head = head.next;
            if (head.next == null) self.tail = null;

            return head.data;
        }

        pub fn deinit(self: *Self) void {
            // Dequeue all remaining items to free their nodes
            while (self.dequeue()) |_| {}
        }
    };
}

// ============ TESTS ============

fn testAdd(a: i32, b: i32) i32 {
    return a + b;
}

fn testTriple(a: i32, b: bool, c: f32) f32 {
    return if (b) @as(f32, @floatFromInt(a)) + c else c;
}

fn testMayFail(a: i32, b: i32) !i32 {
    if (b == 0) return error.DivByZero;
    return @divTrunc(a, b);
}

test "partial - single argument binding" {
    const addFive = partial(testAdd, .{5});
    try std.testing.expectEqual(8, addFive(3));
    try std.testing.expectEqual(5, addFive(0));
}

test "partial - multiple argument binding" {
    const bound = partial(testTriple, .{ 10, true });
    try std.testing.expectEqual(15.5, bound(5.5));
}

test "partial - all arguments bound" {
    const allBound = partial(testAdd, .{ 2, 3 });
    try std.testing.expectEqual(5, allBound());
}

test "partial - error return type" {
    const div10 = partial(testMayFail, .{10});
    try std.testing.expectEqual(5, try div10(2));
    try std.testing.expectError(error.DivByZero, div10(0));
}

test "partial - returns concrete function pointer" {
    const addFive = partial(testAdd, .{5});
    const f: *const fn (i32) i32 = addFive;
    try std.testing.expectEqual(15, f(10));
}

test "partial - function pointers are comparable" {
    const addFive = partial(testAdd, .{5});
    const addTen = partial(testAdd, .{10});
    try std.testing.expect(addFive != addTen);
    try std.testing.expect(addFive == addFive);
}

test "partial - works with struct methods" {
    const Counter = struct {
        value: i32,
        pub fn addTo(self: *const @This(), n: i32) i32 {
            return self.value + n;
        }
    };
    const counter = Counter{ .value = 100 };
    const addToCounter = partial(Counter.addTo, .{&counter});
    try std.testing.expectEqual(142, addToCounter(42));
}
