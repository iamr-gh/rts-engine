const std = @import("std");

fn concatTuples(a: anytype, b: anytype) concatType(@TypeOf(a), @TypeOf(b)) {
    const T1 = @TypeOf(a);
    const T2 = @TypeOf(b);
    const len1 = @typeInfo(T1).@"struct".fields.len;
    const len2 = @typeInfo(T2).@"struct".fields.len;

    var result: concatType(T1, T2) = undefined;

    inline for (0..len1) |i| result[i] = a[i];
    inline for (0..len2) |i| result[i + len1] = b[i];

    return result;
}

/// Helper to calculate the result type separately
fn concatType(comptime T1: type, comptime T2: type) type {
    const fields1 = @typeInfo(T1).@"struct".fields;
    const fields2 = @typeInfo(T2).@"struct".fields;

    var types: [fields1.len + fields2.len]type = undefined;
    inline for (fields1, 0..) |f, i| types[i] = f.type;
    inline for (fields2, 0..) |f, i| types[i + fields1.len] = f.type;

    return std.meta.Tuple(&types);
}

fn makeFunc(comptime T: type, comptime f: anytype, comptime provided: anytype) T {
    const info = @typeInfo(T);
    const Fn = info.@"fn";
    const params = Fn.params;

    // have to hardcode arity in funciton genration due to stricter anytype checking in zig .15
    return switch (params.len) {
        0 => struct {
            fn wrapper() Fn.return_type.? {
                std.debug.print("Called with 0 args\n", .{});
                return @call(.auto, f, provided);
            }
        }.wrapper,
        1 => struct {
            fn wrapper(a: params[0].type.?) Fn.return_type.? {
                const in = concatTuples(provided, .{a});
                return @call(.auto, f, in);
            }
        }.wrapper,
        2 => struct {
            fn wrapper(a: params[0].type.?, b: params[1].type.?) Fn.return_type.? {
                const in = concatTuples(provided, .{ a, b });
                return @call(.auto, f, in);
            }
        }.wrapper,
        3 => struct {
            fn wrapper(a: params[0].type.?, b: params[1].type.?, c: params[2].type.?) Fn.return_type.? {
                const in = concatTuples(provided, .{ a, b, c });
                return @call(.auto, f, in);
            }
        }.wrapper,
        4 => struct {
            fn wrapper(a: params[0].type.?, b: params[1].type.?, c: params[2].type.?, d: params[3].type.?) Fn.return_type.? {
                const in = concatTuples(provided, .{ a, b, c, d });
                return @call(.auto, f, in);
            }
        }.wrapper,
        5 => struct {
            fn wrapper(a: params[0].type.?, b: params[1].type.?, c: params[2].type.?, d: params[3].type.?, e: params[4].type.?) Fn.return_type.? {
                const in = concatTuples(provided, .{ a, b, c, d, e });
                return @call(.auto, f, in);
            }
        }.wrapper,
        else => @compileError("Arity too high! Add more cases to the switch."),
    };
}

fn partialType(comptime args: type, comptime f: type) type {
    const args_info = @typeInfo(args);
    if (args_info != .@"struct") {
        @compileError("Expected a tuple or struct, found " ++ @typeName(args));
    }

    var f_info = @typeInfo(f);
    if (f_info != .@"fn") {
        @compileError("Expected a tuple or struct, found " ++ @typeName(args));
    }

    const original_arg_types = f_info.@"fn".params;
    f_info.@"fn".params = original_arg_types[args_info.@"struct".fields.len..];
    return @Type(f_info);
}

// probably a bad pattern, but fun
pub fn partial(args: anytype, comptime f: anytype) partialType(@TypeOf(args), @TypeOf(f)) {
    const out_t = partialType(@TypeOf(args), @TypeOf(f));
    return makeFunc(out_t, f, args);
}

fn condNum(b: bool, a: i32) i32 {
    if (b) {
        return a;
    } else {
        return 0;
    }
}

fn num(a: i32) i32 {
    return a;
}

test "partialApplication" {
    // type propagation
    std.debug.assert(partialType(@TypeOf(.{false}), @TypeOf(condNum)) == @TypeOf(num));

    const num1 = partial(.{1}, num);
    std.debug.assert(num1() == 1);

    // partial application
    const condTrue = partial(.{true}, condNum);
    std.debug.assert(condTrue(1) == 1);

    const condFalse = partial(.{false}, condNum);
    std.debug.assert(condFalse(1) == 0);
}

// linked list implementation of a queue
// should eventually switch to a deque or something else
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
