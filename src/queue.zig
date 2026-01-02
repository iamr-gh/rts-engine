const std = @import("std");

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
