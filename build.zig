const std = @import("std");
const builtin = @import("builtin");

const WebOutputDir = "web";

fn buildDesktop(b: *std.Build, target: anytype, optimize: std.builtin.OptimizeMode) void {
    const lib_mod = b.createModule(.{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    exe_mod.addImport("rts_engine_lib", lib_mod);

    const lib = b.addLibrary(.{
        .linkage = .static,
        .name = "rts_engine",
        .root_module = lib_mod,
    });

    b.installArtifact(lib);

    const exe = b.addExecutable(.{
        .name = "rts_engine",
        .root_module = exe_mod,
    });

    const raylibDep = b.dependency("raylib", .{
        .target = target,
        .optimize = optimize,
    });

    exe.root_module.linkLibrary(raylibDep.artifact("raylib"));
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const lib_unit_tests = b.addTest(.{
        .root_module = lib_mod,
    });

    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const exe_unit_tests = b.addTest(.{
        .root_module = exe_mod,
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
    test_step.dependOn(&run_exe_unit_tests.step);
}

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    if (target.query.os_tag == .emscripten) {
        const emsdk = b.lazyDependency("emsdk", .{});

        if (emsdk) |dep| {
            const raylibDep = b.dependency("raylib", .{
                .target = target,
                .optimize = optimize,
            });

            const exe_lib = b.addLibrary(.{
                .name = "rts_engine",
                .linkage = .static,
                .root_module = b.createModule(.{
                    .target = target,
                    .optimize = optimize,
                }),
            });

            exe_lib.linkLibrary(raylibDep.artifact("raylib"));

            const emsdk_path = dep.path("upstream/emscripten").getPath(b);
            const emcc_exe = switch (builtin.os.tag) {
                .windows => "emcc.bat",
                else => "emcc",
            };

            const install_dir: std.Build.InstallDir = .{ .custom = WebOutputDir };
            const output_path = b.getInstallPath(install_dir, "index.html");

            const emcc_command = b.addSystemCommand(&[_][]const u8{b.pathJoin(&.{ emsdk_path, emcc_exe })});

            emcc_command.addArgs(&[_][]const u8{
                "-o",                                                                output_path,
                "-sFULL-ES3=1",                                                      "-sUSE_GLFW=3",
                "-sSTACK_OVERFLOW_CHECK=1",                                          "-sALLOW_MEMORY_GROWTH=1",
                "-sEXPORTED_RUNTIME_METHODS=['requestFullscreen','exitFullscreen']", "--emrun",
                "-O" ++ switch (optimize) {
                    .Debug => "0",
                    .ReleaseSafe => "2",
                    .ReleaseFast => "3",
                    .ReleaseSmall => "s",
                },
                "--shell-file",
                b.path("shell.html").getPath(b),
            });

            emcc_command.addFileArg(exe_lib.getEmittedBin());
            emcc_command.addFileArg(raylibDep.artifact("raylib").getEmittedBin());

            const web_step = b.step("web", "Build for web");
            web_step.dependOn(&emcc_command.step);

            b.getInstallStep().dependOn(&emcc_command.step);
        } else {
            std.debug.print("Emscripten SDK not found. Please run: zig build --fetch\n", .{});
        }
    } else {
        buildDesktop(b, target, optimize);
    }
}
