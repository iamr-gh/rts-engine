# rts-engine

## Building

### Desktop

Run native build:
```bash
zig build run
```

### WebAssembly (WASM)

Build for web:
```bash
zig build -Dtarget=wasm32-emscripten web
```

Build and run locally with emrun:
```bash
zig build -Dtarget=wasm32-emscripten run-web
```

Output will be in `zig-out/web/` directory.

**Note**: First web build will automatically download and install Emscripten SDK (approx. 500MB).
