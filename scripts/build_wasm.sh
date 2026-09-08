#!/usr/bin/env bash
# Builds the optimal_parking core + embind bindings into web/public/wasm/.
# Requires: emsdk activated (em++ on PATH) and OSQP/osqp-eigen/yaml-cpp built
# for wasm32 under $WASM_DEPS_PREFIX (see docs/wasm-deps.md for how to build them).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WASM_DEPS_PREFIX="${WASM_DEPS_PREFIX:-$HOME/wasm-deps/install}"
EIGEN3_INCLUDE_DIR="${EIGEN3_INCLUDE_DIR:-/usr/local/include/eigen3}"
OUT_DIR="$ROOT_DIR/web/public/wasm"
BUILD_DIR="$ROOT_DIR/optimal_parking/build-wasm"

if ! command -v em++ >/dev/null 2>&1; then
    echo "em++ not found. Run 'source <emsdk>/emsdk_env.sh' first." >&2
    exit 1
fi

mkdir -p "$BUILD_DIR/objs" "$OUT_DIR"

INCLUDES=(
    "-I$ROOT_DIR/optimal_parking/include"
    "-I$EIGEN3_INCLUDE_DIR"
    "-I$WASM_DEPS_PREFIX/include"
    "-I$WASM_DEPS_PREFIX/include/osqp"
    "-I$WASM_DEPS_PREFIX/include/yaml-cpp"
    "-DOSQP_EIGEN_OSQP_IS_V1"
    "-DOSQP_EIGEN_OSQP_IS_V1_FINAL"
)

SOURCES=(
    config.cpp
    qp_solver.cpp
    rrt_star.cpp
    trajectory_optimizer.cpp
    system/system_state.cpp
    system/system_model.cpp
    system/system_input.cpp
)

for src in "${SOURCES[@]}"; do
    obj="$BUILD_DIR/objs/$(basename "$src" .cpp).o"
    echo "compiling $src"
    em++ -std=c++20 -O2 "${INCLUDES[@]}" -c "$ROOT_DIR/optimal_parking/src/$src" -o "$obj"
done

echo "compiling bindings.cpp"
em++ -std=c++20 -O2 "${INCLUDES[@]}" -c "$ROOT_DIR/optimal_parking/bindings/wasm/bindings.cpp" \
    -o "$BUILD_DIR/objs/bindings.o"

echo "linking planner.js/.wasm"
em++ -std=c++20 -O2 --bind \
    "$BUILD_DIR"/objs/*.o \
    "$WASM_DEPS_PREFIX/lib/libOsqpEigen.a" \
    "$WASM_DEPS_PREFIX/lib/libosqpstatic.a" \
    "$WASM_DEPS_PREFIX/lib/libyaml-cpp.a" \
    --preload-file "$ROOT_DIR/example/config.yaml@/config.yaml" \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s MODULARIZE=1 \
    -s EXPORT_ES6=1 \
    -s EXPORT_NAME=createPlannerModule \
    -o "$OUT_DIR/planner.js"

echo "done: $OUT_DIR/planner.js"
