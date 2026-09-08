#!/usr/bin/env bash
# Builds OSQP, osqp-eigen, and yaml-cpp as static libraries for wasm32-emscripten.
# Used by both local dev setup and CI; safe to re-run (skips already-built deps).
#
# Requires: emsdk activated (em++/emcmake/emmake on PATH), Eigen3 already installed
# natively (only its headers are needed, e.g. via apt/the main Dockerfile).
set -euo pipefail

WASM_DEPS_PREFIX="${WASM_DEPS_PREFIX:-$HOME/wasm-deps/install}"
WORK_DIR="${WASM_DEPS_WORK_DIR:-$HOME/wasm-deps}"
EIGEN3_CMAKE_DIR="${EIGEN3_CMAKE_DIR:-/usr/local/share/eigen3/cmake}"

mkdir -p "$WORK_DIR" "$WASM_DEPS_PREFIX"

if ! command -v em++ >/dev/null 2>&1; then
    echo "em++ not found. Run 'source <emsdk>/emsdk_env.sh' first." >&2
    exit 1
fi

# --- OSQP (static lib only; interrupts/demo executables need OS features wasm lacks) ---
if [ ! -f "$WASM_DEPS_PREFIX/lib/libosqpstatic.a" ]; then
    echo "== Building OSQP for wasm =="
    [ -d "$WORK_DIR/osqp" ] || git clone --branch v1.0.0 --recursive https://github.com/osqp/osqp.git "$WORK_DIR/osqp"
    mkdir -p "$WORK_DIR/osqp/build-wasm" && cd "$WORK_DIR/osqp/build-wasm"
    emcmake cmake \
        -DOSQP_BUILD_DEMO_EXE=OFF \
        -DOSQP_ENABLE_INTERRUPT=OFF \
        -DCMAKE_INSTALL_PREFIX="$WASM_DEPS_PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        ..
    emmake make -j"$(nproc)" install
    # osqp_api_utils.h (csc_set_data/OSQPCscMatrix_set_data) isn't installed by this
    # OSQP version's install rules; copy it manually so osqp-eigen can find it.
    cp "$WORK_DIR/osqp/include/public/osqp_api_utils.h" "$WASM_DEPS_PREFIX/include/"
else
    echo "== OSQP for wasm already built, skipping =="
fi

# --- osqp-eigen (v0.11.2 supports OSQP v1's renamed API; static-only build) ---
if [ ! -f "$WASM_DEPS_PREFIX/lib/libOsqpEigen.a" ]; then
    echo "== Building osqp-eigen for wasm =="
    [ -d "$WORK_DIR/osqp-eigen" ] || git clone https://github.com/robotology/osqp-eigen.git "$WORK_DIR/osqp-eigen"
    (cd "$WORK_DIR/osqp-eigen" && git checkout v0.11.2)
    rm -rf "$WORK_DIR/osqp-eigen/build-wasm" && mkdir -p "$WORK_DIR/osqp-eigen/build-wasm"
    cd "$WORK_DIR/osqp-eigen/build-wasm"
    emcmake cmake \
        -Dosqp_DIR="$WASM_DEPS_PREFIX/lib/cmake/osqp" \
        -DEigen3_DIR="$EIGEN3_CMAKE_DIR" \
        -DCMAKE_PREFIX_PATH="$WASM_DEPS_PREFIX" \
        -DCMAKE_INSTALL_PREFIX="$WASM_DEPS_PREFIX" \
        -DBUILD_TESTING=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DOSQP_IS_V1=TRUE \
        -DOSQP_IS_V1_FINAL=TRUE \
        -DCMAKE_BUILD_TYPE=Release \
        ..
    emmake make -j"$(nproc)" install
else
    echo "== osqp-eigen for wasm already built, skipping =="
fi

# --- yaml-cpp (static lib; lets TrajectoryOptimizer's file-based config loader work unchanged) ---
if [ ! -f "$WASM_DEPS_PREFIX/lib/libyaml-cpp.a" ]; then
    echo "== Building yaml-cpp for wasm =="
    [ -d "$WORK_DIR/yaml-cpp" ] || git clone --depth 1 https://github.com/jbeder/yaml-cpp.git "$WORK_DIR/yaml-cpp"
    mkdir -p "$WORK_DIR/yaml-cpp/build-wasm" && cd "$WORK_DIR/yaml-cpp/build-wasm"
    emcmake cmake \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF \
        -DYAML_BUILD_SHARED_LIBS=OFF \
        -DCMAKE_INSTALL_PREFIX="$WASM_DEPS_PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        ..
    emmake make -j"$(nproc)" install
else
    echo "== yaml-cpp for wasm already built, skipping =="
fi

echo "done: wasm deps installed to $WASM_DEPS_PREFIX"
