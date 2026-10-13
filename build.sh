#!/usr/bin/env bash
# Windows (Git Bash) build helper for the MinGW toolchain that ships with Qt.
# Adjust the three paths below for your installation.
set -euo pipefail
QT_DIR=/e/Qt/6.8.3/mingw_64
MINGW_BIN=/e/Qt/Tools/mingw1310_64/bin
CMAKE_BIN=/e/tools/aqt-venv/Scripts
export PATH="$MINGW_BIN:$CMAKE_BIN:$QT_DIR/bin:$PATH"
cd "$(dirname "$0")"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(cygpath -m "$QT_DIR")" "$@"
cmake --build build
