#!/bin/bash

WORKSPACE=$(dirname "$(realpath "$0")")

echo "[1/4] Cleaning build/install/log..."
rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"

echo "[2/4] Building with symlink install..."
colcon build --symlink-install

if [ $? -ne 0 ]; then
  echo "[!] Build failed."
  exit 1
fi

echo "[3/4] Sourcing workspace..."
source "$WORKSPACE/install/setup.bash"

echo "[4/4] Done! Workspace built and sourced."
