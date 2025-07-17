#!/usr/bin/env bash
# build_robot.sh  – úsalo con:  source build_robot.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "✖️  Ejecuta este script con:  source build_robot.sh"
  exit 1
fi

WORKSPACE=$(dirname "$(realpath "${BASH_SOURCE[0]}")")

echo "[1/3] Cleaning up old build…"
rm -rf $WORKSPACE/build/ $WORKSPACE/log/ $WORKSPACE/install/
echo "[2/3] Build workspace…"
colcon build --symlink-install
echo "[3/3] Source overlay…"
source "$WORKSPACE/install/setup.bash"
echo "✔️  Overlay listo en esta terminal."
