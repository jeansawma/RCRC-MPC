#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
tools_dir="${RCRC_TOOLS_DIR:-$project_dir/.tools}"
version=14.2.1-1.1
name="xpack-arm-none-eabi-gcc-$version-linux-x64.tar.gz"
sha=ed8c7d207a85d00da22b90cf80ab3b0b2c7600509afadf6b7149644e9d4790a6
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo 'Use STM32CubeIDE bundled Arm GNU tools or install Arm GNU GCC for this OS.' >&2
  exit 1
fi
mkdir -p "$tools_dir"
if [[ ! -x "$tools_dir/xpack-arm-none-eabi-gcc-$version/bin/arm-none-eabi-gcc" ]]; then
  curl --fail --location --show-error --output "$tools_dir/$name" \
    "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v$version/$name"
  printf '%s  %s\n' "$sha" "$tools_dir/$name" | sha256sum --check --status
  tar -xzf "$tools_dir/$name" -C "$tools_dir"
fi
"$tools_dir/xpack-arm-none-eabi-gcc-$version/bin/arm-none-eabi-gcc" --version
echo "Build: make CROSS_COMPILE=$tools_dir/xpack-arm-none-eabi-gcc-$version/bin/arm-none-eabi-"
