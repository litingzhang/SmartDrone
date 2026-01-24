SYSROOT="$(cd ../sysroots/cm5 && pwd)"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v "$SYSROOT/usr/bin" | paste -sd ':' -)"

rm -rf build

cmake -S . -B build \
  -DSYSROOT="$SYSROOT" \
  -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config

cmake --build build -j16
