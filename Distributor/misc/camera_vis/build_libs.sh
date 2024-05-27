(
  mkdir -p build_libs/minifb
  cd build_libs/minifb
  cmake ../../libs/minifb -DUSE_METAL_API=ON -DMINIFB_BUILD_EXAMPLES=OFF
  make
)

(
  mkdir -p build_libs/openpnp-capture
  cd build_libs/openpnp-capture
  cmake ../../libs/openpnp-capture
  make
)
