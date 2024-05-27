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

gcc -O2 -DSTB_IMAGE_RESIZE_IMPLEMENTATION -c -x c stb_image_resize2.h -o build_libs/stb_image_resize2.o
