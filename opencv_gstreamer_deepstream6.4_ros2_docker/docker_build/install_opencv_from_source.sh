# 1. Create and navigate to build directory
cd /opt/build

# 2. Clean any previous attempts
rm -rf opencv opencv_contrib

# 3. Clone the repositories
git clone --branch 4.9.0 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 4.9.0 --depth 1 https://github.com/opencv/opencv_contrib.git
mkdir -p /opt/build/opencv/build
cd /opt/build/opencv/build && \
# Apply patch first
sed -i 's/if (weight != 1.0)/if (static_cast<float>(weight) != 1.0f)/' ../modules/dnn/src/layers/../cuda4dnn/primitives/normalize_bbox.hpp && \
# Then run CMake with these additional flags
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/opt/build/opencv_contrib/modules \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=OFF \
    -D CUDA_ARCH_BIN="7.5,8.0,8.6,8.9,9.0" \
    -D WITH_GSTREAMER=ON \
    -D WITH_LIBV4L=ON \
    -D WITH_V4L=ON \
    -D WITH_TBB=ON \
    -D WITH_FFMPEG=ON \
    -D BUILD_opencv_hdf=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE=$(which python3) \
    ../../opencv && \
make -j$(nproc) && \
make install && \
ldconfig
