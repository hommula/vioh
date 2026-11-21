# Cross-compile for the APB (Cortex-A53, aarch64) inside the hold-onboard-arm Docker image.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-a53")
set(OpenCV_DIR /opt/opencv-arm/lib/cmake/opencv4)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
