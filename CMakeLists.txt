cmake_minimum_required(VERSION 3.16)
project(vioh CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d)

# Development tools: run on a laptop against a recording.
add_executable(calib src/calib.cpp)
target_link_libraries(calib ${OpenCV_LIBS})
add_executable(replay src/replay.cpp)
target_link_libraries(replay ${OpenCV_LIBS})
add_executable(selftest src/selftest.cpp)
target_link_libraries(selftest ${OpenCV_LIBS})
add_executable(simtest src/simtest.cpp)
target_link_libraries(simtest ${OpenCV_LIBS})

# The onboard program: Linux only (V4L2, IIO, termios).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(Threads REQUIRED)
  add_executable(vioh src/main.cpp)
  target_link_libraries(vioh ${OpenCV_LIBS} Threads::Threads -static-libstdc++ -static-libgcc)
endif()
