cmake_minimum_required(VERSION 2.8.3)
project(k2_h264_encoder)

find_package(PkgConfig)
pkg_check_modules(ffmpeg REQUIRED libavcodec libavformat libavutil libswscale)

find_package(Boost REQUIRED system thread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wno-unused-parameter")

include_directories(${ffmpeg_INCLUDE_DIRS})
add_executable(k2_h264_encoder ${PROJECT_SOURCE_DIR}/k2_h264_encoder.cpp)
target_link_libraries(k2_h264_encoder ${ffmpeg_LIBRARIES} ${Boost_LIBRARIES})
