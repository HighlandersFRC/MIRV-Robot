
# Project's name
PROJECT ( drive C CXX )

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(drive ${PROJECT_SOURCE_DIR}/drive.cpp)
target_link_libraries(drive ${catkin_LIBRARIES})


if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/MIRV-Robot/mirv_real/drive/lib/raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/MIRV-Robot/mirv_real/drive/lib/jetsontx)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/MIRV-Robot/mirv_real/drive/lib/x86-64)
endif()

target_link_libraries(drive ${catkin_LIBRARIES} ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(drive ${catkin_LIBRARIES} ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)

target_link_libraries(drive Threads::Threads)
target_link_libraries(drive ${SDL2_LIBRARIES})

add_custom_command(TARGET drive POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PHOENIX_LIBS_DIR}
        $<TARGET_FILE_DIR:drive>/..)
