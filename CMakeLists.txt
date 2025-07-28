cmake_minimum_required(VERSION 3.10)

project(GMDM VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


set(EXECUTABLE_NAME "gmdm")

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(${EXECUTABLE_NAME} main_figure6.cpp ${SOURCE_FILES})

target_include_directories(${EXECUTABLE_NAME} PUBLIC include)


# For Release builds (optimized for speed)
target_compile_options(${EXECUTABLE_NAME} PRIVATE
    $<$<CONFIG:Release>:-O3> # -O3 is a high level of optimization for GCC/Clang
)

# For Debug builds (includes debugging symbols, no optimization)
target_compile_options(${EXECUTABLE_NAME} PRIVATE
    $<$<CONFIG:Debug>:-g> # -g adds debug information for GDB/LLDB
)

# --- 7. Output Message ---
# A helpful message to show that CMake has finished configuring.
message(STATUS "CMake configuration complete. You can now build the project.")
message(STATUS "To build a release version: cmake --build . --config Release")
message(STATUS "To build a debug version:   cmake --build . --config Debug")
