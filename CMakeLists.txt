# Compile your code with: (see README for additional information)
# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../ ..
# make # Compile the gridbots library
# make gridbots_example # OPTIONAL: Compile the executable example code
# make install # Install the library (and example, if you compiled it)

cmake_minimum_required (VERSION 3.9)

project(kilosim_gridbots LANGUAGES CXX C)

# Identify the location of the kilosim library
add_subdirectory(submodules/kilosim)

# Directory containing header files
include_directories(api)

# ------------------------------------------------------------------------------

# gridbots LIBRARY
# Use this to be able to use the gridbots robot class in other projects

# Be sure to list all source files
add_library(kilosim_gridbots INTERFACE)

target_include_directories(kilosim_gridbots
INTERFACE
  api
  submodules
)
target_compile_options(kilosim_gridbots
INTERFACE
  -g
  -march=native
  -ffast-math
  -Wall
  -Wextra
)
target_link_libraries(kilosim_gridbots INTERFACE kilosim)

target_compile_features(kilosim_gridbots INTERFACE cxx_std_11)

install(TARGETS kilosim_gridbots ARCHIVE DESTINATION ${CMAKE_SOURCE_DIR}/lib)


# ------------------------------------------------------------------------------

# EXECUTABLE DEMO

add_executable(gridbots_example EXCLUDE_FROM_ALL
  # src/utils.cpp
  src/DemoGridbot.cpp
  src/main.cpp
)
target_link_libraries(gridbots_example PUBLIC kilosim)
target_compile_options(gridbots_example PRIVATE -g -march=native -Wall -Wextra)
target_compile_features(gridbots_example PRIVATE cxx_std_11)
install(TARGETS gridbots_example RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin OPTIONAL)