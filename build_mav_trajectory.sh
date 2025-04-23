#!/bin/bash
# filepath: /home/hariharan/hebi/hebi-cpp-examples/build_mav_trajectory.sh

# Navigate to the root directory
cd "$(dirname "$0")"

# Create build directory if it doesn't exist
mkdir -p mav_trajectory_generation/build

# Navigate to build directory
cd mav_trajectory_generation/build

# Run CMake
cmake .. 

# Build the library
cmake --build . -j$(nproc)

echo "mav_trajectory_generation library built successfully at: $(pwd)/libmav_trajectory_generation.a"