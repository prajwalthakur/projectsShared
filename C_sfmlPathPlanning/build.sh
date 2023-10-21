#!/bin/bash

# Create a build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi

# Navigate to the build directory
cd build

# Generate build files using CMake
cmake ..

# Build the project using the chosen build tool (e.g., make)
make

# Run the compiled program (adjust the executable name as needed)
./testC
