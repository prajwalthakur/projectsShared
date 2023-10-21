cmake_minimum_required(VERSION 3.0)

project(testC)

# Set the C++ standard to C++17 or your desired version
set(CMAKE_CXX_STANDARD 17)

# Find the SFML package
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Define your source files
#set(SOURCES main.cpp)

# Create an executable from your source files
#add_executable(testC ${SOURCES})


file(GLOB_RECURSE APP_SRC_FILES PathPlanning_Obstacle.cpp)
add_executable (testC ${APP_SRC_FILES}) 
# Link against the SFML libraries
target_link_libraries(testC sfml-graphics sfml-window sfml-system) 
