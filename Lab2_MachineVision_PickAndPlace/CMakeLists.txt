
cmake_minimum_required(VERSION 3.10)
project(Lab2_MachineVision_PickAndPlace)

set(CMAKE_CXX_STANDARD 14)

# Set path to your Dynamixel SDK
set(DXL_SDK_PATH /home/labmachinevision/lab2/DynamixelSDK/c++)

# OpenCV
find_package(OpenCV REQUIRED)

# Include paths
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${DXL_SDK_PATH}/include
)

# Source files
set(SOURCES
    src/pickAndPlace.cpp
    src/Utils.cpp
    src/DynamixelHandler.cpp
    src/RobotControl.cpp
    src/ReadWriteFunctions.cpp
    src/Kinematics.cpp
    src/ImageProcessing.cpp
)

# Add the executable
add_executable(Lab2_MachineVision_PickAndPlace ${SOURCES})

# Link the Dynamixel SDK static or shared library
target_link_directories(Lab2_MachineVision_PickAndPlace PRIVATE ${DXL_SDK_PATH}/build)

target_link_libraries(Lab2_MachineVision_PickAndPlace
    ${OpenCV_LIBS}
    dxl_x64_cxx  # Or dxl_x86_cxx, depending on your system build
)
