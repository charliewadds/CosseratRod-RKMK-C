cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
project(CosseratRod_RKMK_C C)
# Set optimization flags for C compiler
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# Set architectures for macOS
#if(APPLE)
#set(CMAKE_OSX_ARCHITECTURES "arm64") # Add any other architectures you need, separated by semicolons
#set(CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE "arm64")
#endif()
set(CMAKE_PREFIX_PATH "/opt/homebrew")
set(CMAKE_PREFIX_PATH "/usr/local/bin")
link_directories("/opt/homebrew/lib")
include_directories("/opt/homebrew/include")
#find_package(GSL REQUIRED)
# Include directories
include_directories(LieGroup)
include_directories(MatrixLib)
include_directories(RobotLib)
include_directories(FDM)
include_directories(Tests)

#link_directories("/opt/homebrew/lib")
#include_directories("/opt/homebrew/include")

# Source files for libraries
set(LIE_GROUP_SOURCES LieGroup/LieGroup.c)
set(MATRIX_LIB_SOURCES MatrixLib/Matrices.c)
set(ROBOT_LIB_SOURCES RobotLib/RobotLib.c)
set(TEST_SOURCE Tests/testMatrix.c)
set(FDM_SOURCE FDM/Discretization.c FDM/FDM.c
        RunMeInvDyn.c)

# Create libraries
add_library(LieGroup STATIC ${LIE_GROUP_SOURCES})
add_library(MatrixLib STATIC ${MATRIX_LIB_SOURCES})
add_library(RobotLib STATIC ${ROBOT_LIB_SOURCES})
add_library(TestLib STATIC ${TEST_SOURCE})
add_library(FDM STATIC ${FDM_SOURCE})
# Main executable
add_executable(CosseratRod_RKMK_C testing.c)
add_executable(runTests runTests.c)
add_executable(rigidKin RunMeRigidKinCW.c)
add_executable(InvDyn RunMeInvDyn.c)
add_executable(playground playground.c)
# Link libraries to the executable
target_link_libraries(CosseratRod_RKMK_C MatrixLib LieGroup RobotLib FDM -lm)
target_link_libraries(InvDyn MatrixLib LieGroup RobotLib FDM -lm -lgsl -lgslcblas -lm)
target_link_libraries(runTests TestLib MatrixLib LieGroup RobotLib FDM -lm)
target_link_libraries(rigidKin TestLib MatrixLib LieGroup RobotLib FDM -lm)
#target_link_libraries(playground -lm)
