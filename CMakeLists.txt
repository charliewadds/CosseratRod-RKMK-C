cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=leak")
project(CosseratRod_RKMK_C C)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_PREFIX_PATH "/usr/local/bin")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG")#remove asserts
link_directories("/opt/homebrew/lib")
include_directories("/opt/homebrew/include")
#find_package(GSL REQUIRED)
# Include directories
include_directories(LieGroup)
include_directories(MatrixLib)
include_directories(RobotLib)
include_directories(FDM)
include_directories(Tests)



# Source files for libraries
set(LIE_GROUP_SOURCES LieGroup/LieGroup.c)
set(MATRIX_LIB_SOURCES MatrixLib/Matrices.c)
set(ROBOT_LIB_SOURCES RobotLib/RobotLib.c)
set(TEST_SOURCE Tests/testMatrix.c)
set(FDM_SOURCE FDM/Discretization.c FDM/FDM.c
        RunMeInvDyn.c)

# Create libraries
add_library(MatrixLib STATIC ${MATRIX_LIB_SOURCES})
add_library(LieGroup STATIC ${LIE_GROUP_SOURCES})
add_library(RobotLib STATIC ${ROBOT_LIB_SOURCES})
add_library(TestLib STATIC ${TEST_SOURCE})
add_library(FDM STATIC ${FDM_SOURCE})
# Main executable

add_executable(runTests runTests.c)
add_executable(InvDyn RunMeInvDyn.c)
add_executable(playground playground.c)
# Link libraries to the executable
# Link GSL libraries with MatrixLib
target_link_libraries(MatrixLib PRIVATE -lm -lgsl -lgslcblas)
target_link_libraries(LieGroup PRIVATE MatrixLib)
target_link_libraries(RobotLib PRIVATE -lgsl -lgslcblas MatrixLib LieGroup)

install(TARGETS MatrixLib LieGroup RobotLib TestLib FDM
        LIBRARY DESTINATION /Users/charliewadds/CLionProjects/CosseratRod-RKMK-C/sharedLibs
)



target_link_libraries(InvDyn -lm -lgsl -lgslcblas MatrixLib LieGroup RobotLib)

target_link_libraries(runTests TestLib MatrixLib LieGroup RobotLib FDM -lm)
target_link_libraries(playground MatrixLib LieGroup RobotLib FDM -lm -lgsl -lgslcblas -lm)
