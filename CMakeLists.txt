cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=leak")
project(CosseratRod_RKMK_C C)
cmake_policy(SET CMP0079 NEW)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(ENV{OPENBLAS_NUM_THREADS} 1)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG")#remove asserts
link_directories("/opt/homebrew/lib")
include_directories("/opt/homebrew/include")
#find_package(GSL REQUIRED)
# Include directories
include_directories(LieGroup)
include_directories(MatrixLib)
include_directories(RobotLib)
include_directories(FDM)
include_directories(Tests)
include_directories(gsl)



# Source files for libraries
set(LIE_GROUP_SOURCES LieGroup/LieGroup.c)
set(MATRIX_LIB_SOURCES MatrixLib/Matrices.c)
set(ROBOT_LIB_SOURCES RobotLib/RobotLib.c)
set(TEST_SOURCE Tests/testMatrix.c)
set(FDM_SOURCE FDM/Discretization.c FDM/FDM.c
        RunMeInvDyn.c)

# Find BLAS and LAPACK
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

# Create libraries
add_library(MatrixLib SHARED ${MATRIX_LIB_SOURCES})
add_library(LieGroup SHARED ${LIE_GROUP_SOURCES})
add_library(RobotLib SHARED ${ROBOT_LIB_SOURCES})
add_library(TestLib STATIC ${TEST_SOURCE})
add_library(FDM STATIC ${FDM_SOURCE})


add_subdirectory(levmar-2.6)
# Main executable
message(STATUS "BLAS_LIBRARIES: ${BLAS_LIBRARIES}")
add_executable(runTests runTests.c)
add_executable(InvDyn RunMeInvDyn.c)
add_executable(playground playground.c)

# Link libraries to the executable
# Link GSL libraries with MatrixLib
#target_link_libraries(levmar PRIVATE levmar-2.6/build/liblevmar.a)
target_link_libraries(MatrixLib PRIVATE -lm -lgsl -lgslcblas)
target_link_libraries(LieGroup PRIVATE MatrixLib gsl)

target_link_libraries(RobotLib PRIVATE -lgsl levmar ${BLAS_LIBRARIES} MatrixLib LieGroup)

install(TARGETS MatrixLib LieGroup RobotLib TestLib FDM
        LIBRARY DESTINATION /Users/charliewadds/CLionProjects/CosseratRod-RKMK-C/sharedLibs
)



target_link_libraries(InvDyn -lm -lgsl  MatrixLib LieGroup RobotLib)

target_link_libraries(runTests TestLib MatrixLib LieGroup RobotLib FDM -lm)
target_link_libraries(playground MatrixLib LieGroup RobotLib FDM -lm gsl -lm)
