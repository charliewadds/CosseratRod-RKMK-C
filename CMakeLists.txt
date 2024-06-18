cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=leak")
project(CosseratRod_RKMK_C C)
cmake_policy(SET CMP0079 NEW)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(ENV{OPENBLAS_NUM_THREADS} 1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG")#remove asserts
link_directories("/opt/homebrew/lib")
include_directories("/opt/homebrew/include")
link_directories("/usr/local/lib")
include_directories("/usr/local/include")
#find_package(GSL REQUIRED)
# Include directories
include_directories(LieGroup)
include_directories(MatrixLib)
include_directories(RobotLib)
include_directories(Tests)
include_directories(gsl)
#include_directories(solvers)


# Source files for libraries
set(LIE_GROUP_SOURCES LieGroup/LieGroup.c)
set(MATRIX_LIB_SOURCES MatrixLib/Matrices.c)
set(ROBOT_LIB_SOURCES RobotLib/RobotLib.c)
set(TEST_SOURCE Tests/testMatrix.c)
#set(SOLVER_SOURCE solvers/solvers.c)

# Find BLAS and LAPACK
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

# Create libraries
add_library(MatrixLib STATIC ${MATRIX_LIB_SOURCES})
add_library(LieGroup STATIC ${LIE_GROUP_SOURCES})
add_library(RobotLib STATIC ${ROBOT_LIB_SOURCES})
add_library(TestLib STATIC ${TEST_SOURCE})
#add_library(Solvers STATIC ${SOLVER_SOURCE})

#add_subdirectory(Solvers/nlopt)
add_subdirectory(levmar-2.6)
# Main executable
message(STATUS "BLAS_LIBRARIES: ${BLAS_LIBRARIES}")

add_executable(InvDyn RunMeInvDyn.c)

add_executable(FwdDyn RunMeFwdDyn.c)
# Link libraries to the executable
# Link GSL libraries with MatrixLib
#target_link_libraries(levmar PRIVATE levmar-2.6/build/liblevmar.a)
target_link_libraries(MatrixLib PRIVATE -lm -lgsl -lgslcblas)
target_link_libraries(LieGroup PRIVATE MatrixLib gsl)
#target_link_libraries(Solvers PRIVATE)
target_link_libraries(RobotLib PRIVATE -lgsl -lnlopt levmar ${BLAS_LIBRARIES} MatrixLib LieGroup)




target_link_libraries(FwdDyn -lm -lgsl  MatrixLib LieGroup RobotLib)
target_link_libraries(InvDyn -lm -lgsl  MatrixLib LieGroup RobotLib)


