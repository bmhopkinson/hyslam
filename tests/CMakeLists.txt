cmake_minimum_required(VERSION 3.8)
project(ORB_SLAM2_TESTS)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS_DEBUG "-O2 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g")

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

LIST(APPEND CMAKE_C_FLAGS "-Wall -Wextra -DUSE_NVTX -Wno-deprecated-declarations")
LIST(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -DUSE_NVTX -Wno-deprecated-declarations")

find_package(CUDA QUIET REQUIRED)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
SET(CUDA_HOST_COMPILER /usr/bin/g++)
LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

message(STATUS "CMAKE PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}   ")

set(CMAKE_SKIP_BUILD_RPATH FALSE)  #otherwise rpath is not properly set to run from build directory - expects install to default location

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake_modules)
SET( G2O_ROOT /usr/local )
message(STATUS "G2O_ROOT ${G2O_ROOT}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

find_package(OpenCV 3.1.0 REQUIRED PATHS /home/cv-bhlab/Documents/Libraries/opencv/opencv_3_4/install/share/OpenCV NO_DEFAULT_PATH)
find_package(G2O REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ORB_SLAM2plus/cmake_modules)
set(ORB_SLAM2_BASE_PATH ..)
find_library(ORB_SLAM2_LIBRARIES NAMES "ORB_SLAM2" PATHS ${ORB_SLAM2_BASE_PATH}/build/lib NO_DEFAULT_PATH)
set(ORB_SLAM2_INCLUDE_DIRS ${ORB_SLAM2_BASE_PATH} ${ORB_SLAM2_BASE_PATH}/include)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/../include
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/../../core_types
${ORB_SLAM2_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${G2O_INCLUDE_DIR}
)
message(STATUS "G2O_FOUND: ${G2O_FOUND}")
message(STATUS "G2O include: ${G2O_INCLUDE_DIR} " )
message(STATUS "G2O libs: ${G2O_LIBS} " )
message(STATUS "CV libs: ${OpenCV_LIBS} " )
message(STATUS "ORB_SLAM2 lib: ${ORB_SLAM2_LIBRARIES}")
message(STATUS "G2O slam3 lib: ${G2O_TYPES_SLAM3D}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

cuda_include_directories(
  ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
)

#target_link_libraries(${PROJECT_NAME}
#${OpenCV_LIBS}
#${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
#${G2O_LIBS}
#${G2O_TYPES_SLAM3D}
#${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#nvToolsExt
#${PROJECT_SOURCE_DIR}/../lib/libdataproc.so
#)

add_executable(test_tracking_datastructs ../test_tracking_datastructs.cpp)

target_link_libraries(test_tracking_datastructs
  ${ORB_SLAM2_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
)

add_executable(test_trajectory ../test_trajectory.cpp)

target_link_libraries(test_trajectory 
  ${ORB_SLAM2_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
#  ${Pangolin_LIBRARIES}
)

add_executable(test_new_g2o_components ../test_new_g2o_components.cpp)

target_link_libraries(test_new_g2o_components
  ${ORB_SLAM2_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
#  ${Pangolin_LIBRARIES}
)
 
