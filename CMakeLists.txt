cmake_minimum_required(VERSION 3.10)
project(HYSLAM)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/CMakeHelper.cmake NO_POLICY_SCOPE)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)  #debug is very slow
ENDIF()

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
LIST(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated-declarations")

set(CMAKE_CXX_STANDARD          14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

message(STATUS "CMAKE PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}   ")

set(CMAKE_SKIP_BUILD_RPATH FALSE)  #otherwise rpath is not properly set to run from build directory - expects install to default location
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #will include non-standard opencv location in installed rpath of libHYSLAM.so

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET( G2O_ROOT /usr/local )
message(STATUS "G2O_ROOT ${G2O_ROOT}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

#find_package(OpenCV 3.1.0 REQUIRED PATHS /home/nvidia/Documents/opencv_build/share/OpenCV NO_DEFAULT_PATH)
find_package(OpenCV 3.1.0 REQUIRED PATHS /home/cv-bhlab/Documents/Libraries/opencv/opencv_3_4_nonfree/install/share/OpenCV NO_DEFAULT_PATH)
#find_package(OpenCV 3.1.0 REQUIRED PATHS /home/brian/Documents/Libraries/opencv/3_4_13/install/share/OpenCV NO_DEFAULT_PATH)
#find_package(OpenCV 3.1.0 REQUIRED PATHS /home/hopkinsonlab/Documents/Libraries/OpenCV/opencv_3_4/install/share/OpenCV NO_DEFAULT_PATH)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${G2O_INCLUDE_DIR}
)


add_subdirectory(src) #HYSLAM IS ADDED HERE

# Install 
set(INSTALL_LIBDIR "/usr/local/lib")
set(INSTALL_INCLUDEDIR "/usr/local/include")

 
unset(CMAKE_RUNTIME_OUTPUT_DIRECTORY)

