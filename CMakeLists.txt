cmake_minimum_required(VERSION 3.8)
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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET( G2O_ROOT /usr/local )
message(STATUS "G2O_ROOT ${G2O_ROOT}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

#find_package(OpenCV 3.1.0 REQUIRED PATHS /home/nvidia/Documents/opencv_build/share/OpenCV NO_DEFAULT_PATH)
#find_package(OpenCV 3.1.0 REQUIRED PATHS /home/cv-bhlab/Documents/Libraries/opencv/opencv_3_4/install/share/OpenCV NO_DEFAULT_PATH)
find_package(OpenCV 3.1.0 REQUIRED PATHS /home/brian/Documents/Libraries/opencv/3_4_13/install/share/OpenCV NO_DEFAULT_PATH)
find_package(G2O REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)

#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

#add_library(dataproc SHARED IMPORTED)
#set_target_properties(dataproc PROPERTIES
#    IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/../lib/libdataproc.so"
#    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/../include/core_types"
#)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${G2O_INCLUDE_DIR}
)
#message(STATUS "G2O_FOUND: ${G2O_FOUND}")
#message(STATUS "G2O include: ${G2O_INCLUDE_DIR} " )
#message(STATUS "G2O libs: ${G2O_LIBS} " )
#message(STATUS "CV libs: ${OpenCV_LIBS} " )
#message(STATUS "G2O slam3 lib: ${G2O_TYPES_SLAM3D}")

add_subdirectory(src) #HYSLAM IS ADDED HERE

# Install header files.
install(DIRECTORY src/
        DESTINATION ${PROJECT_SOURCE_DIR}/include
        FILES_MATCHING PATTERN "*.h")
 
unset(CMAKE_RUNTIME_OUTPUT_DIRECTORY)

# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})
