# Install script for directory: /home/hopkinsonlab/Documents/hyslam/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so"
         RPATH "/home/hopkinsonlab/Documents/Libraries/OpenCV/opencv_3_4/install/lib:/usr/local/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libHYSLAM.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/hopkinsonlab/Documents/hyslam/lib/libHYSLAM.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so"
         OLD_RPATH "/home/hopkinsonlab/Documents/Libraries/OpenCV/opencv_3_4/install/lib:/usr/local/lib:"
         NEW_RPATH "/home/hopkinsonlab/Documents/Libraries/OpenCV/opencv_3_4/install/lib:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libHYSLAM.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/HYSLAM/.")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/HYSLAM" TYPE DIRECTORY FILES "/home/hopkinsonlab/Documents/hyslam/src/." FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/core/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/estimators/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/features/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/initializers/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/main/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/optimizers/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/slam/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/util/cmake_install.cmake")
  include("/home/hopkinsonlab/Documents/hyslam/cmake-build-relwithdebinfo/src/viz/cmake_install.cmake")

endif()

