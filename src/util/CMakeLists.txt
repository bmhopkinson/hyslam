
HYSLAM_ADD_SOURCES(
  Converter.h Converter.cc
  GenUtils.h GenUtils.cpp
  tinyxml2.h tinyxml2.cpp
        ThreadSafeQueue.h   ThreadSafeQueue.cpp
        ImagingFramePlacer.h    ImagingFramePlacer.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
