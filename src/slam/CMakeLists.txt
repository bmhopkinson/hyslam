add_subdirectory(tracking)
add_subdirectory(mapping)

HYSLAM_ADD_SOURCES(
 # LocalMapping.h LocalMapping.cc
       Mapping.h  Mapping.cpp
  LoopClosing.h LoopClosing.cc
  ORBSLAM_datastructs.h ORBSLAM_datastructs.cc
  System.h System.cc
  Tracking.h Tracking.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
