
HYSLAM_ADD_SOURCES(
  FrameDrawer.h FrameDrawer.cc
  MapDrawer.h MapDrawer.cc
  Viewer.h Viewer.cc
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
