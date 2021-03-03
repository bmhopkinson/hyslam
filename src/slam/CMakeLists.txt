add_subdirectory(tracking)
add_subdirectory(mapping)

HYSLAM_ADD_SOURCES(
        dummy.h dummy.cpp #need this for some reason to get tracking and mapping directories added to build
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
