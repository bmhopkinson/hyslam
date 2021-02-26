
HYSLAM_ADD_SOURCES(
  ORBextractor.h ORBextractor.cc
        MatchCriteria.h MatchCriteria.cpp
  ORBmatcher.h ORBmatcher.cc
  ORBstereomatcher.h ORBstereomatcher.cpp
  ORBUtil.h  ORBUtil.cpp
        FeatureFinder.h FeatureFinder.cpp
        ORBFinder.h ORBFinder.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
