add_subdirectory(low_level)

HYSLAM_ADD_SOURCES(
        FeatureExtractor.h FeatureExtractor.cc
        MatchCriteria.h MatchCriteria.cpp
        FeatureMatcher.h FeatureMatcher.cc
  ORBstereomatcher.h ORBstereomatcher.cpp
  ORBUtil.h  ORBUtil.cpp
        FeatureFactory.h    FeatureFactory.cpp
        ORBFactory.h    ORBFactory.cpp
        SURFFactory.h   SURFFactory.cpp

)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
