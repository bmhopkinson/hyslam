add_subdirectory(low_level)

HYSLAM_ADD_SOURCES(
    FeatureExtractor.h FeatureExtractor.cc
    MatchCriteria.h MatchCriteria.cpp
    FeatureMatcher.h FeatureMatcher.cc
    Stereomatcher.h Stereomatcher.cpp
    ORBUtil.h  ORBUtil.cpp
    FeatureFactory.h    FeatureFactory.cpp
    ORBFactory.h    ORBFactory.cpp
    SURFFactory.h   SURFFactory.cpp
    SURFExtractor.h SURFExtractor.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
