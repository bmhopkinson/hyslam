add_subdirectory(low_level)

HYSLAM_ADD_SOURCES(

    MatchCriteria.h MatchCriteria.cpp
    FeatureMatcher.h FeatureMatcher.cc
    Stereomatcher.h Stereomatcher.cpp
        FeatureUtil.h FeatureUtil.cpp
    FeatureFactory.h    FeatureFactory.cpp
    ORBFactory.h    ORBFactory.cpp
    SURFFactory.h   SURFFactory.cpp
        FeatureExtractor.h  FeatureExtractor.cpp
    SURFExtractor.h SURFExtractor.cpp
    ORBExtractor.h ORBExtractor.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
