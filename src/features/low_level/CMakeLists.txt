
HYSLAM_ADD_SOURCES(
        FeatureFinder.h FeatureFinder.cpp
        SURFFinder.h    SURFFinder.cpp
        ORBFinder.h ORBFinder.cpp
        DescriptorDistance.h    DescriptorDistance.cpp
        FeatureDescriptor.h     FeatureDescriptor.cpp
        )
        
        
set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
