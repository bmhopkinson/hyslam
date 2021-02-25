HYSLAM_ADD_SOURCES(
  MapJob.h	MapJob.cpp
        MappingDataStructs.h    MappingDataStructs.cpp
  ProcessNewKeyFrame.h	ProcessNewKeyFrame.cpp
        KeyFrameCuller.h    KeyFrameCuller.cpp
        LandMarkCuller.h    LandMarkCuller.cpp
        LandMarkTriangulator.h  LandMarkTriangulator.cpp
        LandMarkFuser.h LandMarkFuser.cpp
        LocalBundleAdjustmentJob.h  LocalBundleAdjustmentJob.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
