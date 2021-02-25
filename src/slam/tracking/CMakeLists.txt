HYSLAM_ADD_SOURCES(  
  Tracking_datastructs.h Tracking_datastructs.cc
  TrackingStrategy.h  TrackingStrategy.cpp
  TrackReferenceKeyFrame.h    TrackReferenceKeyFrame.cpp
  TrackMotionModel.h  TrackMotionModel.cpp
  TrackLocalMap.h TrackLocalMap.cpp
  TrackPlaceRecognition.h TrackPlaceRecognition.cpp
        TrackingState.h TrackingState.cpp
        TrackingStateNormal.h   TrackingStateNormal.cpp
        TrackingStateRelocalize.h   TrackingStateRelocalize.cpp
        TrackingStateInitialize.h   TrackingStateInitialize.cpp
)


set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
