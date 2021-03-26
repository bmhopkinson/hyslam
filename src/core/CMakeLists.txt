
HYSLAM_ADD_SOURCES(
  Camera.h Camera.cpp
  CovisibilityGraph.h CovisibilityGraph.cpp
  Frame.h Frame.cc
  KeyFrame.h KeyFrame.cc
  KeyFrameDB.h KeyFrameDB.cc
  LandMarkMatches.h LandMarkMatches.cpp
  Map.h Map.cc
  MapPoint.h MapPoint.cc
  MapPointDB.h MapPointDB.cpp
  MapPointFactory.h MapPointFactory.cpp
        FeatureViews.h FeatureViews.cpp
        FeatureExtractorSettings.h  FeatureExtractorSettings.cpp
        FeatureExtractorSettings.h    FeatureExtractorSettings.cpp
  PlaceRecognizer.h PlaceRecognizer.cpp
  SensorData.h SensorData.cpp
  SpanningTree.h SpanningTree.cpp
  Trajectory.h Trajectory.cc
)


set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
