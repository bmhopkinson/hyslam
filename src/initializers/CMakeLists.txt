
HYSLAM_ADD_SOURCES(
  Initializer.h  Initializer.cpp
  MonoInitializer.h MonoInitializer.cpp
  StereoInitializer.h StereoInitializer.cpp
  MonoEstimator.h MonoEstimator.cpp
  InitializerDataStructs.h	InitializerDataStructs.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
