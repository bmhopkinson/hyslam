
HYSLAM_ADD_SOURCES(
  BundleAdjustment.h BundleAdjustment.cc
  GlobalBundleAdjustment.h GlobalBundleAdjustment.cc
  ImagingBundleAdjustment.h ImagingBundleAdjustment.cc
  LocalBundleAdjustment.h LocalBundleAdjustment.cc
  OptHelpers.h OptHelpers.cc
  Optimizer.h Optimizer.cc
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
