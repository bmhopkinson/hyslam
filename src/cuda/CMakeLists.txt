
HYSLAM_ADD_SOURCES(
  Allocator_gpu.cu
  Cuda.cu
  Fast_gpu.cu
  ORBextractor_cuda.h ORBextractor_cuda.cc
  Orb_gpu.cu
  ORBmatcher_cuda.h ORBmatcher_cuda.cc
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
