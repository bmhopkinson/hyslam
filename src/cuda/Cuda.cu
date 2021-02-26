#include <helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace HYSLAM { namespace cuda {
  void deviceSynchronize() {
    checkCudaErrors( cudaDeviceSynchronize() );
  }
} }
