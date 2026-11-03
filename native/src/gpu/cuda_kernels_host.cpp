// native/src/gpu/cuda_kernels_host.cpp — the single host C++ compile of
// cuda_kernels.cu (bike_gpu_kernels). Without __CUDACC__ the kernels' host
// loops and the extern "C" entry points (cuda_kernels_abi.hpp) build as plain
// C++. Every host consumer — the CPU batch runner, the GPU tests — links this
// library instead of including the .cu: the ODR sweep links every selected TU
// into one image, so a second textual compile would define them twice.
// NOLINTNEXTLINE(bugprone-suspicious-include) the .cu stays nvcc's direct input; this TU is its host compile.
#include "cuda_kernels.cu"
