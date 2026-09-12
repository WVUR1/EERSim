// Build with headers from NVIDIA PhysX tag ovphysx-0.5.11.
// Calls public SDK virtual methods; does not own the scene or CUDA context.
// A live PxScene pointer must come from the matching ovphysx runtime.
#include "PxScene.h"
#include "foundation/PxPhysicsVersion.h"
#include "cudamanager/PxCudaContextManager.h"

#if defined(_WIN32)
# if !defined(_MSC_VER) || !defined(_M_X64)
#  error "The Windows ovphysx wheel requires x64 Microsoft C++ ABI. Do not use MinGW ABI."
# endif
# define EERSIM_EXPORT extern "C" __declspec(dllexport)
#else
# define EERSIM_EXPORT extern "C" __attribute__((visibility("default")))
#endif

// Bit 0: GPU dynamics; bit 1: GPU broad phase; bit 2: Direct GPU API;
// bit 3: a valid CUDA context. Require bits 0, 1, and 3 for GPU physics.
// Direct GPU API is optional: host readback mode intentionally omits bit 2.
EERSIM_EXPORT unsigned int eersim_scene_gpu_flags(void* pointer)
{
    if (!pointer) return 0;
    auto* scene = static_cast<physx::PxScene*>(pointer);
    auto* cuda = scene->getCudaContextManager();
    unsigned int result = 0;
    if (scene->getFlags().isSet(physx::PxSceneFlag::eENABLE_GPU_DYNAMICS)) result |= 1;
    if (scene->getBroadPhaseType() == physx::PxBroadPhaseType::eGPU) result |= 2;
    if (scene->getFlags().isSet(physx::PxSceneFlag::eENABLE_DIRECT_GPU_API)) result |= 4;
    if (cuda && cuda->contextIsValid()) result |= 8;
    return result;
}

EERSIM_EXPORT int eersim_scene_cuda_device(void* pointer)
{
    if (!pointer) return -1;
    auto* cuda = static_cast<physx::PxScene*>(pointer)->getCudaContextManager();
    return cuda && cuda->contextIsValid() ? cuda->getDevice() : -1;
}

// Returned text is owned by the CUDA context and remains valid while it lives.
EERSIM_EXPORT const char* eersim_scene_cuda_name(void* pointer)
{
    if (!pointer) return "";
    auto* cuda = static_cast<physx::PxScene*>(pointer)->getCudaContextManager();
    return cuda && cuda->contextIsValid() ? cuda->getDeviceName() : "";
}

EERSIM_EXPORT unsigned int eersim_probe_physx_version()
{
    return PX_PHYSICS_VERSION;
}
