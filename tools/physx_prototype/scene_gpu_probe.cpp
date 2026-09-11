// Compile against PhysX headers from the ovphysx-0.5.11 release tag.
// Uses public virtual SDK methods; no PhysX or CUDA link libraries are needed.
#include "PxScene.h"
#include "cudamanager/PxCudaContextManager.h"

extern "C" unsigned int eersim_scene_gpu_flags(void* pointer)
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

extern "C" int eersim_scene_cuda_device(void* pointer)
{
    if (!pointer) return -1;
    auto* cuda = static_cast<physx::PxScene*>(pointer)->getCudaContextManager();
    return cuda && cuda->contextIsValid() ? cuda->getDevice() : -1;
}

extern "C" const char* eersim_scene_cuda_name(void* pointer)
{
    if (!pointer) return "";
    auto* cuda = static_cast<physx::PxScene*>(pointer)->getCudaContextManager();
    return cuda && cuda->contextIsValid() ? cuda->getDeviceName() : "";
}
