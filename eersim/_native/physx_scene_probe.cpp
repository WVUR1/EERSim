// Build with headers from NVIDIA PhysX tag ovphysx-0.5.11.
// Calls public SDK virtual methods; does not own the scene or CUDA context.
// A live PxScene pointer must come from the matching ovphysx runtime.
#include "PxScene.h"
#include "foundation/PxPhysicsVersion.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxRigidDynamic.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationLink.h"
#include "PxSimulationStatistics.h"
#include "task/PxCpuDispatcher.h"

#if defined(_WIN32)
# if !defined(_MSC_VER) || !defined(_M_X64)
#  error "The Windows ovphysx wheel requires x64 Microsoft C++ ABI. Do not use MinGW ABI."
# endif
# define EERSIM_EXPORT extern "C" __declspec(dllexport)
// The helper intentionally links without the CRT. The SDK statistics
// constructor emits zero-fill, and MSVC-compatible floating-point code refers
// to _fltused. Keep these definitions local to this DLL's symbol table.
extern "C" int _fltused = 0;
# if !defined(__clang__)
#  pragma function(memset)
# endif
extern "C" void* memset(void* destination, int value, decltype(sizeof(0)) size)
{
    auto* bytes = static_cast<volatile unsigned char*>(destination);
    for (decltype(size) i = 0; i < size; ++i) bytes[i] = static_cast<unsigned char>(value);
    return destination;
}
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

// Diagnostic ABI 1. Only call after fetchResults/step_sync has completed, with
// the scene quiescent. This does not step, wake, sleep, or otherwise modify it.
// A row describes a regular actor or an entire articulation. All articulation
// links sleep together: reporting each link as an independent body is incorrect.
// Names remain owned by PhysX; copy them before releasing/changing the scene.
struct EersimActivityRow
{
    const char* name;
    unsigned int kind;       // 0 static, 1 regular dynamic, 2 articulation
    unsigned int sleeping;   // meaningful only for dynamic/articulation
    unsigned int linkCount;  // articulation links, otherwise zero
    float wakeCounter;
    double linearSpeedSquared;   // articulation: maximum over all its links
    double angularSpeedSquared;
};

// Stable POD ABI instead of exposing the SDK's internal statistics layout.
struct EersimActivityStatistics
{
    unsigned int cpuWorkerCount;
    unsigned int activeConstraints;
    unsigned int activeDynamicBodies;
    unsigned int activeKinematicBodies;
    unsigned int staticBodies;
    unsigned int dynamicBodies;
    unsigned int kinematicBodies;
    unsigned int articulations;
    unsigned int axisSolverConstraints;
    unsigned int discreteContactPairsTotal;
    unsigned int discreteContactPairsWithCacheHits;
    unsigned int discreteContactPairsWithContacts;
    unsigned int geometryCount;
    unsigned int shapeCounts[11];
    unsigned int discreteContactPairs[11 * 11];
};
static_assert(physx::PxGeometryType::eGEOMETRY_COUNT == 11, "Update diagnostic ABI for new geometry types");
static_assert(sizeof(EersimActivityRow) == 40, "Diagnostic ABI requires x64 native layout");

static double speedSquared(const physx::PxVec3& velocity)
{
    // Double precision avoids rounding tiny nonzero float velocities to zero.
    // Avoid the Windows SDK's _fpclass CRT call; ordered comparisons also
    // reject NaN, and the largest finite float rejects either infinity.
    constexpr float maximum = 3.402823466e+38F;
    if (!(velocity.x <= maximum && velocity.x >= -maximum &&
          velocity.y <= maximum && velocity.y >= -maximum &&
          velocity.z <= maximum && velocity.z >= -maximum)) return -1;
    return double(velocity.x) * velocity.x + double(velocity.y) * velocity.y +
           double(velocity.z) * velocity.z;
}

EERSIM_EXPORT unsigned int eersim_activity_abi_version() { return 1; }
EERSIM_EXPORT unsigned int eersim_activity_row_size() { return sizeof(EersimActivityRow); }
EERSIM_EXPORT unsigned int eersim_activity_statistics_size() { return sizeof(EersimActivityStatistics); }

// Return 0 on success, 1 for a count-only/insufficient-capacity request,
// -1 for invalid arguments, -2 for DirectGPU (CPU getters are unavailable),
// -3 if scene topology changes while being inspected. No partial result is valid.
EERSIM_EXPORT int eersim_scene_activity(void* pointer, EersimActivityRow* rows,
                                      unsigned int capacity, unsigned int* required)
{
    if (!pointer || !required) return -1;
    auto* scene = static_cast<physx::PxScene*>(pointer);
    if (scene->getFlags().isSet(physx::PxSceneFlag::eENABLE_DIRECT_GPU_API)) return -2;
    const auto types = physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC;
    const auto regularCount = scene->getNbActors(types);
    const auto articulationCount = scene->getNbArticulations();
    *required = regularCount + articulationCount;
    if (!rows || capacity < *required) return 1;
    unsigned int written = 0;
    physx::PxActor* actors[16];
    for (unsigned int start = 0; start < regularCount;)
    {
        const auto count = scene->getActors(types, actors, 16, start);
        if (!count || count > regularCount - start) return -3;
        for (unsigned int i = 0; i < count; ++i)
        {
            auto& row = rows[written++];
            row.name = actors[i]->getName();
            row.kind = 0;
            row.sleeping = 0;
            row.linkCount = 0;
            row.wakeCounter = 0;
            row.linearSpeedSquared = 0;
            row.angularSpeedSquared = 0;
            if (actors[i]->getType() == physx::PxActorType::eRIGID_DYNAMIC)
            {
                auto* body = static_cast<physx::PxRigidDynamic*>(actors[i]);
                row.kind = 1;
                row.sleeping = body->isSleeping() ? 1 : 0;
                row.wakeCounter = body->getWakeCounter();
                row.linearSpeedSquared = speedSquared(body->getLinearVelocity());
                row.angularSpeedSquared = speedSquared(body->getAngularVelocity());
                if (row.linearSpeedSquared < 0 || row.angularSpeedSquared < 0) return -4;
            }
        }
        start += count;
    }
    physx::PxArticulationReducedCoordinate* articulations[16];
    physx::PxArticulationLink* links[16];
    for (unsigned int start = 0; start < articulationCount;)
    {
        const auto count = scene->getArticulations(articulations, 16, start);
        if (!count || count > articulationCount - start) return -3;
        for (unsigned int i = 0; i < count; ++i)
        {
            auto* articulation = articulations[i];
            auto& row = rows[written++];
            row.name = articulation->getName();
            row.kind = 2;
            row.sleeping = articulation->isSleeping() ? 1 : 0;
            row.linkCount = articulation->getNbLinks();
            row.wakeCounter = articulation->getWakeCounter();
            row.linearSpeedSquared = 0;
            row.angularSpeedSquared = 0;
            for (unsigned int linkStart = 0; linkStart < row.linkCount;)
            {
                const auto linkCount = articulation->getLinks(links, 16, linkStart);
                if (!linkCount || linkCount > row.linkCount - linkStart) return -3;
                for (unsigned int j = 0; j < linkCount; ++j)
                {
                    const double linear = speedSquared(links[j]->getLinearVelocity());
                    const double angular = speedSquared(links[j]->getAngularVelocity());
                    if (linear < 0 || angular < 0) return -4;
                    if (linear > row.linearSpeedSquared) row.linearSpeedSquared = linear;
                    if (angular > row.angularSpeedSquared) row.angularSpeedSquared = angular;
                }
                linkStart += linkCount;
            }
        }
        start += count;
    }
    if (written != *required || regularCount != scene->getNbActors(types) ||
        articulationCount != scene->getNbArticulations()) return -3;
    return 0;
}

EERSIM_EXPORT int eersim_scene_activity_statistics(void* pointer, EersimActivityStatistics* output)
{
    if (!pointer || !output) return -1;
    auto* scene = static_cast<physx::PxScene*>(pointer);
    if (scene->getFlags().isSet(physx::PxSceneFlag::eENABLE_DIRECT_GPU_API)) return -2;
    physx::PxSimulationStatistics statistics;
    scene->getSimulationStatistics(statistics);
    auto* dispatcher = scene->getCpuDispatcher();
    output->cpuWorkerCount = dispatcher ? dispatcher->getWorkerCount() : 0;
    output->activeConstraints = statistics.nbActiveConstraints;
    output->activeDynamicBodies = statistics.nbActiveDynamicBodies;
    output->activeKinematicBodies = statistics.nbActiveKinematicBodies;
    output->staticBodies = statistics.nbStaticBodies;
    output->dynamicBodies = statistics.nbDynamicBodies;
    output->kinematicBodies = statistics.nbKinematicBodies;
    output->articulations = statistics.nbArticulations;
    output->axisSolverConstraints = statistics.nbAxisSolverConstraints;
    output->discreteContactPairsTotal = statistics.nbDiscreteContactPairsTotal;
    output->discreteContactPairsWithCacheHits = statistics.nbDiscreteContactPairsWithCacheHits;
    output->discreteContactPairsWithContacts = statistics.nbDiscreteContactPairsWithContacts;
    output->geometryCount = physx::PxGeometryType::eGEOMETRY_COUNT;
    for (unsigned int i = 0; i < output->geometryCount; ++i)
    {
        output->shapeCounts[i] = statistics.nbShapes[i];
        for (unsigned int j = 0; j < output->geometryCount; ++j)
            output->discreteContactPairs[i * output->geometryCount + j] = statistics.getRbPairStats(
                physx::PxSimulationStatistics::eDISCRETE_CONTACT_PAIRS,
                static_cast<physx::PxGeometryType::Enum>(i), static_cast<physx::PxGeometryType::Enum>(j));
    }
    return 0;
}
