// Build using public headers from NVIDIA PhysX tag ovphysx-0.5.11.
// Read-only inspection: no PhysX/CUDA libraries need to be linked.
#include "PxScene.h"
#include "PxRigidDynamic.h"
#include "PxRigidStatic.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationLink.h"
#include "PxShape.h"
#include "PxSimulationStatistics.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxTriangleMesh.h"
#include "cudamanager/PxCudaContextManager.h"
#include "task/PxCpuDispatcher.h"
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

using namespace physx;

namespace {
thread_local std::string output;

struct Json {
    std::ostringstream stream;
    bool first = true;
    Json() { stream.precision(17); stream << '{'; }
    template <typename T> void add(const char* key, T value) {
        if (!first) stream << ',';
        first = false;
        stream << '"' << key << "\":" << value;
    }
    const char* finish() { stream << '}'; output = stream.str(); return output.c_str(); }
};

unsigned flags(PxScene* scene) {
    unsigned value = 0;
    const auto sceneFlags = scene->getFlags();
    if (sceneFlags.isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS)) value |= 1;
    if (scene->getBroadPhaseType() == PxBroadPhaseType::eGPU) value |= 2;
    if (sceneFlags.isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API)) value |= 4;
    auto* cuda = scene->getCudaContextManager();
    if (cuda && cuda->contextIsValid()) value |= 8;
    return value;
}

struct Inventory {
    std::uint64_t shapes = 0, simulationShapes = 0, links = 0;
    std::uint64_t convex = 0, convexGpu = 0, convexVertices = 0, maxConvexVertices = 0;
    std::uint64_t triangleMeshes = 0, triangles = 0;
    std::uint64_t geometry[PxGeometryType::eGEOMETRY_COUNT] = {};
    PxU32 positionMin = 256, positionMax = 0, velocityMin = 256, velocityMax = 0;
    std::uint64_t iterationObjects = 0;
    double mass = 0;
    void iterations(PxU32 position, PxU32 velocity) {
        ++iterationObjects;
        positionMin = std::min(positionMin, position);
        positionMax = std::max(positionMax, position);
        velocityMin = std::min(velocityMin, velocity);
        velocityMax = std::max(velocityMax, velocity);
    }
    void actor(PxRigidActor* actor) {
        std::vector<PxShape*> attached(actor->getNbShapes());
        if (!attached.empty()) actor->getShapes(attached.data(), PxU32(attached.size()));
        for (auto* shape : attached) {
            ++shapes;
            if (shape->getFlags().isSet(PxShapeFlag::eSIMULATION_SHAPE)) ++simulationShapes;
            const PxGeometry& geom = shape->getGeometry();
            const auto type = geom.getType();
            if (type >= 0 && type < PxGeometryType::eGEOMETRY_COUNT) ++geometry[type];
            if (type == PxGeometryType::eCONVEXMESH) {
                auto* mesh = static_cast<const PxConvexMeshGeometry&>(geom).convexMesh;
                ++convex;
                if (mesh->isGpuCompatible()) ++convexGpu;
                const auto vertices = mesh->getNbVertices();
                convexVertices += vertices;
                maxConvexVertices = std::max(maxConvexVertices, std::uint64_t(vertices));
            } else if (type == PxGeometryType::eTRIANGLEMESH) {
                ++triangleMeshes;
                triangles += static_cast<const PxTriangleMeshGeometry&>(geom).triangleMesh->getNbTriangles();
            }
        }
    }
};
}

extern "C" const char* eersim_scene_inventory(void* pointer) {
    if (!pointer) return "{\"error\":\"null scene\"}";
    try {
        auto* scene = static_cast<PxScene*>(pointer);
        Inventory inventory;
        const auto types = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
        std::vector<PxActor*> actors(scene->getNbActors(types));
        if (!actors.empty()) scene->getActors(types, actors.data(), PxU32(actors.size()));
        for (auto* actor : actors) {
            inventory.actor(static_cast<PxRigidActor*>(actor));
            if (actor->getType() == PxActorType::eRIGID_DYNAMIC) {
                auto* body = static_cast<PxRigidDynamic*>(actor);
                PxU32 position, velocity;
                body->getSolverIterationCounts(position, velocity);
                inventory.iterations(position, velocity);
                inventory.mass += body->getMass();
            }
        }
        std::vector<PxArticulationReducedCoordinate*> articulations(scene->getNbArticulations());
        if (!articulations.empty()) scene->getArticulations(articulations.data(), PxU32(articulations.size()));
        for (auto* articulation : articulations) {
            PxU32 position, velocity;
            articulation->getSolverIterationCounts(position, velocity);
            inventory.iterations(position, velocity);
            std::vector<PxArticulationLink*> links(articulation->getNbLinks());
            if (!links.empty()) articulation->getLinks(links.data(), PxU32(links.size()));
            inventory.links += links.size();
            for (auto* link : links) { inventory.actor(link); inventory.mass += link->getMass(); }
        }
        Json report;
        report.add("gpu_flags", flags(scene));
        const auto* dispatcher = scene->getCpuDispatcher();
        report.add("cpu_dispatcher_workers", dispatcher ? static_cast<int>(dispatcher->getWorkerCount()) : -1);
        auto* cuda = scene->getCudaContextManager();
        report.add("cuda_device", cuda && cuda->contextIsValid() ? cuda->getDevice() : -1);
        report.add("broadphase_type", int(scene->getBroadPhaseType()));
        report.add("solver_type", int(scene->getSolverType()));
        report.add("static_actors", scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC));
        report.add("dynamic_actors", scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC));
        report.add("articulations", articulations.size());
        report.add("articulation_links", inventory.links);
        report.add("constraints", scene->getNbConstraints());
        report.add("shapes", inventory.shapes);
        report.add("simulation_shapes", inventory.simulationShapes);
        report.add("dynamic_mass_kg", inventory.mass);
        report.add("iteration_objects", inventory.iterationObjects);
        report.add("position_iterations_min", inventory.iterationObjects ? inventory.positionMin : 0);
        report.add("position_iterations_max", inventory.positionMax);
        report.add("velocity_iterations_min", inventory.iterationObjects ? inventory.velocityMin : 0);
        report.add("velocity_iterations_max", inventory.velocityMax);
        report.add("convex_shapes", inventory.convex);
        report.add("convex_gpu_compatible_shapes", inventory.convexGpu);
        report.add("convex_gpu_incompatible_shapes", inventory.convex - inventory.convexGpu);
        report.add("convex_vertex_total_with_instances", inventory.convexVertices);
        report.add("max_convex_vertices", inventory.maxConvexVertices);
        report.add("triangle_mesh_shapes_gpu_cooking_unverified", inventory.triangleMeshes);
        report.add("triangle_count_with_instances", inventory.triangles);
        const char* names[] = {"sphere_shapes", "plane_shapes", "capsule_shapes", "box_shapes",
            "convex_core_shapes", "convex_mesh_shapes", "particle_shapes", "tetrahedron_shapes",
            "triangle_shapes", "heightfield_shapes", "custom_shapes"};
        for (int i = 0; i < PxGeometryType::eGEOMETRY_COUNT; ++i) report.add(names[i], inventory.geometry[i]);
        return report.finish();
    } catch (...) { return "{\"error\":\"native inventory inspection failed\"}"; }
}

extern "C" const char* eersim_scene_statistics(void* pointer) {
    if (!pointer) return "{\"error\":\"null scene\"}";
    try {
        PxSimulationStatistics stats;
        static_cast<PxScene*>(pointer)->getSimulationStatistics(stats);
        Json report;
        report.add("active_constraints", stats.nbActiveConstraints);
        report.add("active_dynamic_bodies", stats.nbActiveDynamicBodies);
        report.add("active_kinematic_bodies", stats.nbActiveKinematicBodies);
        report.add("dynamic_bodies_including_articulation_links", stats.nbDynamicBodies);
        report.add("axis_solver_constraints", stats.nbAxisSolverConstraints);
        report.add("discrete_contact_pairs_total", stats.nbDiscreteContactPairsTotal);
        report.add("discrete_contact_pairs_with_contacts", stats.nbDiscreteContactPairsWithContacts);
        report.add("discrete_contact_pairs_cache_hits", stats.nbDiscreteContactPairsWithCacheHits);
        report.add("new_pairs", stats.nbNewPairs);
        report.add("lost_pairs", stats.nbLostPairs);
        report.add("solver_partitions", stats.nbPartitions);
        std::uint64_t modified = 0;
        for (int i = 0; i < PxGeometryType::eGEOMETRY_COUNT; ++i)
            for (int j = i; j < PxGeometryType::eGEOMETRY_COUNT; ++j)
                modified += stats.getRbPairStats(PxSimulationStatistics::eMODIFIED_CONTACT_PAIRS,
                    PxGeometryType::Enum(i), PxGeometryType::Enum(j));
        report.add("modified_contact_pairs", modified);
        report.add("gpu_heap_bytes", stats.gpuMemHeap);
        report.add("gpu_broadphase_heap_bytes", stats.gpuMemHeapBroadPhase);
        report.add("gpu_narrowphase_heap_bytes", stats.gpuMemHeapNarrowPhase);
        report.add("gpu_solver_heap_bytes", stats.gpuMemHeapSolver);
        report.add("gpu_contacts_lifetime_high_water", stats.gpuDynamicsMemoryConfigStatistics.rigidContactCount);
        report.add("gpu_patches_lifetime_high_water", stats.gpuDynamicsMemoryConfigStatistics.rigidPatchCount);
        report.add("gpu_temp_buffer_lifetime_high_water_bytes", stats.gpuDynamicsMemoryConfigStatistics.tempBufferCapacity);
        return report.finish();
    } catch (...) { return "{\"error\":\"native statistics inspection failed\"}"; }
}
