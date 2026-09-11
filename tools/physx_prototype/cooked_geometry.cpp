// Read-only cooked convex inspection for the exact PhysX ovphysx-0.5.11 tag.
// Compile against its public headers; no PhysX/CUDA link libraries are required.
// Call only after synchronous initialization/step completion, while the scene lives.
// Deliberately does not read actor global poses: DirectGPU host poses can be stale.
#include "PxScene.h"
#include "PxRigidActor.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationLink.h"
#include "PxShape.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxConvexMesh.h"
#include <cmath>
#include <exception>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#define EERSIM_EXPORT extern "C" __declspec(dllexport)
#else
#define EERSIM_EXPORT extern "C" __attribute__((visibility("default")))
#endif

using namespace physx;

namespace {
thread_local std::string output;

void stringValue(std::ostream& out, const char* value) {
    if (!value) { out << "null"; return; }
    const char* hex = "0123456789abcdef";
    out << '"';
    for (const auto* ch = reinterpret_cast<const unsigned char*>(value); *ch; ++ch) {
        switch (*ch) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (*ch < 0x20) out << "\\u00" << hex[*ch >> 4] << hex[*ch & 15];
            else out << static_cast<char>(*ch);
        }
    }
    out << '"';
}

void number(std::ostream& out, PxReal value) {
    if (!std::isfinite(value)) throw std::runtime_error("nonfinite cooked geometry metadata");
    out << value;
}

void vector3(std::ostream& out, const PxVec3& value) {
    out << '['; number(out, value.x); out << ',';
    number(out, value.y); out << ','; number(out, value.z); out << ']';
}

void quaternion(std::ostream& out, const PxQuat& value) {
    out << '['; number(out, value.x); out << ',';
    number(out, value.y); out << ','; number(out, value.z); out << ',';
    number(out, value.w); out << ']';
}

struct Instance {
    PxRigidActor* actor;
    PxShape* shape;
    std::size_t meshId;
    std::size_t actorOrdinal;
    PxU32 shapeOrdinal;
};

struct Inventory {
    std::unordered_map<const PxConvexMesh*, std::size_t> ids;
    std::vector<const PxConvexMesh*> meshes;
    std::vector<Instance> instances;
    std::size_t actorCount = 0, shapeCount = 0, articulationLinks = 0;

    void add(PxRigidActor* actor) {
        const auto ordinal = actorCount++;
        std::vector<PxShape*> shapes(actor->getNbShapes());
        if (!shapes.empty()) actor->getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
        shapeCount += shapes.size();
        for (PxU32 i = 0; i < shapes.size(); ++i) {
            auto* shape = shapes[i];
            const auto& geometry = shape->getGeometry();
            if (geometry.getType() != PxGeometryType::eCONVEXMESH) continue;
            auto* mesh = static_cast<const PxConvexMeshGeometry&>(geometry).convexMesh;
            if (!mesh) throw std::runtime_error("null convex mesh");
            auto found = ids.emplace(mesh, meshes.size());
            if (found.second) meshes.push_back(mesh);
            instances.push_back({actor, shape, found.first->second, ordinal, i});
        }
    }
};

void writeMesh(std::ostream& out, const PxConvexMesh* mesh, std::size_t id) {
    out << "{\"id\":" << id << ",\"gpu_compatible\":"
        << (mesh->isGpuCompatible() ? "true" : "false")
        << ",\"vertex_count\":" << mesh->getNbVertices()
        << ",\"polygon_count\":" << mesh->getNbPolygons() << ",\"vertices\":[";
    for (PxU32 i = 0; i < mesh->getNbVertices(); ++i) {
        if (i) out << ',';
        vector3(out, mesh->getVertices()[i]);
    }
    out << "],\"polygons\":[";
    for (PxU32 i = 0; i < mesh->getNbPolygons(); ++i) {
        PxHullPolygon polygon;
        if (!mesh->getPolygonData(i, polygon)) throw std::runtime_error("convex polygon read failed");
        if (i) out << ',';
        out << "{\"plane\":[";
        for (unsigned axis = 0; axis < 4; ++axis) {
            if (axis) out << ',';
            number(out, polygon.mPlane[axis]);
        }
        out << "],\"indices\":[";
        for (PxU32 j = 0; j < polygon.mNbVerts; ++j) {
            if (j) out << ',';
            const auto vertex = mesh->getIndexBuffer()[polygon.mIndexBase + j];
            if (vertex >= mesh->getNbVertices()) throw std::runtime_error("invalid convex polygon index");
            out << static_cast<unsigned>(vertex);
        }
        out << "]}";
    }
    out << "]}";
}

void writeInstance(std::ostream& out, const Instance& instance) {
    const auto& geometry = static_cast<const PxConvexMeshGeometry&>(instance.shape->getGeometry());
    const auto localPose = instance.shape->getLocalPose();
    out << "{\"actor_name\":"; stringValue(out, instance.actor->getName());
    out << ",\"shape_name\":"; stringValue(out, instance.shape->getName());
    out << ",\"actor_type\":" << static_cast<unsigned>(instance.actor->getType())
        << ",\"actor_ordinal\":" << instance.actorOrdinal
        << ",\"shape_ordinal\":" << instance.shapeOrdinal
        << ",\"mesh_id\":" << instance.meshId
        << ",\"simulation_shape\":"
        << (instance.shape->getFlags().isSet(PxShapeFlag::eSIMULATION_SHAPE) ? "true" : "false")
        << ",\"local_position\":";
    vector3(out, localPose.p);
    out << ",\"local_quaternion_xyzw\":"; quaternion(out, localPose.q);
    out << ",\"mesh_scale\":"; vector3(out, geometry.scale.scale);
    out << ",\"mesh_scale_rotation_xyzw\":"; quaternion(out, geometry.scale.rotation);
    // Include the authoritative native scale matrix to avoid quaternion-convention guesses.
    const auto matrix = geometry.scale.toMat33();
    out << ",\"mesh_scale_matrix_columns\":[";
    vector3(out, matrix.column0); out << ',';
    vector3(out, matrix.column1); out << ',';
    vector3(out, matrix.column2); out << ']';
    out << ",\"contact_offset\":"; number(out, instance.shape->getContactOffset());
    out << ",\"rest_offset\":"; number(out, instance.shape->getRestOffset());
    out << '}';
}
} // namespace

EERSIM_EXPORT const char* eersim_cooked_convex_geometry(void* pointer) {
    if (!pointer) return "{\"error\":\"null scene\"}";
    try {
        auto* scene = static_cast<PxScene*>(pointer);
        Inventory inventory;
        const auto types = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
        std::vector<PxActor*> actors(scene->getNbActors(types));
        if (!actors.empty()) scene->getActors(types, actors.data(), static_cast<PxU32>(actors.size()));
        for (auto* actor : actors) inventory.add(static_cast<PxRigidActor*>(actor));
        std::vector<PxArticulationReducedCoordinate*> articulations(scene->getNbArticulations());
        if (!articulations.empty())
            scene->getArticulations(articulations.data(), static_cast<PxU32>(articulations.size()));
        for (auto* articulation : articulations) {
            std::vector<PxArticulationLink*> links(articulation->getNbLinks());
            if (!links.empty()) articulation->getLinks(links.data(), static_cast<PxU32>(links.size()));
            inventory.articulationLinks += links.size();
            for (auto* link : links) inventory.add(link);
        }
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out.precision(17);
        out << "{\"schema_version\":1,\"physx_tag\":\"ovphysx-0.5.11\","
               "\"coordinate_space\":\"cooked mesh local; instance transform maps to actor local\","
               "\"actor_count\":" << inventory.actorCount
            << ",\"articulation_link_count\":" << inventory.articulationLinks
            << ",\"all_shape_count\":" << inventory.shapeCount
            << ",\"unique_convex_mesh_count\":" << inventory.meshes.size()
            << ",\"convex_instance_count\":" << inventory.instances.size() << ",\"meshes\":[";
        for (std::size_t i = 0; i < inventory.meshes.size(); ++i) {
            if (i) out << ',';
            writeMesh(out, inventory.meshes[i], i);
        }
        out << "],\"instances\":[";
        for (std::size_t i = 0; i < inventory.instances.size(); ++i) {
            if (i) out << ',';
            writeInstance(out, inventory.instances[i]);
        }
        out << "]}";
        output = out.str();
    } catch (const std::exception& error) {
        std::ostringstream out;
        out << "{\"error\":"; stringValue(out, error.what()); out << '}';
        output = out.str();
    } catch (...) {
        output = "{\"error\":\"native cooked convex inspection failed\"}";
    }
    return output.c_str();
}
