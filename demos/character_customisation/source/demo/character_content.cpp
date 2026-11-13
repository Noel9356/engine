// character_content.cpp - body parts for the customisation demo.

#include "character_content.hpp"
#include "character_metrics.gen.hpp"

#include "geometry.hpp"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdio>
#include <filesystem>
#include <utility>

namespace demo
{
namespace
{

using engine::character::AnchorSpace;
using engine::character::AssemblyProfileDefinition;
using engine::character::CompatibilityRuleDefinition;
using engine::character::ConnectorAnchorDefinition;
using engine::character::ConnectorDefinition;
using engine::character::MaterialSlotDefinition;
using engine::character::ParameterBinding;
using engine::character::ParameterBindingTarget;
using engine::character::ParameterDefinition;
using engine::character::PartDefinition;
using engine::character::ProfileConnection;
using engine::character::ProfilePart;
using engine::character::RigDefinition;
using engine::character::RigJointDefinition;

using metrics::kSpecies;
using metrics::kSpeciesCount;

std::vector<glm::vec3> normalsScratch;

// Loads a colored OBJ produced by scripts/extract_pipeline.py: "v x y z r g
// b" vertices, "vn" normals, indexed "f a//a b//b c//c" faces (the extractor
// dedupes vertices), and optional "m <name>" morph blocks with one "md dx dy
// dz" line per vertex (deltas live in the same order as the "v" lines). The
// renderer consumes a flat layout, so every face corner is expanded here.
// Returns false when the file is missing or holds no triangles.
bool loadPartObj(const std::filesystem::path &path, MeshData &outMesh,
    std::vector<PartMorph> &outMorphs)
{
    std::FILE *file = std::fopen(path.string().c_str(), "rb");
    if (file == nullptr)
    {
        return false;
    }
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> colors;
    std::vector<PartMorph> morphs;
    normalsScratch.clear();
    std::vector<std::array<unsigned, 6>> faces;
    std::string morphName;
    std::vector<glm::vec3> morphDeltas;
    char line[512];
    while (std::fgets(line, sizeof(line), file) != nullptr)
    {
        if (line[0] == 'v' && line[1] == ' ')
        {
            float x = 0.0f, y = 0.0f, z = 0.0f, r = 1.0f, g = 1.0f, b = 1.0f;
            if (std::sscanf(line + 2, "%f %f %f %f %f %f", &x, &y, &z, &r, &g, &b) < 3)
            {
                std::fclose(file);
                return false;
            }
            positions.emplace_back(x, y, z);
            colors.emplace_back(r, g, b);
        }
        else if (line[0] == 'v' && line[1] == 'n')
        {
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (std::sscanf(line + 3, "%f %f %f", &x, &y, &z) == 3)
            {
                normalsScratch.emplace_back(x, y, z);
            }
        }
        else if (line[0] == 'f' && line[1] == ' ')
        {
            unsigned a = 0, an = 0, b = 0, bn = 0, c = 0, cn = 0;
            const bool shaded = std::sscanf(line + 2, "%u//%u %u//%u %u//%u", &a, &an, &b, &bn, &c, &cn) == 6;
            const bool plain = !shaded && std::sscanf(line + 2, "%u %u %u", &a, &b, &c) == 3;
            if (!shaded && !plain)
            {
                std::fclose(file);
                return false;
            }
            faces.push_back(std::array<unsigned, 6>{a, an, b, bn, c, cn});
        }
        else if (line[0] == 'm' && line[1] == ' ')
        {
            if (!morphName.empty())
            {
                morphs.push_back({morphName, morphDeltas});
            }
            morphName = std::string(line + 2);
            while (!morphName.empty() && (morphName.back() == '\n' || morphName.back() == '\r'))
            {
                morphName.pop_back();
            }
            morphDeltas.clear();
        }
        else if (line[0] == 'm' && line[1] == 'd' && line[2] == ' ')
        {
            float dx = 0.0f, dy = 0.0f, dz = 0.0f;
            if (std::sscanf(line + 3, "%f %f %f", &dx, &dy, &dz) == 3)
            {
                morphDeltas.emplace_back(dx, dy, dz);
            }
        }
    }
    std::fclose(file);
    if (!morphName.empty())
    {
        morphs.push_back({morphName, morphDeltas});
    }
    for (const auto &face : faces)
    {
        const unsigned indices[3]  = {face[0], face[2], face[4]};
        const unsigned normalIndex[3] = {face[1], face[3], face[5]};
        const std::uint32_t base = static_cast<std::uint32_t>(outMesh.vertices.size());
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const unsigned index = indices[corner];
            if (index == 0 || index > positions.size())
            {
                return false;
            }
            Vertex vertex;
            vertex.position = positions[index - 1];
            vertex.color    = colors[index - 1];
            const unsigned normal = normalIndex[corner] != 0 ? normalIndex[corner] : index;
            vertex.normal   = normalsScratch.empty()
                                ? glm::vec3(0.0f, 1.0f, 0.0f)
                                : normalsScratch[std::min<std::size_t>(normal - 1, normalsScratch.size() - 1)];
            vertex.uv       = {0.0f, 0.0f};
            outMesh.vertices.push_back(vertex);
        }
        outMesh.indices.insert(outMesh.indices.end(), {base, base + 1, base + 2});
    }
    // The morph blocks are authored per unique "v" line; expand them onto
    // the flat corner layout so indices align.
    for (PartMorph &morph : morphs)
    {
        if (morph.deltas.size() != positions.size())
        {
            continue;
        }
        std::vector<glm::vec3> expanded;
        expanded.reserve(outMesh.vertices.size());
        for (const auto &face : faces)
        {
            const unsigned indices[3] = {face[0], face[2], face[4]};
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                expanded.push_back(morph.deltas[indices[corner] - 1]);
            }
        }
        morph.deltas = std::move(expanded);
    }
    outMorphs = std::move(morphs);
    return !outMesh.indices.empty();
}

// ---- content registration helpers ------------------------------------------

ConnectorAnchorDefinition partLocalAnchor(float x, float y, float z)
{
    ConnectorAnchorDefinition anchor{};
    anchor.space          = AnchorSpace::PartLocal;
    anchor.translation[0] = x;
    anchor.translation[1] = y;
    anchor.translation[2] = z;
    return anchor;
}

ConnectorAnchorDefinition partLocalAnchor(float x, float y, float z, const glm::quat &rotation)
{
    ConnectorAnchorDefinition anchor = partLocalAnchor(x, y, z);
    anchor.rotation[0]    = rotation.x;
    anchor.rotation[1]    = rotation.y;
    anchor.rotation[2]    = rotation.z;
    anchor.rotation[3]    = rotation.w;
    return anchor;
}

ConnectorAnchorDefinition jointAnchor(
    const char *jointSemantic, const glm::vec3 &position, const glm::quat &rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
{
    ConnectorAnchorDefinition anchor = partLocalAnchor(position.x, position.y, position.z, rotation);
    anchor.space                     = AnchorSpace::Joint;
    anchor.jointSemantic             = jointSemantic;
    return anchor;
}

ParameterBinding scaleBinding(const char *parameter, ParameterBindingTarget target, float scale)
{
    ParameterBinding binding{};
    binding.parameter = parameter;
    binding.target    = target;
    binding.scale     = scale;
    return binding;
}

} // namespace

// ---- geometry plumbing ------------------------------------------------------

bool CharacterContent::loadGeometry(const std::string &geometryId, const std::filesystem::path &path,
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle)
{
    GeometryEntry entry;
    if (!loadPartObj(path, entry.rest, entry.morphs))
    {
        std::fprintf(stderr, "demo: character part '%s' missing or unreadable at '%s'\n",
            geometryId.c_str(), path.string().c_str());
        return false;
    }
    entry.triangles = static_cast<std::uint32_t>(entry.rest.indices.size() / 3);
    geometries_[geometryId] = std::move(entry);
    return uploadGeometry(geometryId, renderer, rendererHandle);
}

bool CharacterContent::uploadGeometry(const std::string &geometryId,
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle)
{
    GeometryEntry &entry = geometries_[geometryId];
    if (entry.handle != engine::renderer::invalidMesh)
    {
        renderer.destroyMesh(renderer.self, rendererHandle, entry.handle);
        entry.handle = engine::renderer::invalidMesh;
    }
    const engine::renderer::MeshDescription description = entry.rest.description();
    engine::renderer::RenderError           error      = engine::renderer::RenderError::None;
    entry.handle = renderer.createMesh(renderer.self, rendererHandle, &description, &error);
    if (entry.handle == engine::renderer::invalidMesh)
    {
        std::fprintf(stderr,
            "demo: character mesh '%s' upload failed (%u)\n", geometryId.c_str(), static_cast<unsigned>(error));
        return false;
    }
    return true;
}

void CharacterContent::mirrorAcrossX(const char *leftId, const char *rightId,
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle)
{
    GeometryEntry mirrored;
    mirrored.rest         = geometries_[leftId].rest;
    mirrored.morphs       = geometries_[leftId].morphs;
    mirrored.triangles    = geometries_[leftId].triangles;
    for (Vertex &vertex : mirrored.rest.vertices)
    {
        vertex.position.x *= -1.0f;
        vertex.normal.x *= -1.0f;
    }
    for (PartMorph &morph : mirrored.morphs)
    {
        for (glm::vec3 &delta : morph.deltas)
        {
            delta.x *= -1.0f;
        }
    }
    for (std::size_t index = 0; index + 2 < mirrored.rest.indices.size(); index += 3)
    {
        std::swap(mirrored.rest.indices[index], mirrored.rest.indices[index + 1]);
    }
    geometries_[rightId] = std::move(mirrored);
    uploadGeometry(rightId, renderer, rendererHandle);
}

// ---- registration -----------------------------------------------------------

bool CharacterContent::create(engine::character::CharacterCapability &character,
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle,
    const std::string &assetRoot)
{
    using namespace engine::character;

    CharacterError error = CharacterError::None;

    // Generated ids/display names and per-part connector definitions; the
    // definitions borrow these pointers until the module copies them during
    // the registerRig/registerPart calls in this function.
    std::vector<std::string> nameStorage;
    std::vector<engine::character::ConnectorDefinition> connectorStorage;
    nameStorage.reserve(512);
    connectorStorage.reserve(256);

    // ---- global parameters (the registry; assets map them locally) --------
    // body.plan selects the assembly profile (0 humanoid, 1 taur,
    // 2 quadruped); the screen rebuilds the assembly when it changes.
    const ParameterDefinition parameters[] = {
        {"body.plan", "Body Plan", 0.0f, 2.0f, 0.0f},
        {"body.height", "Body Height", 0.75f, 1.35f, 1.0f},
        {"body.width", "Body Width", 0.70f, 1.45f, 1.0f},
        {"body.muscle_mass", "Muscle Mass", 0.70f, 1.55f, 1.0f},
        {"head.size", "Head Size", 0.75f, 1.35f, 1.0f},
        {"muzzle.length", "Muzzle Length", 0.75f, 1.35f, 1.0f},
        {"eyes.width", "Eye Spacing", 0.85f, 1.20f, 1.0f},
        {"limbs.length", "Limb Length", 0.75f, 1.35f, 1.0f},
        {"limbs.thickness", "Limb Thickness", 0.65f, 1.55f, 1.0f},
        {"wings.span", "Wingspan", 0.55f, 1.45f, 1.0f},
        {"tail.length", "Tail Length", 0.55f, 1.50f, 1.0f},
        {"tail.thickness", "Tail Thickness", 0.55f, 1.65f, 1.0f},
        {"taur.scale", "Lower Body Size", 0.75f, 1.40f, 1.0f},
        {"skin.tone", "Skin Tone", 0.0f, 4.0f, 0.0f},
        {"fur.coat", "Fur Coat", 0.0f, 5.0f, 0.0f},
        {"fur.markings", "Fur Markings", 0.0f, 3.0f, 0.0f},
    };
    for (const ParameterDefinition &definition : parameters)
    {
        if (!character.registerParameter(character.self, &definition, &error))
        {
            std::fprintf(stderr,
                "demo: parameter registration failed for '%s' (%u)\n",
                definition.id,
                static_cast<unsigned>(error));
            return false;
        }
    }

    // ---- connector compatibility rules (data-defined, extensible) ---------
    const CompatibilityRuleDefinition rules[] = {
        {"rule_head", "head.mount", "head.root", true, 0.55f, 1.80f, nullptr, nullptr},
        {"rule_arm", "arm.mount", "arm.root", true, 0.45f, 2.00f, nullptr, nullptr},
        {"rule_leg", "leg.mount", "leg.root", true, 0.45f, 2.00f, nullptr, nullptr},
        {"rule_wing", "wing.mount", "wing.root", true, 0.30f, 2.80f, nullptr, nullptr},
        {"rule_appendage", "appendage.mount", "appendage.root", true, 0.30f, 2.80f, nullptr, nullptr},
        {"rule_body_mount", "body.mount", "body.root", true, 0.50f, 2.00f, nullptr, nullptr},
        {"rule_ear", "ear.mount", "ear.root", true, 0.30f, 3.20f, nullptr, nullptr},
        {"rule_garment", "garment.mount", "garment.root", true, 0.30f, 3.20f, nullptr, nullptr},
    };
    for (const CompatibilityRuleDefinition &rule : rules)
    {
        if (!character.registerCompatibilityRule(character.self, &rule, &error))
        {
            std::fprintf(
                stderr, "demo: rule registration failed for '%s' (%u)\n", rule.id, static_cast<unsigned>(error));
            return false;
        }
    }

    // ---- human torso rigs (one per torso variant, measured sockets) --------
    const auto buildTorsoRig = [&](const char *rigId, const glm::vec3 &neck, const glm::vec3 &shoulderLeft,
                                 const glm::vec3 &shoulderRight, const glm::vec3 &hipLeft, const glm::vec3 &hipRight,
                                 const glm::vec3 &tailSocket, const glm::quat &tailRotation,
                                 const glm::vec3 &wingLeft, const glm::vec3 &wingRight, const glm::vec3 &belt,
                                 const glm::vec3 &padLeft, const glm::vec3 &padRight) {
        const RigJointDefinition joints[] = {
            {"pelvis", -1, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"neck_base", 0, {neck.x, neck.y, neck.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"shoulder_left", 0, {shoulderLeft.x, shoulderLeft.y, shoulderLeft.z}, {0.0f, 0.0f, 0.0f, 1.0f},
                {1.0f, 1.0f, 1.0f}},
            {"shoulder_right", 0, {shoulderRight.x, shoulderRight.y, shoulderRight.z}, {0.0f, 0.0f, 0.0f, 1.0f},
                {1.0f, 1.0f, 1.0f}},
            {"hip_left", 0, {hipLeft.x, hipLeft.y, hipLeft.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"hip_right", 0, {hipRight.x, hipRight.y, hipRight.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"wing_left", 0, {wingLeft.x, wingLeft.y, wingLeft.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"wing_right", 0, {wingRight.x, wingRight.y, wingRight.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"tail_socket", 0, {tailSocket.x, tailSocket.y, tailSocket.z},
                {tailRotation.x, tailRotation.y, tailRotation.z, tailRotation.w}, {1.0f, 1.0f, 1.0f}},
            {"belt_socket", 0, {belt.x, belt.y, belt.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"pad_left", 0, {padLeft.x, padLeft.y, padLeft.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
            {"pad_right", 0, {padRight.x, padRight.y, padRight.z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        };
        const RigDefinition definition{rigId, joints, static_cast<std::uint32_t>(std::size(joints))};
        return character.registerRig(character.self, &definition, &error);
    };

    const glm::quat rangerTailRotation = metrics::kRangerTorsoSocketTailRotation;
    const glm::quat bareTailRotation   = metrics::kBareTorsoSocketTailRotation;
    if (!buildTorsoRig("torso_ranger_core", metrics::kRangerTorsoNeckRing,
            metrics::kRangerTorsoSocketShoulderLeft, metrics::kRangerTorsoSocketShoulderRight,
            metrics::kRangerTorsoSocketHipLeft, metrics::kRangerTorsoSocketHipRight,
            metrics::kRangerTorsoSocketTail, rangerTailRotation, metrics::kRangerTorsoSocketWingLeft,
            metrics::kRangerTorsoSocketWingRight, metrics::kRangerTorsoSocketBelt,
            metrics::kRangerTorsoSocketPadLeft, metrics::kRangerTorsoSocketPadRight)
        || !buildTorsoRig("torso_bare_core", metrics::kBareTorsoNeckRing, metrics::kBareTorsoSocketShoulderLeft,
            metrics::kBareTorsoSocketShoulderRight, metrics::kBareTorsoSocketHipLeft,
            metrics::kBareTorsoSocketHipRight, metrics::kBareTorsoSocketTail, bareTailRotation,
            metrics::kBareTorsoSocketWingLeft, metrics::kBareTorsoSocketWingRight,
            metrics::kBareTorsoSocketBelt, metrics::kBareTorsoSocketPadLeft, metrics::kBareTorsoSocketPadRight))
    {
        std::fprintf(stderr, "demo: torso rig registration failed (%u)\n", static_cast<unsigned>(error));
        return false;
    }

    // ---- species rigs (taur sockets on the animal's own body) --------------
    std::vector<RigJointDefinition> rigJointStorage;
    rigJointStorage.reserve(static_cast<std::size_t>(kSpeciesCount) * 5);
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const metrics::SpeciesMetrics &entry = kSpecies[species];
        const std::string rigId = std::string("quad_") + entry.id + "_core";
        nameStorage.push_back(rigId);
        const std::size_t firstJoint = rigJointStorage.size();
        const glm::quat torsoRotation = entry.torsoMountRotation;
        const glm::quat tailRotation  = entry.tailMountRotation;
        RigJointDefinition bodyRoot{};
        bodyRoot.semantic    = "body_root";
        bodyRoot.parentIndex = -1;
        rigJointStorage.push_back(bodyRoot);
        RigJointDefinition torsoMount{};
        torsoMount.semantic        = "torso_mount";
        torsoMount.parentIndex     = 0;
        torsoMount.translation[0]  = entry.torsoMount.x;
        torsoMount.translation[1]  = entry.torsoMount.y;
        torsoMount.translation[2]  = entry.torsoMount.z;
        torsoMount.rotation[0]     = torsoRotation.x;
        torsoMount.rotation[1]     = torsoRotation.y;
        torsoMount.rotation[2]     = torsoRotation.z;
        torsoMount.rotation[3]     = torsoRotation.w;
        rigJointStorage.push_back(torsoMount);
        RigJointDefinition tailMount{};
        tailMount.semantic        = "tail_mount";
        tailMount.parentIndex     = 0;
        tailMount.translation[0]  = entry.tailMount.x;
        tailMount.translation[1]  = entry.tailMount.y;
        tailMount.translation[2]  = entry.tailMount.z;
        tailMount.rotation[0]     = tailRotation.x;
        tailMount.rotation[1]     = tailRotation.y;
        tailMount.rotation[2]     = tailRotation.z;
        tailMount.rotation[3]     = tailRotation.w;
        rigJointStorage.push_back(tailMount);
        RigJointDefinition withersLeft{};
        withersLeft.semantic        = "withers_left";
        withersLeft.parentIndex     = 0;
        withersLeft.translation[0]  = entry.withers.x - 0.02f;
        withersLeft.translation[1]  = entry.withers.y;
        withersLeft.translation[2]  = entry.withers.z;
        rigJointStorage.push_back(withersLeft);
        RigJointDefinition withersRight{};
        withersRight.semantic        = "withers_right";
        withersRight.parentIndex     = 0;
        withersRight.translation[0]  = entry.withers.x + 0.02f;
        withersRight.translation[1]  = entry.withers.y;
        withersRight.translation[2]  = entry.withers.z;
        rigJointStorage.push_back(withersRight);
        const RigDefinition speciesRig{nameStorage.back().c_str(), &rigJointStorage[firstJoint], 5};
        if (!character.registerRig(character.self, &speciesRig, &error))
        {
            std::fprintf(stderr, "demo: rig registration failed for '%s' (%u)\n", nameStorage.back().c_str(),
                static_cast<unsigned>(error));
            return false;
        }
    }

    // ---- parts table --------------------------------------------------------
    std::vector<PartDefinition> parts;
    parts.reserve(128);

    // ---- torsos (the clothing toggle swaps the variants) -------------------
    const auto addTorsoPart = [&](const char *id, const char *display, const char *rig) {
        static const ConnectorDefinition torsoConnectors[] = {
            {"neck.base", "head.mount", jointAnchor("neck_base", glm::vec3(0.0f)), nullptr, 1},
            {"shoulder.left", "arm.mount", jointAnchor("shoulder_left", glm::vec3(0.0f)), nullptr, 1},
            {"shoulder.right", "arm.mount", jointAnchor("shoulder_right", glm::vec3(0.0f)), nullptr, 1},
            {"hip.left", "leg.mount", jointAnchor("hip_left", glm::vec3(0.0f)), nullptr, 1},
            {"hip.right", "leg.mount", jointAnchor("hip_right", glm::vec3(0.0f)), nullptr, 1},
            {"wings.left", "wing.mount", jointAnchor("wing_left", glm::vec3(0.0f)), nullptr, 1},
            {"wings.right", "wing.mount", jointAnchor("wing_right", glm::vec3(0.0f)), nullptr, 1},
            {"tail.base", "appendage.mount", jointAnchor("tail_socket", glm::vec3(0.0f)), nullptr, 1},
            {"belt.mount", "garment.mount", jointAnchor("belt_socket", glm::vec3(0.0f)), nullptr, 1},
            {"pad.left", "garment.mount", jointAnchor("pad_left", glm::vec3(0.0f)), nullptr, 1},
            {"pad.right", "garment.mount", jointAnchor("pad_right", glm::vec3(0.0f)), nullptr, 1},
            {"body.root", "body.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding torsoBindings[] = {
            scaleBinding("body.height", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("body.width", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("body.width", ParameterBindingTarget::PartScaleZ, 0.60f),
            scaleBinding("body.muscle_mass", ParameterBindingTarget::PartScaleX, 0.25f),
            scaleBinding("body.muscle_mass", ParameterBindingTarget::PartScaleZ, 0.25f),
        };
        static const char *torsoTags[] = {"slot:torso", "plan:torso"};
        static const MaterialSlotDefinition torsoSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.75f, 0.0f}};
        PartDefinition part{};
        part.id                    = id;
        part.displayName           = display;
        part.geometryId            = id;
        part.rigId                 = rig;
        part.connectors            = torsoConnectors;
        part.connectorCount        = static_cast<std::uint32_t>(std::size(torsoConnectors));
        part.parameterBindings     = torsoBindings;
        part.parameterBindingCount = static_cast<std::uint32_t>(std::size(torsoBindings));
        part.materialSlots         = torsoSlots;
        part.materialSlotCount     = 1;
        part.tags                  = torsoTags;
        part.tagCount              = 2;
        part.boundsCenter[1]       = 0.33f;
        part.boundsCenter[2]       = -0.02f;
        part.boundsRadius          = 0.55f;
        parts.push_back(part);
    };
    addTorsoPart("torso_ranger", "Ranger Torso", "torso_ranger_core");
    addTorsoPart("torso_bare", "Bare Torso", "torso_bare_core");

    // ---- limbs (bare + ranger variants; left pieces, mirrored at load) -----
    const auto addArmPart = [&](const char *id, const char *display, const glm::vec3 &bracerSocket,
                                const char *const *tags, std::uint32_t tagCount) {
        static const ParameterBinding armBindings[] = {
            scaleBinding("limbs.length", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const MaterialSlotDefinition limbSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.72f, 0.0f}};
        const std::size_t connectorBase = connectorStorage.size();
        connectorStorage.push_back({"shoulder.root", "arm.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1});
        connectorStorage.push_back({"bracer.mount", "garment.mount",
            partLocalAnchor(bracerSocket.x, bracerSocket.y, bracerSocket.z), nullptr, 1});
        PartDefinition part{};
        part.id                    = id;
        part.displayName           = display;
        part.geometryId            = id;
        part.connectors            = &connectorStorage[connectorBase];
        part.connectorCount        = 2;
        part.parameterBindings     = armBindings;
        part.parameterBindingCount = 3;
        part.materialSlots         = limbSlots;
        part.materialSlotCount     = 1;
        part.tags                  = tags;
        part.tagCount              = tagCount;
        part.boundsCenter[1]       = -0.35f;
        part.boundsRadius          = 0.50f;
        parts.push_back(part);
    };
    static const char *armBareTags[]    = {"slot:arms", "skin"};
    static const char *armRangerTags[]  = {"slot:arms", "clothing"};
    addArmPart("arm_bare_left", "Arm (Bare)", metrics::kBareArmBracerSocket, armBareTags, 2);
    addArmPart("arm_ranger_left", "Arm (Ranger)", metrics::kRangerArmBracerSocket, armRangerTags, 2);
    // right variants share the left definitions; the geometry is mirrored
    addArmPart("arm_bare_right", "Arm (Bare)", metrics::kBareArmBracerSocket, armBareTags, 2);
    addArmPart("arm_ranger_right", "Arm (Ranger)", metrics::kRangerArmBracerSocket, armRangerTags, 2);

    const auto addLegPart = [&](const char *id, const char *display, const char *const *tags,
                                std::uint32_t tagCount) {
        static const ConnectorDefinition legConnectors[] = {
            {"hip.root", "leg.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding legBindings[] = {
            scaleBinding("limbs.length", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const MaterialSlotDefinition limbSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.72f, 0.0f}};
        PartDefinition part{};
        part.id                    = id;
        part.displayName           = display;
        part.geometryId            = id;
        part.connectors            = legConnectors;
        part.connectorCount        = 1;
        part.parameterBindings     = legBindings;
        part.parameterBindingCount = 3;
        part.materialSlots         = limbSlots;
        part.materialSlotCount     = 1;
        part.tags                  = tags;
        part.tagCount              = tagCount;
        part.boundsCenter[1]       = -0.50f;
        part.boundsRadius          = 0.60f;
        parts.push_back(part);
    };
    static const char *legBareTags[]   = {"slot:legs", "skin"};
    static const char *legRangerTags[] = {"slot:legs", "clothing"};
    addLegPart("leg_bare_left", "Leg (Bare)", legBareTags, 2);
    addLegPart("leg_ranger_left", "Booted Leg (Ranger)", legRangerTags, 2);
    addLegPart("leg_bare_right", "Leg (Bare)", legBareTags, 2);
    addLegPart("leg_ranger_right", "Booted Leg (Ranger)", legRangerTags, 2);

    // ---- clothing overlays --------------------------------------------------
    const auto addGarmentPart = [&](const char *id, const char *display, float roughness) {
        static const ConnectorDefinition garmentConnectors[] = {
            {"garment.root", "garment.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const char *garmentTags[] = {"slot:garment", "clothing"};
        static const MaterialSlotDefinition garmentSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, roughness, 0.0f}};
        PartDefinition part{};
        part.id             = id;
        part.displayName    = display;
        part.geometryId     = id;
        part.connectors     = garmentConnectors;
        part.connectorCount = 1;
        part.materialSlots  = garmentSlots;
        part.materialSlotCount = 1;
        part.tags           = garmentTags;
        part.tagCount       = 2;
        part.boundsCenter[1] = 0.08f;
        part.boundsRadius   = 0.35f;
        parts.push_back(part);
    };
    addGarmentPart("clothes_belt", "Ranger Belt", 0.70f);
    addGarmentPart("clothes_bracer_left", "Ranger Bracers", 0.55f);
    addGarmentPart("clothes_bracer_right", "Ranger Bracers", 0.55f);
    addGarmentPart("clothes_pauldron_left", "Ranger Pauldron", 0.50f);
    addGarmentPart("clothes_pauldron_right", "Ranger Pauldron", 0.50f);
    addGarmentPart("clothes_hood", "Ranger Hood", 0.80f);

    // ---- human head (kept from the artist-split pack) -----------------------
    {
        static const ConnectorDefinition headConnectors[] = {
            {"head.root", "head.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
            {"hood.mount", "garment.mount", partLocalAnchor(0.0f, 0.03f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding headBindings[] = {
            scaleBinding("head.size", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *headTags[] = {"slot:head", "skin"};
        static const MaterialSlotDefinition headSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.70f, 0.0f}};
        PartDefinition part{};
        part.id                    = "head_human";
        part.displayName           = "Human Head";
        part.geometryId            = "head_human";
        part.connectors            = headConnectors;
        part.connectorCount        = 2;
        part.parameterBindings     = headBindings;
        part.parameterBindingCount = 3;
        part.materialSlots         = headSlots;
        part.materialSlotCount     = 1;
        part.tags                  = headTags;
        part.tagCount              = 2;
        part.boundsCenter[1]       = 0.15f;
        part.boundsRadius          = 0.25f;
        parts.push_back(part);
    }

    // ---- dragon head (kept) ---------------------------------------------------
    {
        static const ConnectorDefinition headConnectors[] = {
            {"head.root", "head.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
            {"hood.mount", "garment.mount", partLocalAnchor(0.0f, 0.03f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding headBindings[] = {
            scaleBinding("head.size", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *headTags[] = {"slot:head"};
        static const MaterialSlotDefinition headSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.60f, 0.0f}};
        PartDefinition part{};
        part.id                    = "head_dragon";
        part.displayName           = "Dragon Head";
        part.geometryId            = "head_dragon";
        part.connectors            = headConnectors;
        part.connectorCount        = 2;
        part.parameterBindings     = headBindings;
        part.parameterBindingCount = 3;
        part.materialSlots         = headSlots;
        part.materialSlotCount     = 1;
        part.tags                  = headTags;
        part.tagCount              = 1;
        part.boundsCenter[1]       = 0.12f;
        part.boundsCenter[2]       = 0.06f;
        part.boundsRadius          = 0.22f;
        parts.push_back(part);
    }

    // ---- species heads (furry options with ear sockets + morphs) ------------
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const metrics::SpeciesMetrics &entry = kSpecies[species];
        static const ParameterBinding speciesHeadBindings[] = {
            scaleBinding("head.size", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *speciesHeadTags[] = {"slot:head"};
        static const MaterialSlotDefinition speciesHeadSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.75f, 0.0f}};
        const glm::quat earLeft  = entry.earRotationLeft;
        const glm::quat earRight = entry.earRotationRight;
        const std::size_t connectorBase = connectorStorage.size();
        connectorStorage.push_back({"head.root", "head.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1});
        const std::uint32_t connectorCount = entry.hasEarSockets ? 4u : 2u;
        if (entry.hasEarSockets)
        {
            connectorStorage.push_back({"ear.left", "ear.mount",
                partLocalAnchor(entry.earSocketLeft.x, entry.earSocketLeft.y, entry.earSocketLeft.z, earLeft),
                nullptr, 1});
            connectorStorage.push_back({"ear.right", "ear.mount",
                partLocalAnchor(entry.earSocketRight.x, entry.earSocketRight.y, entry.earSocketRight.z, earRight),
                nullptr, 1});
        }
        connectorStorage.push_back({"hood.mount", "garment.mount", partLocalAnchor(0.0f, 0.03f, 0.0f), nullptr, 1});
        const ConnectorDefinition *connectors = &connectorStorage[connectorBase];
        const std::string id      = std::string("head_") + entry.id;
        const std::string display = std::string(entry.display) + " Head";
        nameStorage.push_back(id);
        nameStorage.push_back(display);
        PartDefinition part{};
        part.id                    = nameStorage[nameStorage.size() - 2].c_str();
        part.displayName           = nameStorage[nameStorage.size() - 1].c_str();
        part.geometryId            = part.id;
        part.connectors            = connectors;
        part.connectorCount        = connectorCount;
        part.parameterBindings     = speciesHeadBindings;
        part.parameterBindingCount = 3;
        part.materialSlots         = speciesHeadSlots;
        part.materialSlotCount     = 1;
        part.tags                  = speciesHeadTags;
        part.tagCount              = 1;
        part.boundsCenter[1]       = 0.15f;
        part.boundsCenter[2]       = 0.02f;
        part.boundsRadius          = 0.28f;
        parts.push_back(part);
    }

    // ---- species ears (left pieces; mirrored right) ---------------------------
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const metrics::SpeciesMetrics &entry = kSpecies[species];
        if (!entry.hasEarSockets)
        {
            continue; // ears (and horns) are part of that species' head mesh
        }
        static const ConnectorDefinition earConnectors[] = {
            {"ear.root", "ear.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const char *earTags[] = {"slot:ears"};
        static const MaterialSlotDefinition earSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.75f, 0.0f}};
        for (const char *side : {"_left", "_right"})
        {
        const std::string id      = std::string("ear_") + entry.id + side;
        const std::string display = std::string(entry.display) + " Ears";
        const std::string geometry = std::string("ear_") + entry.id;
        nameStorage.push_back(id);
        nameStorage.push_back(display);
        nameStorage.push_back(geometry);
        PartDefinition part{};
        part.id               = nameStorage[nameStorage.size() - 3].c_str();
        part.displayName      = nameStorage[nameStorage.size() - 2].c_str();
        part.geometryId       = nameStorage[nameStorage.size() - 1].c_str();
        part.connectors       = earConnectors;
        part.connectorCount   = 1;
        part.materialSlots    = earSlots;
        part.materialSlotCount = 1;
        part.tags             = earTags;
        part.tagCount         = 1;
        part.boundsCenter[1]  = 0.05f;
        part.boundsRadius     = 0.12f;
        parts.push_back(part);
        }
    }

    // ---- tails ------------------------------------------------------------------
    {
        static const ConnectorDefinition tailConnectors[] = {
            {"tail.root", "appendage.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding tailBindings[] = {
            scaleBinding("tail.length", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("tail.length", ParameterBindingTarget::PartScaleZ, 1.0f),
            scaleBinding("tail.thickness", ParameterBindingTarget::PartScaleX, 1.0f),
        };
        static const char *tailTags[] = {"slot:tail"};
        static const MaterialSlotDefinition tailSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.62f, 0.0f}};
        PartDefinition reptile{};
        reptile.id                    = "tail_reptile";
        reptile.displayName           = "Dragon Tail (Spiked)";
        reptile.geometryId            = "tail_reptile";
        reptile.connectors            = tailConnectors;
        reptile.connectorCount        = 1;
        reptile.parameterBindings     = tailBindings;
        reptile.parameterBindingCount = 3;
        reptile.materialSlots         = tailSlots;
        reptile.materialSlotCount     = 1;
        reptile.tags                  = tailTags;
        reptile.tagCount              = 1;
        reptile.boundsCenter[1]       = -0.22f;
        reptile.boundsCenter[2]       = -0.32f;
        reptile.boundsRadius          = 0.50f;
        parts.push_back(reptile);

        for (int species = 0; species < kSpeciesCount; ++species)
        {
            const metrics::SpeciesMetrics &entry = kSpecies[species];
            const std::string id      = std::string("tail_") + entry.id;
            const std::string display = std::string(entry.display) + " Tail";
            nameStorage.push_back(id);
            nameStorage.push_back(display);
            PartDefinition part{};
            part.id                    = nameStorage[nameStorage.size() - 2].c_str();
            part.displayName           = nameStorage[nameStorage.size() - 1].c_str();
            part.geometryId            = part.id;
            part.connectors            = tailConnectors;
            part.connectorCount        = 1;
            part.parameterBindings     = tailBindings;
            part.parameterBindingCount = 3;
            part.materialSlots         = tailSlots;
            part.materialSlotCount     = 1;
            part.tags                  = tailTags;
            part.tagCount              = 1;
            part.boundsCenter[1]       = -0.22f;
            part.boundsCenter[2]       = -0.16f;
            part.boundsRadius          = 0.45f;
            parts.push_back(part);
        }
    }

    // ---- wings -------------------------------------------------------------------
    {
        static const ConnectorDefinition wingConnectors[] = {
            {"wing.root", "wing.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding wingBindings[] = {
            scaleBinding("wings.span", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("wings.span", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("wings.span", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *wingTags[] = {"slot:wings"};
        static const MaterialSlotDefinition wingSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.65f, 0.0f}};
        for (const char *id : {"wing_dragon_left", "wing_dragon_right"})
        {
            PartDefinition part{};
            part.id                    = id;
            part.displayName           = "Dragon Wing";
            part.geometryId            = id; // the loader mirrors the right OBJ
            part.connectors            = wingConnectors;
            part.connectorCount        = 1;
            part.parameterBindings     = wingBindings;
            part.parameterBindingCount = 3;
            part.materialSlots         = wingSlots;
            part.materialSlotCount     = 1;
            part.tags                  = wingTags;
            part.tagCount              = 1;
            part.boundsCenter[1]       = 0.15f;
            part.boundsCenter[2]       = 0.38f;
            part.boundsRadius          = 0.95f;
            parts.push_back(part);
        }
    }

    // ---- taur lower bodies + whole quadrupeds ------------------------------------
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const metrics::SpeciesMetrics &entry = kSpecies[species];
        const std::string speciesId = entry.id;
        const std::string taurName  = std::string("quad_taur_") + speciesId;
        const std::string bodyName  = std::string("quad_body_") + speciesId;
        const std::string rigName   = std::string("quad_") + speciesId + "_core";
        const std::string taurDisplay = std::string(entry.display) + " Taur";
        const std::string bodyDisplay = std::string(entry.display) + " Body";
        nameStorage.push_back(taurName);
        nameStorage.push_back(bodyName);
        nameStorage.push_back(rigName);
        nameStorage.push_back(taurDisplay);
        nameStorage.push_back(bodyDisplay);

        static const ParameterBinding quadBindings[] = {
            scaleBinding("taur.scale", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("taur.scale", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("taur.scale", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *quadTags[] = {"slot:quadruped"};
        static const MaterialSlotDefinition quadSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.80f, 0.0f}};

        const std::size_t taurConnectorBase = connectorStorage.size();
        connectorStorage.push_back({"torso.mount", "body.mount", jointAnchor("torso_mount", glm::vec3(0.0f)),
            nullptr, 1});
        connectorStorage.push_back({"tail.mount", "appendage.mount", jointAnchor("tail_mount", glm::vec3(0.0f)),
            nullptr, 1});
        connectorStorage.push_back({"wings.left", "wing.mount", jointAnchor("withers_left", glm::vec3(0.0f)),
            nullptr, 1});
        connectorStorage.push_back({"wings.right", "wing.mount", jointAnchor("withers_right", glm::vec3(0.0f)),
            nullptr, 1});
        const std::size_t bodyConnectorBase = connectorStorage.size();
        connectorStorage.push_back({"wings.left", "wing.mount", jointAnchor("withers_left", glm::vec3(0.0f)),
            nullptr, 1});
        connectorStorage.push_back({"wings.right", "wing.mount", jointAnchor("withers_right", glm::vec3(0.0f)),
            nullptr, 1});
        const ConnectorDefinition *taurConnectors  = &connectorStorage[taurConnectorBase];
        const ConnectorDefinition *bodyConnectors  = &connectorStorage[bodyConnectorBase];

        PartDefinition taurPart{};
        taurPart.id                    = nameStorage[nameStorage.size() - 5].c_str();
        taurPart.displayName           = nameStorage[nameStorage.size() - 2].c_str();
        taurPart.geometryId            = taurPart.id;
        taurPart.rigId                 = nameStorage[nameStorage.size() - 3].c_str();
        taurPart.connectors            = taurConnectors;
        taurPart.connectorCount        = 4;
        taurPart.parameterBindings     = quadBindings;
        taurPart.parameterBindingCount = 3;
        taurPart.materialSlots         = quadSlots;
        taurPart.materialSlotCount     = 1;
        taurPart.tags                  = quadTags;
        taurPart.tagCount              = 1;
        taurPart.boundsCenter[1]       = (entry.groundY + entry.torsoMount.y) * 0.5f;
        taurPart.boundsCenter[2]       = entry.lengthZ * 0.25f;
        taurPart.boundsRadius          = entry.lengthZ * 0.62f + 0.30f;
        parts.push_back(taurPart);

        PartDefinition bodyPart = taurPart;
        bodyPart.id                    = nameStorage[nameStorage.size() - 4].c_str();
        bodyPart.displayName           = nameStorage[nameStorage.size() - 1].c_str();
        bodyPart.geometryId            = bodyPart.id;
        bodyPart.connectors            = bodyConnectors;
        bodyPart.connectorCount        = 2;
        bodyPart.boundsCenter[1]       = (entry.groundY + entry.withers.y) * 0.5f;
        bodyPart.boundsRadius          = entry.lengthZ * 0.72f + 0.30f;
        parts.push_back(bodyPart);
    }

    // ---- geometry (extracted OBJ downloads) --------------------------------
    const std::filesystem::path partDirectory = std::filesystem::path(assetRoot) / "character_parts";
    std::vector<std::pair<std::string, std::string>> objParts = {
        {"torso_ranger", "torso_ranger.obj"},
        {"torso_bare", "torso_bare.obj"},
        {"arm_bare_left", "arm_bare_left.obj"},
        {"arm_ranger_left", "arm_ranger_left.obj"},
        {"leg_bare_left", "leg_bare_left.obj"},
        {"leg_ranger_left", "leg_ranger_left.obj"},
        {"clothes_belt", "clothes_belt.obj"},
        {"clothes_bracer_left", "clothes_bracer_left.obj"},
        {"clothes_pauldron_left", "clothes_pauldron_left.obj"},
        {"clothes_hood", "clothes_hood.obj"},
        {"head_human", "head_human.obj"},
        {"head_dragon", "head_dragon.obj"},
        {"wing_dragon_left", "wing_dragon_left.obj"},
        {"tail_reptile", "tail_reptile.obj"},
    };
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const std::string id = kSpecies[species].id;
        objParts.push_back({"quad_taur_" + id, "quad_taur_" + id + ".obj"});
        objParts.push_back({"quad_body_" + id, "quad_body_" + id + ".obj"});
        objParts.push_back({"head_" + id, "head_" + id + ".obj"});
        objParts.push_back({"tail_" + id, "tail_" + id + ".obj"});
        if (kSpecies[species].hasEarSockets)
        {
            objParts.push_back({"ear_" + id, "ear_" + id + ".obj"});
        }
    }
    for (const auto &[geometryId, fileName] : objParts)
    {
        if (!loadGeometry(geometryId, partDirectory / fileName, renderer, rendererHandle))
        {
            return false;
        }
    }
    // Mirror the right-side limbs and ears from the left OBJs at load time:
    // the junction ring sits at the part origin, so mirroring across X is
    // exact at the interface (a rigid transform of the downloaded geometry).
    mirrorAcrossX("arm_bare_left", "arm_bare_right", renderer, rendererHandle);
    mirrorAcrossX("arm_ranger_left", "arm_ranger_right", renderer, rendererHandle);
    mirrorAcrossX("leg_bare_left", "leg_bare_right", renderer, rendererHandle);
    mirrorAcrossX("leg_ranger_left", "leg_ranger_right", renderer, rendererHandle);
    mirrorAcrossX("clothes_bracer_left", "clothes_bracer_right", renderer, rendererHandle);
    mirrorAcrossX("clothes_pauldron_left", "clothes_pauldron_right", renderer, rendererHandle);
    mirrorAcrossX("wing_dragon_left", "wing_dragon_right", renderer, rendererHandle);
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const std::string id = kSpecies[species].id;
        if (kSpecies[species].hasEarSockets)
        {
            mirrorAcrossX(("ear_" + id).c_str(), ("ear_" + id + "_right").c_str(), renderer, rendererHandle);
        }
    }

    // Fix up the registered triangle counts from the loaded geometry.
    PartDefinition resolvedParts[160];
    std::uint32_t   resolvedCount = 0;
    for (PartDefinition &part : parts)
    {
        resolvedParts[resolvedCount] = part;
        const auto found = geometries_.find(resolvedParts[resolvedCount].geometryId);
        if (found != geometries_.end())
        {
            resolvedParts[resolvedCount].triangleCount = found->second.triangles;
        }
        ++resolvedCount;
    }
    for (std::uint32_t index = 0; index < resolvedCount; ++index)
    {
        if (!character.registerPart(character.self, &resolvedParts[index], &error))
        {
            std::fprintf(stderr,
                "demo: part registration failed for '%s' (%u) geom=%s rig=%s conns=%u bind=%u tags=%u slots=%u\n",
                resolvedParts[index].id, static_cast<unsigned>(error),
                resolvedParts[index].geometryId ? resolvedParts[index].geometryId : "(null)",
                resolvedParts[index].rigId ? resolvedParts[index].rigId : "(null)",
                resolvedParts[index].connectorCount, resolvedParts[index].parameterBindingCount,
                resolvedParts[index].tagCount, resolvedParts[index].materialSlotCount);
            return false;
        }
    }

    // ---- assembly profiles: the three body plans ---------------------------
    {
        // humanoid: torso + head + one arm pair + one leg pair + tail
        // (the fox tail is the default; the tail variants swap live)
        static const ProfilePart humanoidParts[] = {
            {"torso", "torso_ranger"},
            {"head", "head_human"},
            {"arm_left", "arm_ranger_left"},
            {"arm_right", "arm_ranger_right"},
            {"leg_left", "leg_ranger_left"},
            {"leg_right", "leg_ranger_right"},
            {"tail", "tail_fox"},
            {"belt", "clothes_belt"},
            {"bracer_left", "clothes_bracer_left"},
            {"bracer_right", "clothes_bracer_right"},
            {"pauldron_left", "clothes_pauldron_left"},
            {"pauldron_right", "clothes_pauldron_right"},
        };
        static const ProfileConnection humanoidConnections[] = {
            {"torso", "neck.base", "head", "head.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "shoulder.left", "arm_left", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "shoulder.right", "arm_right", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "hip.left", "leg_left", "hip.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "hip.right", "leg_right", "hip.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "tail.base", "tail", "tail.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "belt.mount", "belt", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"arm_left", "bracer.mount", "bracer_left", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"arm_right", "bracer.mount", "bracer_right", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "pad.left", "pauldron_left", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "pad.right", "pauldron_right", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
        };
        const AssemblyProfileDefinition humanoidProfile{
            .id              = humanoidProfileId,
            .displayName     = "Humanoid Biped",
            .parts           = humanoidParts,
            .partCount       = static_cast<std::uint32_t>(std::size(humanoidParts)),
            .connections     = humanoidConnections,
            .connectionCount = static_cast<std::uint32_t>(std::size(humanoidConnections)),
        };
        if (!character.registerProfile(character.self, &humanoidProfile, &error))
        {
            std::fprintf(stderr, "demo: profile registration failed (%u)\n", static_cast<unsigned>(error));
            return false;
        }
    }
    {
        // taur: quadruped lower body + humanoid torso + head + arms; the tail
        // attaches to the lower body's own tail socket.
        static const ProfilePart taurParts[] = {
            {"body", "quad_taur_horse"},
            {"chest", "torso_ranger"},
            {"head", "head_human"},
            {"arm_left", "arm_ranger_left"},
            {"arm_right", "arm_ranger_right"},
            {"tail", "tail_horse"},
            {"belt", "clothes_belt"},
            {"bracer_left", "clothes_bracer_left"},
            {"bracer_right", "clothes_bracer_right"},
            {"pauldron_left", "clothes_pauldron_left"},
            {"pauldron_right", "clothes_pauldron_right"},
        };
        // The taur's hanging arms fan outward so they clear the animal's
        // barrel; rolls about the forward axis (left arm rolls negative).
        const glm::quat armRollLeft  = glm::angleAxis(glm::radians(22.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::quat armRollRight = glm::angleAxis(glm::radians(-22.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        const float taurArmLeft[4]  = {armRollLeft.x, armRollLeft.y, armRollLeft.z, armRollLeft.w};
        const float taurArmRight[4] = {armRollRight.x, armRollRight.y, armRollRight.z, armRollRight.w};
        static const ProfileConnection taurConnections[] = {
            {"body", "torso.mount", "chest", "body.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "neck.base", "head", "head.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "shoulder.left", "arm_left", "shoulder.root", {0.0f, 0.0f, 0.0f}, {taurArmLeft[0], taurArmLeft[1], taurArmLeft[2], taurArmLeft[3]}},
            {"chest", "shoulder.right", "arm_right", "shoulder.root", {0.0f, 0.0f, 0.0f}, {taurArmRight[0], taurArmRight[1], taurArmRight[2], taurArmRight[3]}},
            {"body", "tail.mount", "tail", "tail.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "belt.mount", "belt", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"arm_left", "bracer.mount", "bracer_left", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"arm_right", "bracer.mount", "bracer_right", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "pad.left", "pauldron_left", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "pad.right", "pauldron_right", "garment.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
        };
        const AssemblyProfileDefinition taurProfile{
            .id              = taurProfileId,
            .displayName     = "Taur",
            .parts           = taurParts,
            .partCount       = static_cast<std::uint32_t>(std::size(taurParts)),
            .connections     = taurConnections,
            .connectionCount = static_cast<std::uint32_t>(std::size(taurConnections)),
        };
        if (!character.registerProfile(character.self, &taurProfile, &error))
        {
            std::fprintf(stderr, "demo: profile registration failed (%u)\n", static_cast<unsigned>(error));
            return false;
        }
    }
    {
        // quadruped: the WHOLE animal - its own head, own limbs, own tail;
        // nothing is grafted onto it. Species swaps ride replacePart on the
        // "body" instance (each species carries its own rig).
        static const ProfilePart quadrupedParts[] = {
            {"body", "quad_body_horse"},
        };
        const AssemblyProfileDefinition quadrupedProfile{
            .id              = quadrupedProfileId,
            .displayName     = "Quadruped",
            .parts           = quadrupedParts,
            .partCount       = static_cast<std::uint32_t>(std::size(quadrupedParts)),
            .connections     = nullptr,
            .connectionCount = 0,
        };
        if (!character.registerProfile(character.self, &quadrupedProfile, &error))
        {
            std::fprintf(stderr, "demo: profile registration failed (%u)\n", static_cast<unsigned>(error));
            return false;
        }
    }

    // ---- render info table (UI grouping + materials) -----------------------
    std::vector<std::pair<std::string, std::string>> slotTable = {
        {"torso_ranger", "torso"},
        {"torso_bare", "torso"},
        {"arm_bare_left", "arms"},
        {"arm_bare_right", "arms"},
        {"arm_ranger_left", "arms"},
        {"arm_ranger_right", "arms"},
        {"leg_bare_left", "legs"},
        {"leg_bare_right", "legs"},
        {"leg_ranger_left", "legs"},
        {"leg_ranger_right", "legs"},
        {"clothes_belt", "garment"},
        {"clothes_bracer_left", "garment"},
        {"clothes_bracer_right", "garment"},
        {"clothes_pauldron_left", "garment"},
        {"clothes_pauldron_right", "garment"},
        {"clothes_hood", "garment"},
        {"head_human", "head"},
        {"head_dragon", "head"},
        {"wing_dragon_left", "wings"},
        {"wing_dragon_right", "wings"},
        {"tail_reptile", "tail"},
    };
    for (int species = 0; species < kSpeciesCount; ++species)
    {
        const std::string id = kSpecies[species].id;
        slotTable.push_back({"quad_taur_" + id, "quadruped"});
        slotTable.push_back({"quad_body_" + id, "quadruped"});
        slotTable.push_back({"head_" + id, "head"});
        slotTable.push_back({"tail_" + id, "tail"});
        if (kSpecies[species].hasEarSockets)
        {
            slotTable.push_back({"ear_" + id + "_left", "ears"});
            slotTable.push_back({"ear_" + id + "_right", "ears"});
        }
    }
    for (const PartDefinition &part : parts)
    {
        PartRenderInfo info;
        info.displayName = part.displayName;
        info.geometryId  = part.geometryId;
        for (const auto &[partId, slot] : slotTable)
        {
            if (partId == part.id)
            {
                info.slot = slot;
                break;
            }
        }
        // Vertex colors carry the authored look for every part (baked by the
        // extractor from the source textures/factors), so the draw-call base
        // color stays white and only roughness/metallic come from the slot
        // metadata.
        const MaterialSlotDefinition &slot = part.materialSlots[0];
        info.material.baseColor[0]         = 1.0f;
        info.material.baseColor[1]         = 1.0f;
        info.material.baseColor[2]         = 1.0f;
        info.material.roughnessFactor      = slot.roughness;
        info.material.metallicFactor       = slot.metallic;
        partRenderInfo_[part.id]           = std::move(info);
        partsBySlot_[partRenderInfo_[part.id].slot].push_back(part.id);
    }

    // Per-part facts for the assembly helpers.
    legFootY_["leg_bare_left"]     = metrics::kBareLegFootY;
    legFootY_["leg_bare_right"]    = metrics::kBareLegFootY;
    legFootY_["leg_ranger_left"]   = metrics::kRangerLegFootY;
    legFootY_["leg_ranger_right"]  = metrics::kRangerLegFootY;

    renderCapability_ = renderer;
    rendererHandle_   = rendererHandle;
    return true;
}

// ---- deformation entry points -----------------------------------------------

void CharacterContent::applyHeadMorphs(float muzzleWeight, float eyesWidthWeight)
{
    for (auto &[geometryId, entry] : geometries_)
    {
        const bool hasMuzzle = std::any_of(entry.morphs.begin(), entry.morphs.end(),
            [](const PartMorph &morph) { return morph.name == "muzzle"; });
        if (!hasMuzzle)
        {
            continue;
        }
        // Apply the morphs to a copy of the rest pose and re-upload; the
        // rest pose itself stays pristine so later changes re-derive.
        MeshData working;
        working.vertices = entry.rest.vertices;
        working.indices  = entry.rest.indices;
        for (const PartMorph &morph : entry.morphs)
        {
            if (morph.deltas.size() != working.vertices.size())
            {
                continue;
            }
            const float weight = morph.name == "muzzle" ? muzzleWeight : eyesWidthWeight;
            for (std::size_t index = 0; index < working.vertices.size(); ++index)
            {
                working.vertices[index].position += morph.deltas[index] * weight;
            }
        }
        const engine::renderer::MeshDescription description = working.description();
        engine::renderer::RenderError error = engine::renderer::RenderError::None;
        if (entry.handle != engine::renderer::invalidMesh)
        {
            renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, entry.handle);
            entry.handle = engine::renderer::invalidMesh;
        }
        entry.handle = renderCapability_.createMesh(
            renderCapability_.self, rendererHandle_, &description, &error);
    }
}

void CharacterContent::applyMarkings(int markingMode, const float furTint[3])
{
    // Fur markings (the fandom's socks/gloves/tail-tip/muzzle patterns) are
    // vertex-colour edits re-derived from the rest colours whenever the mode
    // changes; the draw-time fur coat tint still multiplies on top.
    if (markingMode < 0 || markingMode > 3)
    {
        markingMode = 0;
    }
    const glm::vec3 markingColor(furTint[0], furTint[1], furTint[2]);
    for (auto &[geometryId, entry] : geometries_)
    {
        const std::string &id = geometryId;
        const bool isLeg      = id.rfind("leg_bare", 0) == 0;
        const bool isArm      = id.rfind("arm_bare", 0) == 0;
        const bool isTail     = id.rfind("tail_", 0) == 0 && id != "tail_reptile";
        const bool isHead     = id.rfind("head_", 0) == 0 && id != "head_human" && id != "head_dragon";
        if (!isLeg && !isArm && !isTail && !isHead)
        {
            continue;
        }
        MeshData working;
        working.vertices = entry.rest.vertices;
        working.indices  = entry.rest.indices;
        glm::vec3 boundsMin(0.0f);
        glm::vec3 boundsMax(0.0f);
        for (const Vertex &vertex : working.vertices)
        {
            boundsMin = glm::min(boundsMin, vertex.position);
            boundsMax = glm::max(boundsMax, vertex.position);
        }
        for (Vertex &vertex : working.vertices)
        {
            float blend = 0.0f;
            if ((isLeg || isArm) && markingMode >= 1)
            {
                // socks and gloves: the distal third of the limb
                const float length = boundsMax.y - boundsMin.y;
                const float t = (vertex.position.y - boundsMin.y) / std::max(length, 1.0e-5f);
                if (t < 0.33f)
                {
                    blend = glm::smoothstep(0.38f, 0.20f, t);
                }
            }
            else if (isTail && (markingMode == 2 || markingMode == 3))
            {
                // tail tip: the quarter farthest from the root (origin)
                const float extent = glm::length(boundsMax - boundsMin);
                const float d = glm::length(vertex.position) / std::max(extent, 1.0e-5f);
                if (d > 0.55f)
                {
                    blend = glm::smoothstep(0.55f, 0.80f, d);
                }
            }
            else if (isHead && (markingMode == 2 || markingMode == 3))
            {
                // muzzle and lower jaw: the forward lower face
                const float zRange = boundsMax.z - boundsMin.z;
                const float yRange = boundsMax.y - boundsMin.y;
                const float z = (vertex.position.z - boundsMin.z) / std::max(zRange, 1.0e-5f);
                const float y = (vertex.position.y - boundsMin.y) / std::max(yRange, 1.0e-5f);
                if (z > 0.55f && y < 0.55f)
                {
                    blend = 0.65f;
                }
            }
            if (blend > 0.0f)
            {
                vertex.color = glm::mix(vertex.color, markingColor, blend);
            }
        }
        const engine::renderer::MeshDescription description = working.description();
        engine::renderer::RenderError error = engine::renderer::RenderError::None;
        if (entry.handle != engine::renderer::invalidMesh)
        {
            renderCapability_.destroyMesh(renderCapability_.self, rendererHandle_, entry.handle);
            entry.handle = engine::renderer::invalidMesh;
        }
        entry.handle = renderCapability_.createMesh(
            renderCapability_.self, rendererHandle_, &description, &error);
    }
}

bool CharacterContent::geometryBounds(const std::string &geometryId, float outCenter[3], float *outRadius) const
{
    const auto found = geometries_.find(geometryId);
    if (found == geometries_.end() || found->second.rest.vertices.empty())
    {
        return false;
    }
    glm::vec3 boundsMin(std::numeric_limits<float>::max());
    glm::vec3 boundsMax(std::numeric_limits<float>::lowest());
    for (const Vertex &vertex : found->second.rest.vertices)
    {
        boundsMin = glm::min(boundsMin, vertex.position);
        boundsMax = glm::max(boundsMax, vertex.position);
    }
    const glm::vec3 center = (boundsMin + boundsMax) * 0.5f;
    outCenter[0] = center.x;
    outCenter[1] = center.y;
    outCenter[2] = center.z;
    *outRadius   = glm::length(boundsMax - boundsMin) * 0.5f;
    return true;
}

float CharacterContent::legFootYForPart(const std::string &partId) const
{
    const auto found = legFootY_.find(partId);
    return found != legFootY_.end() ? found->second : -0.9752f;
}

bool CharacterContent::partHasMorph(const std::string &geometryId, const char *morphName) const
{
    const auto found = geometries_.find(geometryId);
    if (found == geometries_.end())
    {
        return false;
    }
    return std::any_of(found->second.morphs.begin(), found->second.morphs.end(),
        [morphName](const PartMorph &morph) { return morph.name == morphName; });
}

engine::renderer::TextureHandle CharacterContent::previewTextureForPart(const std::string &partId) const
{
    const auto found = previewTextures_.find(partId);
    return found != previewTextures_.end() ? found->second : engine::renderer::invalidTexture;
}

bool CharacterContent::loadPreviewTextures(const std::string &assetRoot)
{
    const std::filesystem::path previewDirectory = std::filesystem::path(assetRoot) / "previews";
    if (!std::filesystem::exists(previewDirectory) || renderCapability_.createTextureFromFile == nullptr)
    {
        return false;
    }
    for (const auto &[partId, info] : partRenderInfo_)
    {
        const std::filesystem::path image = previewDirectory / (partId + ".png");
        if (!std::filesystem::exists(image))
        {
            continue;
        }
        engine::renderer::RenderError error = engine::renderer::RenderError::None;
        const engine::renderer::TextureHandle texture = renderCapability_.createTextureFromFile(
            renderCapability_.self, rendererHandle_, image.string().c_str(), &error);
        if (texture != engine::renderer::invalidTexture)
        {
            previewTextures_[partId] = texture;
        }
    }
    return !previewTextures_.empty();
}

void CharacterContent::destroy(engine::character::CharacterCapability &character,
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle)
{
    (void)character;
    for (auto &[geometryId, entry] : geometries_)
    {
        if (entry.handle != engine::renderer::invalidMesh)
        {
            renderer.destroyMesh(renderer.self, rendererHandle, entry.handle);
        }
    }
    geometries_.clear();
    partRenderInfo_.clear();
    partsBySlot_.clear();
    previewTextures_.clear();
    legFootY_.clear();
}

engine::renderer::MeshHandle CharacterContent::meshForGeometry(const std::string &geometryId) const
{
    const auto found = geometries_.find(geometryId);
    return found != geometries_.end() ? found->second.handle : engine::renderer::invalidMesh;
}

const PartRenderInfo *CharacterContent::renderInfoForPart(const std::string &partId) const
{
    const auto found = partRenderInfo_.find(partId);
    return found != partRenderInfo_.end() ? &found->second : nullptr;
}

const std::vector<std::string> &CharacterContent::partsInSlot(const std::string &slot) const
{
    static const std::vector<std::string> empty;
    const auto                            found = partsBySlot_.find(slot);
    return found != partsBySlot_.end() ? found->second : empty;
}

std::uint32_t CharacterContent::triangleCountForGeometry(const std::string &geometryId) const
{
    const auto found = geometries_.find(geometryId);
    return found != geometries_.end() ? found->second.triangles : 0;
}

} // namespace demo
