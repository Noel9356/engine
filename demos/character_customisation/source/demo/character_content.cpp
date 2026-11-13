// character_content.cpp - body parts for the customisation demo.
//
// Revision 10 mandate: "Don't generate models (or parts of models) by
// yourself. Downloading is allowed." Every part mesh below is a VERBATIM
// piece of a downloaded, rigged CC0 model, extracted (rigid transform /
// uniform scale / triangle-subset selection / winding repair only) by
// scripts/extract_parts_rev10.py and loaded from assets/character_parts/*.obj:
//   - Human parts: Quaternius Ultimate Base Characters "Ranger" male,
//     ARTIST-SPLIT modular meshes (Body/Belt_1/Belt_2, Arms/Bracer/Pauldron,
//     Legs/Feet_Boots, Head/Eyes/Eyebrows) on one UE-style skeleton,
//     mirrored from the ValTrainer Godot project. Arms hang via the model's
//     own upperarm joints; the hood node is simply not part of any part.
//   - Dragon head: Quaternius Ultimate Monsters "Dragon Evolved", triangles
//     dominated by its own Neck/Head joints (the head graft carries its
//     natural neck). Rigidly rescaled this revision to human-head
//     proportion (0.36 m over neck+skull; the rev-9 pack scale measured
//     ~0.50 m and read oversized).
//   - Wings: the basic Flying "Dragon" Wing1-4 chains in bind pose, rigid
//     0.85 scale. Loader mirrors the left OBJ for the right side.
//   - Tails: OpenGameArt CC0 "Cethiel's Dragon 3D" tail1-3 joints in bind
//     pose (spiked dragon tail; kept as the explicit dragon option), plus
//     eleven species tails from the AnimaSim GLB animals (fox/shiba/husky/
//     wolf/donkey/cow/deer/bull/alpaca/horse/stag). The naga serpent lower
//     body is the Cethiel tail re-centred on its measured root ring, pitched
//     horizontal and ring-normalised (rev-9 shipped it off-centre and steep,
//     which read as two disconnected halves).
//   - Quadruped bodies + centaur hindquarters: AnimaSim GLB animals
//     (fox/shiba/husky/wolf/donkey/cow/deer/bull/alpaca/horse/stag) WHOLE
//     for the quadruped plan; the centaur variant is the same animal minus
//     its neck/head joints. Each species carries its own rig anchored at its
//     MEASURED stump opening, so every species grafts the human torso at the
//     same band and none of them floats (rev-9 registered one horse rig for
//     all four species).
//   - Species heads: the same animals' Neck/Head/Ear joint triangles, stood
//     upright and scaled to the humanoid graft band - furry-fandom head
//     options for every plan.
//
// No cap/skirt/seal geometry, no vertex reshaping, no baked morphs, no
// procedural recoloring anywhere: junctions close by overlap (artist part
// splits and socket burial), and sizes ride rigid part-scale bindings.

#include "character_content.hpp"
#include "character_metrics.gen.hpp"

#include "geometry.hpp"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
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

constexpr float pi = glm::pi<float>();

// Loads a colored OBJ produced by scripts/extract_parts_verbatim.py:
// "v x y z r g b" vertices, "vn" normals, and extended
// "f a//a b//b c//c" faces. No morph blocks: every part is verbatim
// downloaded geometry and all placement is rigid. Returns false when the
// file is missing or holds no triangles.
bool loadPartObj(const std::filesystem::path &path, MeshData &outMesh)
{
    std::FILE *file = std::fopen(path.string().c_str(), "rb");
    if (file == nullptr)
    {
        return false;
    }
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> colors;
    std::vector<glm::vec3> normals;
    char                   line[512];
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
                normals.emplace_back(x, y, z);
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
            const std::uint32_t base = static_cast<std::uint32_t>(outMesh.vertices.size());
            const unsigned indices[3] = {a, b, c};
            const unsigned normalIndices[3] = {an, bn, cn};
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const unsigned index = indices[corner];
                Vertex vertex;
                vertex.position = positions[index - 1];
                vertex.color    = colors[index - 1];
                const unsigned normalIndex = shaded ? normalIndices[corner] : index;
                vertex.normal   = normals.empty()
                                    ? glm::vec3(0.0f, 1.0f, 0.0f)
                                    : normals[std::min<std::size_t>(normalIndex > 0 ? normalIndex - 1 : 0u,
                                          normals.size() - 1)];
                vertex.uv       = {0.0f, 0.0f};
                outMesh.vertices.push_back(vertex);
            }
            outMesh.indices.insert(outMesh.indices.end(), {base, base + 1, base + 2});
        }
    }
    std::fclose(file);
    return !outMesh.indices.empty();
}

// ---- content registration helpers ------------------------------------------

ConnectorAnchorDefinition partLocalAnchor(float x, float y, float z, float yawDegrees = 0.0f, float rollDegrees = 0.0f)
{
    ConnectorAnchorDefinition anchor{};
    anchor.space          = AnchorSpace::PartLocal;
    anchor.translation[0] = x;
    anchor.translation[1] = y;
    anchor.translation[2] = z;
    const glm::quat yaw   = glm::angleAxis(glm::radians(yawDegrees), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat roll  = glm::angleAxis(glm::radians(rollDegrees), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::quat rotation = yaw * roll;
    anchor.rotation[0]    = rotation.x;
    anchor.rotation[1]    = rotation.y;
    anchor.rotation[2]    = rotation.z;
    anchor.rotation[3]    = rotation.w;
    return anchor;
}

ConnectorAnchorDefinition jointAnchor(
    const char *jointSemantic, float x, float y, float z, float yawDegrees = 0.0f, float rollDegrees = 0.0f)
{
    ConnectorAnchorDefinition anchor = partLocalAnchor(x, y, z, yawDegrees, rollDegrees);
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
    if (!loadPartObj(path, entry.rest))
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

// ---- registration -----------------------------------------------------------

bool CharacterContent::create(engine::character::CharacterCapability &character,
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle,
    const std::string &assetRoot)
{
    using namespace engine::character;

    CharacterError error = CharacterError::None;
    const auto    &metrics = kCharacterMetrics;

    // Generated ids/display names for the species parts and rigs;
    // definitions borrow these pointers until the module copies them during
    // the registerRig/registerPart calls in this function. The reservation
    // covers every generated string (2 per species head/tail, 5 per species
    // quad pair, 1 rig id) - a reallocation here would dangle every pointer
    // handed out before it.
    std::vector<std::string> nameStorage;
    nameStorage.reserve(256);

    // ---- global parameters (the registry; assets map them locally) --------
    // body.plan selects the assembly profile (0 humanoid, 1 centaur, 2 naga,
    // 3 quadruped); the screen rebuilds the assembly when it changes.
    const ParameterDefinition parameters[] = {
        {"body.plan", "Body Plan", 0.0f, 3.0f, 0.0f},
        {"body.height", "Body Height", 0.75f, 1.35f, 1.0f},
        {"body.width", "Body Width", 0.70f, 1.45f, 1.0f},
        {"body.muscle_mass", "Muscle Mass", 0.70f, 1.55f, 1.0f},
        {"head.size", "Head Size", 0.75f, 1.35f, 1.0f},
        {"limbs.length", "Limb Length", 0.75f, 1.35f, 1.0f},
        {"limbs.thickness", "Limb Thickness", 0.65f, 1.55f, 1.0f},
        {"wings.span", "Wingspan", 0.55f, 1.45f, 1.0f},
        {"tail.length", "Tail Length", 0.55f, 1.50f, 1.0f},
        {"tail.thickness", "Tail Thickness", 0.55f, 1.65f, 1.0f},
        {"skin.tone", "Skin Tone", 0.0f, 4.0f, 0.0f},
        {"fur.coat", "Fur Coat", 0.0f, 5.0f, 0.0f},
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

    // ---- rig fragments (independent, never merged) -------------------------
    // The torso rig is flat (every joint a pelvis child): it exists to place
    // parameter-driven anchors, not to animate. All joint positions are the
    // measured source-skeleton positions in torso-local space (revision 9
    // metrics). The module scales joint-anchored connector positions by part
    // scale automatically.
    const RigJointDefinition torsoJoints[] = {
        {"pelvis", -1, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"chest", 0, {0.0f, 0.400f, -0.010f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"neck_base", 0, {0.0f, metrics.headNeckY, metrics.headNeckZ}, {0.0f, 0.0f, 0.0f, 1.0f},
            {1.0f, 1.0f, 1.0f}},
        {"shoulder_left", 0, {-metrics.shoulderX, metrics.shoulderY, metrics.shoulderZ},
            {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"shoulder_right", 0, {metrics.shoulderX, metrics.shoulderY, metrics.shoulderZ},
            {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"arm2_left", 0, {-0.225f, 0.310f, -0.120f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"arm2_right", 0, {0.225f, 0.310f, -0.120f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"arm3_left", 0, {-0.195f, 0.105f, -0.100f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"arm3_right", 0, {0.195f, 0.105f, -0.100f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"hip_left", 0, {-metrics.hipX, metrics.hipY, metrics.hipZ}, {0.0f, 0.0f, 0.0f, 1.0f},
            {1.0f, 1.0f, 1.0f}},
        {"hip_right", 0, {metrics.hipX, metrics.hipY, metrics.hipZ}, {0.0f, 0.0f, 0.0f, 1.0f},
            {1.0f, 1.0f, 1.0f}},
        {"wing1", 0, {0.0f, 0.560f, -0.170f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"wing2", 0, {0.0f, 0.420f, -0.205f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"wing3", 0, {0.0f, 0.285f, -0.225f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
        {"tail_socket", 0, {0.0f, 0.065f, -0.105f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
    };
    const RigDefinition torsoRig{"torso_core", torsoJoints, 15};
    if (!character.registerRig(character.self, &torsoRig, &error))
    {
        std::fprintf(stderr, "demo: rig registration failed for 'torso_core' (%u)\n", static_cast<unsigned>(error));
        return false;
    }

    // Quadruped rigs: one per species (rev-9 registered a single horse rig
    // for every species, which left wolf/deer/bull centaur torsos floating
    // above their smaller bodies). The body part origin is the animal's Body
    // joint; torso.mount sits at the species' MEASURED stump opening
    // (kQuadMountY/kQuadMountZ), sunk 0.10 m so the human torso's hip ring
    // buries over it; withers1/2 host the optional wing pairs.
    std::vector<RigJointDefinition> rigJointStorage;
    rigJointStorage.reserve(static_cast<std::size_t>(kQuadSpeciesCount) * 4);
    for (int species = 0; species < kQuadSpeciesCount; ++species)
    {
        const float mountY = kQuadMountY[species];
        const float mountZ = kQuadMountZ[species];
        const float withersY = kQuadWithersY[species];
        const std::string rigId = std::string("quad_") + kQuadSpeciesIds[species] + "_core";
        nameStorage.push_back(rigId);
        const std::size_t firstJoint = rigJointStorage.size();
        // RigJointDefinition defaults are already the identity rotation and
        // unit scale; only the semantics, parents, and mount translations
        // need setting.
        RigJointDefinition bodyRoot{};
        bodyRoot.semantic     = "body_root";
        bodyRoot.parentIndex  = -1;
        rigJointStorage.push_back(bodyRoot);
        RigJointDefinition torsoMount{};
        torsoMount.semantic        = "torso_mount";
        torsoMount.parentIndex     = 0;
        torsoMount.translation[0]  = 0.0f;
        torsoMount.translation[1]  = mountY - 0.10f;
        torsoMount.translation[2]  = mountZ;
        rigJointStorage.push_back(torsoMount);
        RigJointDefinition withers1{};
        withers1.semantic          = "withers1";
        withers1.parentIndex       = 0;
        withers1.translation[0]    = 0.0f;
        withers1.translation[1]    = withersY;
        withers1.translation[2]    = mountZ - 0.25f;
        rigJointStorage.push_back(withers1);
        RigJointDefinition withers2{};
        withers2.semantic          = "withers2";
        withers2.parentIndex       = 0;
        withers2.translation[0]    = 0.0f;
        withers2.translation[1]    = withersY - 0.12f;
        withers2.translation[2]    = mountZ - 0.45f;
        rigJointStorage.push_back(withers2);
        const RigDefinition speciesRig{nameStorage.back().c_str(), &rigJointStorage[firstJoint], 4};
        if (!character.registerRig(character.self, &speciesRig, &error))
        {
            std::fprintf(stderr, "demo: rig registration failed for '%s' (%u)\n", nameStorage.back().c_str(),
                static_cast<unsigned>(error));
            return false;
        }
    }

    // ---- body parts ---------------------------------------------------------

    // ---- parts table --------------------------------------------------------
    // Each entry: id, display, geometry, rig, bounds (center y/z + radius).
    struct PartSpec
    {
        const char *id;
        const char *display;
        const char *geometry;
        const char *rig;
        float       boundsY;
        float       boundsZ;
        float       boundsRadius;
    };
    const PartSpec partSpecs[] = {
        {"torso_ranger", "Ranger Body", "torso_ranger", "torso_core", 0.33f, -0.02f, 0.55f},
        {"head_human", "Human Head", "head_human", nullptr, 0.15f, -0.01f, 0.25f},
        {"head_dragon", "Dragon Head", "head_dragon", nullptr, 0.12f, 0.06f, 0.22f},
        {"arm_human_left", "Ranger Arm", "arm_human_left", nullptr, -0.35f, 0.0f, 0.45f},
        {"leg_human_left", "Ranger Leg", "leg_human_left", nullptr, -0.50f, 0.0f, 0.60f},
        {"wing_dragon_left", "Dragon Wing", "wing_dragon_left", nullptr, -0.10f, -0.30f, 1.15f},
        {"tail_reptile", "Dragon Tail (Spiked)", "tail_reptile", nullptr, -0.22f, -0.32f, 0.50f},
        {"tail_naga", "Serpent Body", "tail_naga", nullptr, -0.18f, -0.40f, 0.75f},
    };
    // (the species parts are generated from kQuadSpeciesIds below; bounds
    // live in the per-part blocks)

    // Connectors per part kind (built inline below).
    std::vector<PartDefinition> parts;
    parts.reserve(96);

    // Torso connectors + bindings (the documented block above, materialized).
    {
        static const ConnectorDefinition torsoConnectors[] = {
            {"neck.base", "head.mount", jointAnchor("neck_base", 0.0f, 0.0f, 0.0f), nullptr, 3},
            {"shoulder.left", "arm.mount", jointAnchor("shoulder_left", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"shoulder.right", "arm.mount", jointAnchor("shoulder_right", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"arm2.left", "arm.mount", jointAnchor("arm2_left", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"arm2.right", "arm.mount", jointAnchor("arm2_right", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"arm3.left", "arm.mount", jointAnchor("arm3_left", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"arm3.right", "arm.mount", jointAnchor("arm3_right", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"hip.left", "leg.mount", jointAnchor("hip_left", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"hip.right", "leg.mount", jointAnchor("hip_right", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"wings1.left", "wing.mount", jointAnchor("wing1", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings1.right", "wing.mount", jointAnchor("wing1", 0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings2.left", "wing.mount", jointAnchor("wing2", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings2.right", "wing.mount", jointAnchor("wing2", 0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings3.left", "wing.mount", jointAnchor("wing3", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings3.right", "wing.mount", jointAnchor("wing3", 0.020f, 0.0f, 0.0f), nullptr, 1},
            {"tail.base", "appendage.mount", jointAnchor("tail_socket", 0.0f, 0.0f, 0.0f), nullptr, 1},
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
        PartDefinition torsoPart{};
        torsoPart.id                    = "torso_ranger";
        torsoPart.displayName           = "Ranger Body";
        torsoPart.geometryId            = "torso_ranger";
        torsoPart.rigId                 = "torso_core";
        torsoPart.connectors            = torsoConnectors;
        torsoPart.connectorCount        = static_cast<std::uint32_t>(std::size(torsoConnectors));
        torsoPart.parameterBindings     = torsoBindings;
        torsoPart.parameterBindingCount = static_cast<std::uint32_t>(std::size(torsoBindings));
        torsoPart.materialSlots         = torsoSlots;
        torsoPart.materialSlotCount     = 1;
        torsoPart.tags                  = torsoTags;
        torsoPart.tagCount              = 2;
        torsoPart.boundsCenter[1]       = 0.33f;
        torsoPart.boundsCenter[2]       = -0.02f;
        torsoPart.boundsRadius          = 0.55f;
        parts.push_back(torsoPart);
    }

    // Human head: whole downloaded Head/Eyes/Eyebrows nodes with the artist
    // neck-skin boundary; grafts onto the torso by the pack's own overlap.
    // head.size is a rigid uniform part scale about the neck-base origin.
    {
        static const ConnectorDefinition headConnectors[] = {
            {"head.root", "head.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
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
        part.rigId                 = nullptr;
        part.connectors            = headConnectors;
        part.connectorCount        = 1;
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

    // Dragon head (Dragon Evolved Neck+Head joints, rigidly rescaled this
    // revision to 0.36 m over neck+skull): the graft carries the dragon's
    // natural neck, whose base ring buries inside the torso collar; size
    // rides rigid part scale.
    {
        static const ConnectorDefinition headConnectors[] = {
            {"head.root", "head.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
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
        part.rigId                 = nullptr;
        part.connectors            = headConnectors;
        part.connectorCount        = 1;
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

    // Species heads (furry-fandom grafts): the animals' own Neck/Head/Ear
    // triangles, stood upright and scaled to the graft band by the extractor.
    // The part origin is the neck-base ring, exactly like the human and
    // dragon heads, so every plan can wear them. PartDefinition borrows the
    // name pointers until registerPart copies them below, so the generated
    // strings live in the function-scope name storage.
    for (int species = 0; species < kQuadSpeciesCount; ++species)
    {
        static const ConnectorDefinition speciesHeadConnectors[] = {
            {"head.root", "head.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding speciesHeadBindings[] = {
            scaleBinding("head.size", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("head.size", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *speciesHeadTags[] = {"slot:head"};
        static const MaterialSlotDefinition speciesHeadSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.75f, 0.0f}};
        const std::string id = std::string("head_") + kQuadSpeciesIds[species];
        const std::string display = std::string(kQuadSpeciesNames[species]) + " Head";
        nameStorage.push_back(id);
        nameStorage.push_back(display);
        PartDefinition part{};
        part.id                    = nameStorage[nameStorage.size() - 2].c_str();
        part.displayName           = nameStorage[nameStorage.size() - 1].c_str();
        part.geometryId            = part.id;
        part.rigId                 = nullptr;
        part.connectors            = speciesHeadConnectors;
        part.connectorCount        = 1;
        part.parameterBindings     = speciesHeadBindings;
        part.parameterBindingCount = 3;
        part.materialSlots         = speciesHeadSlots;
        part.materialSlotCount     = 1;
        part.tags                  = speciesHeadTags;
        part.tagCount              = 1;
        part.boundsCenter[1]       = 0.15f;
        part.boundsCenter[2]       = 0.02f;
        part.boundsRadius          = 0.25f;
        parts.push_back(part);
    }

    // Limbs (left pieces; the loader mirrors the right side across X). Their
    // only ring is at the origin, so part scale keeps them on their anchors.
    {
        static const ConnectorDefinition limbConnectors[] = {
            {"shoulder.root", "arm.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding armBindings[] = {
            scaleBinding("limbs.length", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *armTags[] = {"slot:arms", "skin"};
        static const MaterialSlotDefinition limbSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.72f, 0.0f}};
        for (const char *id : {"arm_human_left", "arm_human_right"})
        {
            PartDefinition part{};
            part.id                    = id;
            part.displayName           = "Ranger Arm";
            part.geometryId            = id; // the loader mirrors the right OBJ
            part.connectors            = limbConnectors;
            part.connectorCount        = 1;
            part.parameterBindings     = armBindings;
            part.parameterBindingCount = 3;
            part.materialSlots         = limbSlots;
            part.materialSlotCount     = 1;
            part.tags                  = armTags;
            part.tagCount              = 2;
            part.boundsCenter[1]       = -0.35f;
            part.boundsRadius          = 0.45f;
            parts.push_back(part);
        }
        static const ConnectorDefinition legConnectors[] = {
            {"hip.root", "leg.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding legBindings[] = {
            scaleBinding("limbs.length", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("limbs.thickness", ParameterBindingTarget::PartScaleZ, 1.0f),
        };
        static const char *legTags[] = {"slot:legs"};
        for (const char *id : {"leg_human_left", "leg_human_right"})
        {
            PartDefinition part{};
            part.id                    = id;
            part.displayName           = "Ranger Leg";
            part.geometryId            = id; // the loader mirrors the right OBJ
            part.connectors            = legConnectors;
            part.connectorCount        = 1;
            part.parameterBindings     = legBindings;
            part.parameterBindingCount = 3;
            part.materialSlots         = limbSlots;
            part.materialSlotCount     = 1;
            part.tags                  = legTags;
            part.tagCount              = 1;
            part.boundsCenter[1]       = -0.50f;
            part.boundsRadius          = 0.60f;
            parts.push_back(part);
        }
    }

    // Wings (left piece mirrored to the right at load): origin at the wing
    // root ring; flap/scale ride the connection offsets and part scale.
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

    // Tails: the OpenGameArt CC0 Cethiel dragon tail1-3 (spiked dragon
    // tail, re-centred on its root ring this revision) plus the eleven
    // AnimaSim species tails (bushy canine tails, bovine tassels, deer
    // stubs...), all with rest droop baked by the extractor so the
    // attachment stays identity. Length/thickness scale about the root.
    // The fox tail is the default; the Cethiel piece stays as the explicit
    // dragon option (rev-9's "tail is very weird" complaint).
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

        for (int species = 0; species < kQuadSpeciesCount; ++species)
        {
            const std::string id = std::string("tail_") + kQuadSpeciesIds[species];
            const std::string display = std::string(kQuadSpeciesNames[species]) + " Tail";
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

    // Naga lower body: the same downloaded Cethiel tail, re-centred on its
    // measured root ring, pitched horizontal and ring-normalised by the
    // extractor - it IS the plan's lower half, not an accessory. The profile
    // connection buries the ring inside the torso hip opening (rev-9 shipped
    // it off-centre with a steep tube axis, which read as two disconnected
    // halves and a jagged wall).
    {
        static const ConnectorDefinition nagaConnectors[] = {
            {"tail.root", "appendage.root", partLocalAnchor(0.0f, 0.0f, 0.0f), nullptr, 1},
        };
        static const char *nagaTags[] = {"slot:naga"};
        static const MaterialSlotDefinition nagaSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.62f, 0.0f}};
        PartDefinition part{};
        part.id                    = "tail_naga";
        part.displayName           = "Serpent Body";
        part.geometryId            = "tail_naga";
        part.connectors            = nagaConnectors;
        part.connectorCount        = 1;
        part.materialSlots         = nagaSlots;
        part.materialSlotCount     = 1;
        part.tags                  = nagaTags;
        part.tagCount              = 1;
        part.boundsCenter[1]       = -0.18f;
        part.boundsCenter[2]       = -0.40f;
        part.boundsRadius          = 0.75f;
        parts.push_back(part);
    }

    // Quadruped bodies + centaur hindquarters (11 species each). The WHOLE
    // animal is the quadruped body (own head, own tail - nothing grafted).
    // The centaur variant is the same animal minus its neck/head joints,
    // with a torso.mount connector over the neck-stump opening, on the
    // species' own rig. Name storage outlives the loop: PartDefinition
    // borrows the pointers and the module copies them only during
    // registration below.
    for (int species = 0; species < kQuadSpeciesCount; ++species)
    {
        const std::string speciesId = kQuadSpeciesIds[species];
        const std::string bodyName = std::string("quad_body_") + speciesId;
        const std::string centaurName = std::string("quad_centaur_") + speciesId;
        const std::string rigName = std::string("quad_") + speciesId + "_core";
        const std::string bodyDisplay = std::string(kQuadSpeciesNames[species]) + " Body";
        const std::string centaurDisplay = std::string(kQuadSpeciesNames[species]) + " (Centaur)";
        nameStorage.push_back(bodyName);
        nameStorage.push_back(centaurName);
        nameStorage.push_back(rigName);
        nameStorage.push_back(bodyDisplay);
        nameStorage.push_back(centaurDisplay);
        const char *idBody = nameStorage[nameStorage.size() - 5].c_str();
        const char *idCentaur = nameStorage[nameStorage.size() - 4].c_str();
        const char *idRig = nameStorage[nameStorage.size() - 3].c_str();
        const char *displayBody = nameStorage[nameStorage.size() - 2].c_str();
        const char *displayCentaur = nameStorage[nameStorage.size() - 1].c_str();

        static const ConnectorDefinition quadConnectors[] = {
            {"wings1.left", "wing.mount", jointAnchor("withers1", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings1.right", "wing.mount", jointAnchor("withers1", 0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings2.left", "wing.mount", jointAnchor("withers2", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings2.right", "wing.mount", jointAnchor("withers2", 0.020f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ConnectorDefinition centaurConnectors[] = {
            {"torso.mount", "body.mount", jointAnchor("torso_mount", 0.0f, 0.0f, 0.0f), nullptr, 1},
            {"wings1.left", "wing.mount", jointAnchor("withers1", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings1.right", "wing.mount", jointAnchor("withers1", 0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings2.left", "wing.mount", jointAnchor("withers2", -0.020f, 0.0f, 0.0f), nullptr, 1},
            {"wings2.right", "wing.mount", jointAnchor("withers2", 0.020f, 0.0f, 0.0f), nullptr, 1},
        };
        static const ParameterBinding quadBindings[] = {
            scaleBinding("body.height", ParameterBindingTarget::PartScaleY, 1.0f),
            scaleBinding("body.width", ParameterBindingTarget::PartScaleX, 1.0f),
            scaleBinding("body.width", ParameterBindingTarget::PartScaleZ, 0.70f),
            scaleBinding("body.muscle_mass", ParameterBindingTarget::PartScaleX, 0.20f),
            scaleBinding("body.muscle_mass", ParameterBindingTarget::PartScaleZ, 0.20f),
        };
        static const char *quadTags[] = {"slot:quadruped"};
        static const MaterialSlotDefinition quadSlots[] = {{"primary", {1.0f, 1.0f, 1.0f}, 0.80f, 0.0f}};

        PartDefinition bodyPart{};
        bodyPart.id                    = idBody;
        bodyPart.displayName           = displayBody;
        bodyPart.geometryId            = idBody;
        bodyPart.rigId                 = idRig;
        bodyPart.connectors            = quadConnectors;
        bodyPart.connectorCount        = static_cast<std::uint32_t>(std::size(quadConnectors));
        bodyPart.parameterBindings     = quadBindings;
        bodyPart.parameterBindingCount = static_cast<std::uint32_t>(std::size(quadBindings));
        bodyPart.materialSlots         = quadSlots;
        bodyPart.materialSlotCount     = 1;
        bodyPart.tags                  = quadTags;
        bodyPart.tagCount              = 1;
        bodyPart.boundsCenter[1]       = (kQuadGroundY[species] + kQuadMountY[species]) * 0.5f;
        bodyPart.boundsCenter[2]       = kQuadLengthZ[species] * 0.25f;
        bodyPart.boundsRadius          = kQuadLengthZ[species] * 0.62f + 0.30f;
        parts.push_back(bodyPart);

        PartDefinition centaurPart = bodyPart;
        centaurPart.id                    = idCentaur;
        centaurPart.displayName           = displayCentaur;
        centaurPart.geometryId            = idCentaur;
        centaurPart.connectors            = centaurConnectors;
        centaurPart.connectorCount        = static_cast<std::uint32_t>(std::size(centaurConnectors));
        parts.push_back(centaurPart);
    }

    // ---- geometry (extracted OBJ downloads) --------------------------------
    const std::filesystem::path partDirectory = std::filesystem::path(assetRoot) / "character_parts";
    std::vector<std::pair<std::string, std::string>> objParts = {
        {"torso_ranger", "torso_ranger.obj"},
        {"head_human", "head_human.obj"},
        {"head_dragon", "head_dragon.obj"},
        {"arm_human_left", "arm_human_left.obj"},
        {"leg_human_left", "leg_human_left.obj"},
        {"wing_dragon_left", "wing_dragon_left.obj"},
        {"tail_reptile", "tail_reptile.obj"},
        {"tail_naga", "tail_naga.obj"},
    };
    for (int species = 0; species < kQuadSpeciesCount; ++species)
    {
        const std::string id = kQuadSpeciesIds[species];
        objParts.push_back({"quad_body_" + id, "quad_body_" + id + ".obj"});
        objParts.push_back({"quad_centaur_" + id, "quad_centaur_" + id + ".obj"});
        objParts.push_back({"head_" + id, "head_" + id + ".obj"});
        objParts.push_back({"tail_" + id, "tail_" + id + ".obj"});
    }
    for (const auto &[geometryId, fileName] : objParts)
    {
        if (!loadGeometry(geometryId, partDirectory / fileName, renderer, rendererHandle))
        {
            return false;
        }
    }
    // Mirror the right-side limbs from the left OBJs at load time: the
    // junction ring sits at the part origin, so mirroring across X is exact
    // at the interface and keeps each side's anatomical bend correct. (The
    // mirror is a rigid transform of the downloaded geometry, same as the
    // pack's own symmetric design.)
    const auto mirrorAcrossX = [this, &renderer, rendererHandle](const char *leftId, const char *rightId)
    {
        GeometryEntry mirrored;
        mirrored.rest       = geometries_[leftId].rest;
        for (Vertex &vertex : mirrored.rest.vertices)
        {
            vertex.position.x *= -1.0f;
            vertex.normal.x *= -1.0f;
        }
        for (std::size_t index = 0; index + 2 < mirrored.rest.indices.size(); index += 3)
        {
            std::swap(mirrored.rest.indices[index], mirrored.rest.indices[index + 1]);
        }
        mirrored.triangles = geometries_[leftId].triangles;
        geometries_[rightId] = std::move(mirrored);
        uploadGeometry(rightId, renderer, rendererHandle);
    };
    mirrorAcrossX("arm_human_left", "arm_human_right");
    mirrorAcrossX("leg_human_left", "leg_human_right");
    mirrorAcrossX("wing_dragon_left", "wing_dragon_right");

    // Fix up the registered triangle counts from the loaded geometry.
    PartDefinition resolvedParts[64];
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
            for (std::uint32_t c = 0; c < resolvedParts[index].connectorCount; ++c)
            {
                std::fprintf(stderr, "  conn %u: id=%s role=%s space=%u joint=%s\n", c,
                    resolvedParts[index].connectors[c].id ? resolvedParts[index].connectors[c].id : "(null)",
                    resolvedParts[index].connectors[c].role ? resolvedParts[index].connectors[c].role : "(null)",
                    static_cast<unsigned>(resolvedParts[index].connectors[c].anchor.space),
                    resolvedParts[index].connectors[c].anchor.jointSemantic
                        ? resolvedParts[index].connectors[c].anchor.jointSemantic
                        : "(null)");
            }
            return false;
        }
    }

    // ---- assembly profiles: the four body plans -----------------------------
    {
        // humanoid: torso + head + one arm pair + one leg pair + species tail
        // (the fox tail is the default; the tail variants swap live)
        static const ProfilePart humanoidParts[] = {
            {"torso", "torso_ranger"},
            {"head", "head_human"},
            {"arm_left", "arm_human_left"},
            {"arm_right", "arm_human_right"},
            {"leg_left", "leg_human_left"},
            {"leg_right", "leg_human_right"},
            {"tail", "tail_fox"},
        };
        static const ProfileConnection humanoidConnections[] = {
            {"torso", "neck.base", "head", "head.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "shoulder.left", "arm_left", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "shoulder.right", "arm_right", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "hip.left", "leg_left", "hip.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "hip.right", "leg_right", "hip.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            // The tail piece carries its back-down rest pitch baked in the
            // OBJ (revision 9); the connection stays identity.
            {"torso", "tail.base", "tail", "tail.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
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
        // centaur: quadruped hindquarters + human torso ("chest") + head + arms
        static const ProfilePart centaurParts[] = {
            {"body", "quad_centaur_horse"},
            {"chest", "torso_ranger"},
            {"head", "head_human"},
            {"arm_left", "arm_human_left"},
            {"arm_right", "arm_human_right"},
        };
        static const ProfileConnection centaurConnections[] = {
            // The torso.mount joint sits at the measured horse neck base (a
            // little sunk), so the human torso's hip ring closes over the
            // neck-stump opening of the headless-horse centaur body.
            {"body", "torso.mount", "chest", "body.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "neck.base", "head", "head.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "shoulder.left", "arm_left", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"chest", "shoulder.right", "arm_right", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
        };
        const AssemblyProfileDefinition centaurProfile{
            .id              = centaurProfileId,
            .displayName     = "Centaur",
            .parts           = centaurParts,
            .partCount       = static_cast<std::uint32_t>(std::size(centaurParts)),
            .connections     = centaurConnections,
            .connectionCount = static_cast<std::uint32_t>(std::size(centaurConnections)),
        };
        if (!character.registerProfile(character.self, &centaurProfile, &error))
        {
            std::fprintf(stderr, "demo: profile registration failed (%u)\n", static_cast<unsigned>(error));
            return false;
        }
    }
    {
        // naga: human torso on a serpent lower body + head + arms
        static const ProfilePart nagaParts[] = {
            {"torso", "torso_ranger"},
            {"naga_body", "tail_naga"},
            {"head", "head_human"},
            {"arm_left", "arm_human_left"},
            {"arm_right", "arm_human_right"},
        };
        static const ProfileConnection nagaConnections[] = {
            // The serpent piece is re-centred on its measured root ring and
            // pitched horizontal by the extractor; this connection places the
            // ring centre at torso-local (0, -0.04, -0.05), burying ring and
            // root flare inside the hip opening (the flare's front edge sits
            // 0.016 m inside the torso silhouette) so the lower half grafts
            // seamlessly (rev-9 shipped the piece off its measured ring,
            // which read as two disconnected halves).
            {"torso", "tail.base", "naga_body", "tail.root",
                {0.0f, -0.031f, 0.176f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "neck.base", "head", "head.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "shoulder.left", "arm_left", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
            {"torso", "shoulder.right", "arm_right", "shoulder.root", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},
        };
        const AssemblyProfileDefinition nagaProfile{
            .id              = nagaProfileId,
            .displayName     = "Naga",
            .parts           = nagaParts,
            .partCount       = static_cast<std::uint32_t>(std::size(nagaParts)),
            .connections     = nagaConnections,
            .connectionCount = static_cast<std::uint32_t>(std::size(nagaConnections)),
        };
        if (!character.registerProfile(character.self, &nagaProfile, &error))
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
        static const ProfileConnection* quadrupedConnections = nullptr;
        const AssemblyProfileDefinition quadrupedProfile{
            .id              = quadrupedProfileId,
            .displayName     = "Quadruped",
            .parts           = quadrupedParts,
            .partCount       = static_cast<std::uint32_t>(std::size(quadrupedParts)),
            .connections     = quadrupedConnections,
            .connectionCount = 0,
        };
        if (!character.registerProfile(character.self, &quadrupedProfile, &error))
        {
            std::fprintf(stderr, "demo: profile registration failed (%u)\n", static_cast<unsigned>(error));
            return false;
        }
    }

    // ---- render info table (UI grouping + materials) -----------------------
    const std::pair<const char *, const char *> slotOfPart[] = {
        {"torso_ranger", "torso"},
        {"head_human", "head"},
        {"head_dragon", "head"},
        {"arm_human_left", "arms"},
        {"arm_human_right", "arms"},
        {"leg_human_left", "legs"},
        {"leg_human_right", "legs"},
        {"wing_dragon_left", "wings"},
        {"wing_dragon_right", "wings"},
        {"tail_reptile", "tail"},
        {"tail_naga", "tail"},
    };
    std::vector<std::pair<std::string, std::string>> slotTable(slotOfPart,
        slotOfPart + sizeof(slotOfPart) / sizeof(slotOfPart[0]));
    for (int species = 0; species < kQuadSpeciesCount; ++species)
    {
        const std::string id = kQuadSpeciesIds[species];
        slotTable.push_back({"quad_body_" + id, "quadruped"});
        slotTable.push_back({"quad_centaur_" + id, "quadruped"});
        slotTable.push_back({"head_" + id, "head"});
        slotTable.push_back({"tail_" + id, "tail"});
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
        // extractor from the source textures), so the draw-call base color
        // stays white and only roughness/metallic come from the slot
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

    return true;
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
