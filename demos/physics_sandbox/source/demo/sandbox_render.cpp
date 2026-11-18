// sandbox_render.cpp - procedural primitive meshes, interpolation state
// capture, and draw assembly. Meshes are unit-sized (box half extents 1,
// sphere radius 1, capsule radius and half-length 1) and every draw scales
// them from the body's shape description, so one mesh set serves any scene.
#include "sandbox_render.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

namespace demo::physics_sandbox
{
namespace
{

namespace renderer = engine::renderer;
namespace math     = engine::physics_rigid::math;

constexpr std::uint32_t kVertexFloats = 11; // position, normal, color, uv

struct RenderVertex
{
    float position[3];
    float normal[3];
    float color[3];
    float uv[2];
};

std::array<renderer::VertexAttribute, 4> canonicalAttributes()
{
    return {{
        {renderer::VertexSemantic::Position, renderer::VertexFormat::Float32x3, 0},
        {renderer::VertexSemantic::Normal, renderer::VertexFormat::Float32x3, 12},
        {renderer::VertexSemantic::Color, renderer::VertexFormat::Float32x3, 24},
        {renderer::VertexSemantic::TexCoord0, renderer::VertexFormat::Float32x2, 36},
    }};
}

renderer::MeshHandle uploadMesh(renderer::RenderCapability &renderCapability, renderer::RendererHandle renderer,
    const std::vector<RenderVertex> &vertices, const std::vector<std::uint32_t> &indices)
{
    const auto                attributes = canonicalAttributes();
    renderer::MeshDescription description;
    description.vertices             = vertices.data();
    description.vertexCount          = static_cast<std::uint32_t>(vertices.size());
    description.indices              = indices.data();
    description.indexCount           = static_cast<std::uint32_t>(indices.size());
    description.indexFormat          = renderer::MeshIndexFormat::UInt32;
    description.vertexStrideBytes    = sizeof(RenderVertex);
    description.vertexAttributes     = attributes.data();
    description.vertexAttributeCount = static_cast<std::uint32_t>(attributes.size());
    renderer::RenderError error      = renderer::RenderError::None;
    return renderCapability.createMesh(renderCapability.self, renderer, &description, &error);
}

// Unit cube: half extents 1, flat per-face normals.
void buildCube(std::vector<RenderVertex> &vertices, std::vector<std::uint32_t> &indices)
{
    const float faces[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const auto &faceNormal : faces)
    {
        const std::uint32_t base       = static_cast<std::uint32_t>(vertices.size());
        float               tangent[3] = {faceNormal[2], faceNormal[0], faceNormal[1]};
        float               bitangent[3]
            = {faceNormal[1] * tangent[2] - faceNormal[2] * tangent[1],
                faceNormal[2] * tangent[0] - faceNormal[0] * tangent[2],
                faceNormal[0] * tangent[1] - faceNormal[1] * tangent[0]};
        for (std::uint32_t corner = 0; corner < 4; ++corner)
        {
            const float  signU = (corner & 1u) != 0u ? 1.0f : -1.0f;
            const float  signV = (corner & 2u) != 0u ? 1.0f : -1.0f;
            RenderVertex vertex{};
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                vertex.position[axis] = faceNormal[axis] + tangent[axis] * signU + bitangent[axis] * signV;
            }
            std::memcpy(vertex.normal, faceNormal, sizeof(vertex.normal));
            // Neutral vertex colors: the material's baseColor drives shading.
            vertex.color[0] = vertex.color[1] = vertex.color[2] = 1.0f;
            vertex.uv[0]                                        = signU * 0.5f + 0.5f;
            vertex.uv[1]                                        = signV * 0.5f + 0.5f;
            vertices.push_back(vertex);
        }
        // Corner loop order is (u,v) = (-,-), (+,-), (-,+), (+,+); the
        // perimeter is 0-1-3-2, so both triangles share the face's winding.
        indices.insert(indices.end(), {base, base + 1, base + 3, base, base + 3, base + 2});
    }
}

// Unit sphere: radius 1, UV latitude-longitude grid.
void buildSphere(std::uint32_t stacks, std::uint32_t slices, std::vector<RenderVertex> &vertices,
    std::vector<std::uint32_t> &indices)
{
    for (std::uint32_t stack = 0; stack <= stacks; ++stack)
    {
        const float phi = 3.14159265f * static_cast<float>(stack) / static_cast<float>(stacks);
        for (std::uint32_t slice = 0; slice <= slices; ++slice)
        {
            const float  theta = 6.28318530f * static_cast<float>(slice) / static_cast<float>(slices);
            const float  nx    = std::sin(phi) * std::cos(theta);
            const float  ny    = std::cos(phi);
            const float  nz    = std::sin(phi) * std::sin(theta);
            RenderVertex vertex{};
            vertex.position[0] = nx;
            vertex.position[1] = ny;
            vertex.position[2] = nz;
            std::memcpy(vertex.normal, vertex.position, sizeof(vertex.normal));
            vertex.color[0] = 0.7f;
            vertex.color[1] = 0.7f;
            vertex.color[2] = 0.7f;
            vertex.uv[0]    = static_cast<float>(slice) / static_cast<float>(slices);
            vertex.uv[1]    = static_cast<float>(stack) / static_cast<float>(stacks);
            vertices.push_back(vertex);
        }
    }
    for (std::uint32_t stack = 0; stack < stacks; ++stack)
    {
        for (std::uint32_t slice = 0; slice < slices; ++slice)
        {
            const std::uint32_t rowStart     = stack * (slices + 1) + slice;
            const std::uint32_t nextRowStart = rowStart + slices + 1;
            indices.insert(
                indices.end(), {rowStart, nextRowStart, rowStart + 1, rowStart + 1, nextRowStart, nextRowStart + 1});
        }
    }
}

// Unit capsule: radius 1, half-length 1 along Y (total height 4).
void buildCapsule(std::uint32_t stacks, std::uint32_t slices, std::vector<RenderVertex> &vertices,
    std::vector<std::uint32_t> &indices)
{
    // Two hemispheres plus a cylinder tube; the top cap center sits at +1,
    // the bottom cap center at -1.
    for (std::uint32_t stack = 0; stack <= stacks; ++stack)
    {
        const float phi = 1.57079632f * static_cast<float>(stack) / static_cast<float>(stacks); // 0..pi/2
        for (std::uint32_t slice = 0; slice <= slices; ++slice)
        {
            const float  theta  = 6.28318530f * static_cast<float>(slice) / static_cast<float>(slices);
            const float  radial = std::sin(phi);
            const float  nx     = radial * std::cos(theta);
            const float  ny     = std::cos(phi);
            const float  nz     = radial * std::sin(theta);
            RenderVertex vertex{};
            vertex.position[0] = nx;
            vertex.position[1] = ny + 1.0f;
            vertex.position[2] = nz;
            std::memcpy(vertex.normal, vertex.position, sizeof(float) * 3);
            vertex.normal[1] = ny;
            vertex.color[0] = vertex.color[1] = vertex.color[2] = 1.0f;
            vertices.push_back(vertex);
        }
    }
    const std::uint32_t equatorTop = static_cast<std::uint32_t>(vertices.size());
    for (std::uint32_t stack = 0; stack <= stacks; ++stack)
    {
        const float phi = 1.57079632f * static_cast<float>(stack) / static_cast<float>(stacks); // pi/2..0
        for (std::uint32_t slice = 0; slice <= slices; ++slice)
        {
            const float  theta  = 6.28318530f * static_cast<float>(slice) / static_cast<float>(slices);
            const float  radial = std::sin(phi);
            const float  ny     = -std::cos(phi);
            RenderVertex vertex{};
            vertex.position[0] = radial * std::cos(theta);
            vertex.position[1] = ny - 1.0f;
            vertex.position[2] = radial * std::sin(theta);
            vertex.normal[0]   = radial * std::cos(theta);
            vertex.normal[1]   = ny;
            vertex.normal[2]   = radial * std::sin(theta);
            vertex.color[0] = vertex.color[1] = vertex.color[2] = 0.7f;
            vertices.push_back(vertex);
        }
    }
    auto capRingIndices = [&](std::uint32_t ringStart, bool upward)
    {
        for (std::uint32_t stack = 0; stack < stacks; ++stack)
        {
            for (std::uint32_t slice = 0; slice < slices; ++slice)
            {
                const std::uint32_t current   = ringStart + stack * (slices + 1) + slice;
                const std::uint32_t next      = current + 1;
                const std::uint32_t below     = current + slices + 1;
                const std::uint32_t belowNext = below + 1;
                if (upward)
                {
                    indices.insert(indices.end(), {current, below, next, next, below, belowNext});
                }
                else
                {
                    indices.insert(indices.end(), {current, next, below, next, belowNext, below});
                }
            }
        }
    };
    capRingIndices(0, true);
    capRingIndices(equatorTop, false);
    // Cylinder tube between the two equators.
    for (std::uint32_t slice = 0; slice < slices; ++slice)
    {
        const std::uint32_t topA    = stacks * (slices + 1) + slice;
        const std::uint32_t topB    = topA + 1;
        const std::uint32_t bottomA = equatorTop + slice;
        const std::uint32_t bottomB = bottomA + 1;
        indices.insert(indices.end(), {topA, bottomA, topB, topB, bottomA, bottomB});
    }
}

void composeModelMatrix(
    const math::Vec3 &position, const math::Quaternion &rotation, const math::Vec3 &scale, float *outMatrix)
{
    const math::Mat3 rotationMatrix = math::makeRotationMatrix(rotation);
    const math::Vec3 scaledColumns[3]
        = {math::Vec3{
               rotationMatrix.elements[0] * scale.x,
               rotationMatrix.elements[1] * scale.x,
               rotationMatrix.elements[2] * scale.x},
            math::Vec3{
                rotationMatrix.elements[3] * scale.y,
                rotationMatrix.elements[4] * scale.y,
                rotationMatrix.elements[5] * scale.y},
            math::Vec3{
                rotationMatrix.elements[6] * scale.z,
                rotationMatrix.elements[7] * scale.z,
                rotationMatrix.elements[8] * scale.z}};
    // Column-major 4x4.
    outMatrix[0]  = static_cast<float>(scaledColumns[0].x);
    outMatrix[1]  = static_cast<float>(scaledColumns[0].y);
    outMatrix[2]  = static_cast<float>(scaledColumns[0].z);
    outMatrix[3]  = 0.0f;
    outMatrix[4]  = static_cast<float>(scaledColumns[1].x);
    outMatrix[5]  = static_cast<float>(scaledColumns[1].y);
    outMatrix[6]  = static_cast<float>(scaledColumns[1].z);
    outMatrix[7]  = 0.0f;
    outMatrix[8]  = static_cast<float>(scaledColumns[2].x);
    outMatrix[9]  = static_cast<float>(scaledColumns[2].y);
    outMatrix[10] = static_cast<float>(scaledColumns[2].z);
    outMatrix[11] = 0.0f;
    outMatrix[12] = static_cast<float>(position.x);
    outMatrix[13] = static_cast<float>(position.y);
    outMatrix[14] = static_cast<float>(position.z);
    outMatrix[15] = 1.0f;
}

math::Quaternion nlerp(const math::Quaternion &from, const math::Quaternion &to, float alpha)
{
    math::Quaternion target = to;
    if (math::dot(from, target) < 0.0)
    {
        target = math::Quaternion{-target.x, -target.y, -target.z, -target.w};
    }
    math::Quaternion blended{
        from.x + (target.x - from.x) * alpha,
        from.y + (target.y - from.y) * alpha,
        from.z + (target.z - from.z) * alpha,
        from.w + (target.w - from.w) * alpha};
    return math::normalize(blended);
}

} // namespace

bool SandboxRenderer::createMeshes(
    renderer::RenderCapability &renderCapability, renderer::RendererHandle renderer, std::string &outReport)
{
    std::vector<RenderVertex>  vertices;
    std::vector<std::uint32_t> indices;
    std::ostringstream         report;

    vertices.clear();
    indices.clear();
    buildCube(vertices, indices);
    cubeMesh_ = uploadMesh(renderCapability, renderer, vertices, indices);

    vertices.clear();
    indices.clear();
    buildSphere(24, 32, vertices, indices);
    sphereMesh_ = uploadMesh(renderCapability, renderer, vertices, indices);

    vertices.clear();
    indices.clear();
    buildCapsule(10, 20, vertices, indices);
    capsuleMesh_ = uploadMesh(renderCapability, renderer, vertices, indices);

    vertices.clear();
    indices.clear();
    buildSphere(8, 12, vertices, indices);
    clothNodeMesh_ = uploadMesh(renderCapability, renderer, vertices, indices);

    const bool ok = cubeMesh_ != renderer::invalidMesh && sphereMesh_ != renderer::invalidMesh
                 && capsuleMesh_ != renderer::invalidMesh && clothNodeMesh_ != renderer::invalidMesh;
    if (!ok)
    {
        report << "primitive mesh upload failed\n";
        outReport = report.str();
    }
    return ok;
}

void SandboxRenderer::destroyMeshes(renderer::RenderCapability &renderCapability, renderer::RendererHandle renderer)
{
    for (const renderer::MeshHandle handle : {cubeMesh_, sphereMesh_, capsuleMesh_, clothNodeMesh_})
    {
        if (handle != renderer::invalidMesh)
        {
            renderCapability.destroyMesh(renderCapability.self, renderer, handle);
        }
    }
    cubeMesh_ = sphereMesh_ = capsuleMesh_ = clothNodeMesh_ = renderer::invalidMesh;
}

void SandboxRenderer::syncScene(const SceneDependencies &dependencies, const SceneHandles &handles,
    const engine::physics_cloth::ClothCapability *clothCapability)
{
    bodyStateIndex_.clear();
    bodyStates_.clear();
    bodyShapes_.clear();
    bodyKinds_.clear();
    clothBuffers_.clear();

    for (std::size_t index = 0; index < handles.bodies.size(); ++index)
    {
        const SceneBody                 &sceneBody = handles.bodies[index];
        engine::physics_rigid::BodyState state{};
        if (!dependencies.rigid->copyBodyState(dependencies.rigid->self, handles.world, sceneBody.body, &state))
        {
            continue;
        }
        engine::physics_rigid::BodyDescription description{};
        if (!dependencies.rigid->copyBodyDescription(
                dependencies.rigid->self, handles.world, sceneBody.body, &description))
        {
            continue;
        }
        bodyStateIndex_.emplace(sceneBody.body, bodyStates_.size());
        BodyRenderState renderState;
        renderState.current  = state.transform;
        renderState.previous = state.transform;
        bodyStates_.push_back(renderState);
        bodyShapes_.push_back(description.shape);
        bodyKinds_.push_back(description.kind);
    }

    if (clothCapability != nullptr)
    {
        for (const engine::physics_cloth::ClothHandle clothHandle : handles.cloths)
        {
            engine::physics_cloth::ClothInfo info{};
            if (!clothCapability->clothInfo(clothCapability->self, clothHandle, &info))
            {
                continue;
            }
            ClothRenderBuffers buffers;
            buffers.columns    = info.columns;
            buffers.rows       = info.rows;
            buffers.nodeRadius = 0.045;
            buffers.currentPositions.resize(static_cast<std::size_t>(info.particleCount) * 3);
            buffers.previousPositions = buffers.currentPositions;
            clothCapability->copyParticlePositions(
                clothCapability->self, clothHandle, buffers.currentPositions.data(), info.particleCount * 3u, nullptr);
            clothBuffers_.push_back(buffers);
        }
    }
}

void SandboxRenderer::recordFixedStep(const SceneDependencies &dependencies, const SceneHandles &handles,
    const engine::physics_cloth::ClothCapability *clothCapability)
{
    for (std::size_t index = 0; index < handles.bodies.size(); ++index)
    {
        const auto iterator = bodyStateIndex_.find(handles.bodies[index].body);
        if (iterator == bodyStateIndex_.end())
        {
            continue;
        }
        BodyRenderState &state = bodyStates_[iterator->second];
        state.previous         = state.current;
        engine::physics_rigid::BodyState fresh{};
        if (dependencies.rigid->copyBodyState(
                dependencies.rigid->self, handles.world, handles.bodies[index].body, &fresh))
        {
            state.current = fresh.transform;
        }
    }
    if (clothCapability != nullptr)
    {
        for (std::size_t index = 0; index < handles.cloths.size() && index < clothBuffers_.size(); ++index)
        {
            ClothRenderBuffers &buffers = clothBuffers_[index];
            buffers.previousPositions   = buffers.currentPositions;
            const std::uint32_t floats  = static_cast<std::uint32_t>(buffers.currentPositions.size());
            clothCapability->copyParticlePositions(
                clothCapability->self, handles.cloths[index], buffers.currentPositions.data(), floats, nullptr);
        }
    }
}

void SandboxRenderer::buildDrawCommands(const SceneHandles &handles, float alpha)
{
    modelMatrices_.clear();
    materials_.clear();
    clothNodeMatrices_.clear();
    clothLinkMatrices_.clear();

    // Pass one: fill the matrix and material storage to its final size (no
    // reallocation while command pointers reference it).
    std::vector<std::uint32_t>    drawableIndices;
    std::vector<math::Vec3>       scales;
    std::vector<math::Vec3>       positions;
    std::vector<math::Quaternion> rotations;
    for (std::size_t index = 0; index < handles.bodies.size(); ++index)
    {
        const auto iterator = bodyStateIndex_.find(handles.bodies[index].body);
        if (iterator == bodyStateIndex_.end())
        {
            continue;
        }
        const engine::physics_rigid::ShapeDescription &shape = bodyShapes_[iterator->second];
        math::Vec3                                     scale;
        switch (shape.kind)
        {
        case engine::physics_rigid::ShapeKind::Box:
            scale = math::Vec3{shape.boxHalfExtents[0], shape.boxHalfExtents[1], shape.boxHalfExtents[2]};
            break;
        case engine::physics_rigid::ShapeKind::Sphere:
            scale = math::Vec3{shape.sphereRadius, shape.sphereRadius, shape.sphereRadius};
            break;
        case engine::physics_rigid::ShapeKind::Capsule:
            scale = math::Vec3{shape.capsuleRadius, shape.capsuleHalfLength, shape.capsuleRadius};
            break;
        case engine::physics_rigid::ShapeKind::Plane:
        default:
            continue; // planes have no bounded geometry to draw
        }
        const BodyRenderState &state = bodyStates_[iterator->second];
        drawableIndices.push_back(static_cast<std::uint32_t>(index));
        scales.push_back(scale);
        positions.push_back(math::lerp(state.previous.position, state.current.position, alpha));
        rotations.push_back(nlerp(state.previous.rotation, state.current.rotation, alpha));
    }
    modelMatrices_.resize(drawableIndices.size());
    materials_.resize(drawableIndices.size());
    for (std::size_t entry = 0; entry < drawableIndices.size(); ++entry)
    {
        composeModelMatrix(positions[entry], rotations[entry], scales[entry], modelMatrices_[entry].data());
        const SceneBody       &sceneBody = handles.bodies[drawableIndices[entry]];
        renderer::MeshMaterial material;
        material.baseColor[0]    = sceneBody.baseColor[0];
        material.baseColor[1]    = sceneBody.baseColor[1];
        material.baseColor[2]    = sceneBody.baseColor[2];
        material.metallicFactor  = 0.0f;
        material.roughnessFactor = 0.85f;
        materials_[entry]        = material;
    }

    // Pass two: assemble the commands; the storage no longer moves.
    drawCommands_.clear();
    for (std::size_t entry = 0; entry < drawableIndices.size(); ++entry)
    {
        const engine::physics_rigid::ShapeDescription &shape
            = bodyShapes_[bodyStateIndex_.at(handles.bodies[drawableIndices[entry]].body)];
        renderer::RenderDrawCommand command;
        command.mesh        = shape.kind == engine::physics_rigid::ShapeKind::Box    ? cubeMesh_
                            : shape.kind == engine::physics_rigid::ShapeKind::Sphere ? sphereMesh_
                                                                                     : capsuleMesh_;
        command.modelMatrix = modelMatrices_[entry].data();
        command.material    = &materials_[entry];
        drawCommands_.push_back(command);
    }

    // Cloth renders as its particle lattice: one sphere node per particle
    // plus one thin box per structural edge, all interpolated.
    for (ClothRenderBuffers &buffers : clothBuffers_)
    {
        const math::Vec3 nodeScale{buffers.nodeRadius, buffers.nodeRadius, buffers.nodeRadius};
        const math::Vec3 linkScale{buffers.nodeRadius * 0.6, buffers.nodeRadius * 0.6, buffers.nodeRadius * 0.6};
        auto             particleAt = [&](std::uint32_t column, std::uint32_t row, std::vector<double> &storage)
        {
            const std::size_t offset         = (static_cast<std::size_t>(row) * buffers.columns + column) * 3;
            const std::size_t previousOffset = offset;
            (void)previousOffset;
            return math::Vec3{storage[offset], storage[offset + 1], storage[offset + 2]};
        };
        auto interpolatedParticle = [&](std::uint32_t column, std::uint32_t row)
        {
            const std::size_t offset = (static_cast<std::size_t>(row) * buffers.columns + column) * 3;
            return math::Vec3{
                buffers.previousPositions[offset]
                    + (buffers.currentPositions[offset] - buffers.previousPositions[offset]) * alpha,
                buffers.previousPositions[offset + 1]
                    + (buffers.currentPositions[offset + 1] - buffers.previousPositions[offset + 1]) * alpha,
                buffers.previousPositions[offset + 2]
                    + (buffers.currentPositions[offset + 2] - buffers.previousPositions[offset + 2]) * alpha};
        };
        (void)particleAt;

        for (std::uint32_t row = 0; row < buffers.rows; ++row)
        {
            for (std::uint32_t column = 0; column < buffers.columns; ++column)
            {
                const math::Vec3 position = interpolatedParticle(column, row);
                clothNodeMatrices_.emplace_back();
                composeModelMatrix(position, math::Quaternion{}, nodeScale, clothNodeMatrices_.back().data());
            }
        }
        auto pushLink = [&](const math::Vec3 &from, const math::Vec3 &to)
        {
            const math::Vec3 delta  = to - from;
            const double     length = math::length(delta);
            if (length <= 1.0e-9)
            {
                return;
            }
            const math::Vec3       direction = delta * (1.0 / length);
            const math::Quaternion rotation  = math::makeRotationFromTo(math::Vec3{0.0, 1.0, 0.0}, direction);
            const math::Vec3       center    = from + delta * 0.5;
            clothLinkMatrices_.emplace_back();
            composeModelMatrix(
                center, rotation, math::Vec3{linkScale.x, length, linkScale.z}, clothLinkMatrices_.back().data());
        };
        for (std::uint32_t row = 0; row < buffers.rows; ++row)
        {
            for (std::uint32_t column = 0; column < buffers.columns; ++column)
            {
                if (column + 1 < buffers.columns)
                {
                    pushLink(interpolatedParticle(column, row), interpolatedParticle(column + 1, row));
                }
                if (row + 1 < buffers.rows)
                {
                    pushLink(interpolatedParticle(column, row), interpolatedParticle(column, row + 1));
                }
            }
        }
    }
}

void SandboxRenderer::submit(renderer::RenderCapability &renderCapability, renderer::RendererHandle renderer)
{
    if (!drawCommands_.empty())
    {
        renderCapability.drawMeshes(
            renderCapability.self, renderer, drawCommands_.data(), static_cast<std::uint32_t>(drawCommands_.size()));
    }
    renderer::MeshMaterial clothMaterial;
    clothMaterial.baseColor[0]    = 0.30f;
    clothMaterial.baseColor[1]    = 0.66f;
    clothMaterial.baseColor[2]    = 0.78f;
    clothMaterial.roughnessFactor = 0.6f;
    if (!clothLinkMatrices_.empty())
    {
        renderCapability.drawMeshInstances(renderCapability.self,
            renderer,
            cubeMesh_,
            clothLinkMatrices_.front().data(),
            static_cast<std::uint32_t>(clothLinkMatrices_.size()),
            &clothMaterial);
    }
    if (!clothNodeMatrices_.empty())
    {
        renderCapability.drawMeshInstances(renderCapability.self,
            renderer,
            clothNodeMesh_,
            clothNodeMatrices_.front().data(),
            static_cast<std::uint32_t>(clothNodeMatrices_.size()),
            &clothMaterial);
    }
}

} // namespace demo::physics_sandbox
