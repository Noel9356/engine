// spatial_index.hpp - umbrella header for the spatial_index module.
//
// Pulls in the AABB primitive, the dynamic AABB tree, the static BVH, and the
// combined scene index; consumers can include this single header.

#pragma once

#include <engine/spatial_index/aabb.hpp>
#include <engine/spatial_index/dynamic_aabb_tree.hpp>
#include <engine/spatial_index/frustum.hpp>
#include <engine/spatial_index/scene_spatial_index.hpp>
#include <engine/spatial_index/static_bvh.hpp>
#include <engine/spatial_index/types.hpp>

