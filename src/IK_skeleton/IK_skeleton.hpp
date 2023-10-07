#pragma once

#include "cgp/core/array/numarray/numarray.hpp"
#include "cgp/geometry/transform/affine/affine_rt/affine_rt.hpp"
#include "cgp/geometry/vec/vec3/vec3.hpp"
#include "skeleton/skeleton.hpp"
namespace cgp
{
struct IK_skeleton
{
    cgp::numarray<float> join_length;
    cgp::numarray<cgp::vec3> join_positions;
    cgp::numarray<int> join_parent;
    float epsilon;
    size_t max_iter;

private:
    bool is_reachable(cgp::vec3 target);

public:
    IK_skeleton() = default;
    IK_skeleton(cgp::skeleton_animation_structure &skeleton_, float epsilon_, size_t max_iter_);

public:
    void calculate_IK_joins(cgp::vec3 target_position);

public:
    void update_skeleton(float animation_time, skeleton_animation_structure &skeleton);
    // skeleton.animation_time = {0, animation_time}
    // skeleton.animation_geometry_local = {current_pose, IK_pose)
};
} // namespace cgp