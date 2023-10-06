#pragma once

#include "cgp/core/array/numarray/numarray.hpp"
#include "cgp/geometry/transform/affine/affine_rt/affine_rt.hpp"
#include "cgp/geometry/vec/vec3/vec3.hpp"
#include "skeleton/skeleton.hpp"

struct IK_skeleton
{
    cgp::numarray<float> join_length;
    cgp::numarray<cgp::vec3> join_positions;
    cgp::numarray<int> join_parent;

public:
    IK_skeleton(cgp::skeleton_animation_structure &skeleton, float epsilon);

public:
    void calculate_IK_joins(cgp::vec3 target_position);

public:
    void update_skeleton(float animation_time);
    // skeleton.animation_time = {0, animation_time}
    // skeleton.animation_geometry_local = {current_pose, IK_pose)
};
