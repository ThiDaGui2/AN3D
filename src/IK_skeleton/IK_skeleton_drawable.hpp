#pragma once
#include "cgp/cgp.hpp"

namespace cgp
{
    struct IK_skeleton_drawable
    {
        IK_skeleton_drawable();
        IK_skeleton_drawable(numarray<vec3> const& ik_skeleton, numarray<int> const& parent_index);
        void clear();
        void update(numarray<vec3> const& ik_skeleton, numarray<int> const& parent_index);

        float size_frame = 0.05f;
        float size_sphere = 0.01f;
        bool display_segments = true;
        bool display_joint_frame = true;
        bool display_joint_sphere = true;

        curve_drawable segments;
        mesh_drawable joint_frame;
        mesh_drawable joint_sphere;
        numarray<vec3> data;
    };

    template <typename SCENE>
    void draw(IK_skeleton_drawable const& ik_skeleton, SCENE const& scene);
}

namespace cgp
{
    template <typename SCENE>
    void draw(IK_skeleton_drawable const& ik_skeleton, SCENE const& scene)
    {
        if(ik_skeleton.display_segments)
            draw(ik_skeleton.segments, scene);
        
        size_t const N = ik_skeleton.data.size();

        if(ik_skeleton.display_joint_frame)
        {
            mesh_drawable joint_frame_temp = ik_skeleton.joint_frame;
            joint_frame_temp.model.scaling = ik_skeleton.size_frame;
            for (size_t k = 0; k < N; ++k)
            {
                joint_frame_temp.model.translation = ik_skeleton.data[k];
                draw(joint_frame_temp, scene);
            }
        }
        
        if (ik_skeleton.display_joint_sphere)
        {
            mesh_drawable joint_sphere_temp = ik_skeleton.joint_sphere;
            joint_sphere_temp.model.scaling = ik_skeleton.size_sphere;
            for (size_t k = 0; k < N; ++k)
            {
                joint_sphere_temp.model.translation = ik_skeleton.data[k];
                draw(joint_sphere_temp, scene);
            }
        }
    }
}
