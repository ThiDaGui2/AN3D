#include "IK_skeleton_drawable.hpp"

namespace cgp
{
    IK_skeleton_drawable::IK_skeleton_drawable()
        :segments(), joint_frame(), joint_sphere(), data()
    {}

    IK_skeleton_drawable::IK_skeleton_drawable(numarray<vec3> const& ik_skeleton, numarray<int> const& parent_index)
        : segments(), joint_frame(), joint_sphere(), data(ik_skeleton)
    {
        size_t const N = parent_index.size();
        numarray<vec3> edges;
        for (size_t k = 1; k < N; ++k){
            size_t const parent = parent_index[k];
            assert_cgp_no_msg(parent>=0 && parent<N);
            edges.push_back(ik_skeleton[k]);
            edges.push_back(ik_skeleton[parent]);
        }
        
        segments.display_type = curve_drawable_display_type::Segments;
        segments.initialize_data_on_gpu(edges);
        joint_frame.initialize_data_on_gpu(mesh_primitive_frame());
        joint_sphere.initialize_data_on_gpu(mesh_primitive_sphere());
    }

    void IK_skeleton_drawable::clear()
    {
        segments.clear();
        joint_frame.clear();
        joint_sphere.clear();
        data.clear();
    }

    void IK_skeleton_drawable::update(numarray<vec3> const& ik_skeleton, numarray<int> const& parent_index)
    {
        data = ik_skeleton;

        size_t const N = ik_skeleton.size();
        numarray<vec3> edges;
        for (size_t k = 1; k < N; ++k){
            size_t const parent = parent_index[k];
            assert_cgp_no_msg(parent>=0 && parent<N);
            edges.push_back(ik_skeleton[k]);
            edges.push_back(ik_skeleton[parent]);
        }

        segments.vbo_position.update(edges);
    }
}