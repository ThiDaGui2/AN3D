#include "IK_skeleton.hpp"
namespace cgp
{
    IK_skeleton::IK_skeleton(cgp::skeleton_animation_structure &skeleton_, float epsilon_, size_t max_iter_)
    {
        epsilon = epsilon_;
        max_iter = max_iter_;
        joint_parents = skeleton_.parent_index;
        joint_lengths = cgp::numarray<float>(skeleton_.number_joint() - 1);
        for (int i = 0; i < skeleton_.number_joint() - 1; i++)
        {
            joint_lengths[i] = cgp::norm(skeleton_.rest_pose_local[i + 1].translation);
        }

        numarray<affine_rt> rest_pose_global = skeleton_.rest_pose_global();
        joint_positions = cgp::numarray<vec3>(skeleton_.number_joint());
        for (int i = 0; i < skeleton_.number_joint(); i++)
        {
            joint_positions[i] = rest_pose_global[i].translation;
        }
        assert_cgp(joint_positions.size() >= 2, "IK skeleton need at least 2 joints and " + str(joint_positions.size()) + "were given")

    }
    
    bool IK_skeleton::is_reachable(vec3 target)
    {
        float dist = cgp::norm(target - joint_positions[0]);
        float total_length = 0;
        for (int i = 0; i < joint_lengths.size(); i++)
        {
            total_length += joint_lengths[i];
        }
        return dist <= total_length;
    }

    // FABRIK algorithm
    void IK_skeleton::calculate_IK_joints(cgp::vec3 target_position)
    {
        // 1. Forward reaching
        // 2. Backward reaching
        // 3. Update joint positions

        if (!is_reachable(target_position))
        {
            for (int i = 0; i < joint_positions.size() - 1; i++)
            {
                float r = norm(target_position - joint_positions[i]);
                float lambda = joint_lengths[i] / r;
                joint_positions[i + 1] = (1 - lambda) * joint_positions[i] + lambda * target_position;
            }
        }

        else
        {
            // TODO: need to check if all joints and the target are aligned: maybe not able to converge if that is the
            //  case

            bool all_aligned = true;
            for (int iter = 2; iter < joint_positions.size() && all_aligned; iter++)
            {
                vec3 ab = joint_positions[iter] - joint_positions[0];
                vec3 ac = joint_positions[1] - joint_positions[0];
                all_aligned = (is_equal(norm(cross(ab, ac)), 0));
            }
            if (all_aligned)
            {
                vec3 ab = target_position - joint_positions[0];
                vec3 ac = joint_positions[1] - joint_positions[0];
                all_aligned = (is_equal(norm(cross(ab, ac)), 0));
                if (all_aligned && joint_positions.size() > 2)
                {
                    vec3 offset = orthogonal_vector(joint_positions[1] - joint_positions[0]);
                    offset = 0.01f * offset;
                    joint_positions[joint_positions.size() - 2] += offset;
                }
            }

            size_t nb_iter = 0;
            vec3 b = joint_positions[0];
            float dif = norm(target_position - joint_positions[joint_positions.size() - 1]);
            while (dif > epsilon && nb_iter < max_iter)
            {
                joint_positions[joint_positions.size() - 1] = target_position;
                for (int i = joint_positions.size() - 2; i >= 0; i--)
                {
                    float r = norm(joint_positions[i + 1] - joint_positions[i]);
                    float lambda = joint_lengths[i] / r;
                    joint_positions[i] = (1 - lambda) * joint_positions[i + 1] + lambda * joint_positions[i];
                }

                joint_positions[0] = b;
                for (int i = 0; i < joint_positions.size() - 1; i++)
                {
                    float r = norm(joint_positions[i + 1] - joint_positions[i]);
                    float lambda = joint_lengths[i] / r;
                    joint_positions[i + 1] = (1 - lambda) * joint_positions[i] + lambda * joint_positions[i + 1];
                }

                dif = norm(target_position - joint_positions[joint_positions.size() - 1]);
                nb_iter++;
            }
        }
    }

    void IK_skeleton::update_skeleton(float animation_time, skeleton_animation_structure &skeleton)
    {
        int nb_joints = joint_positions.size();

        // update local geometry
        numarray<affine_rt> local_geometry(skeleton.rest_pose_local);
        numarray<affine_rt> global_geometry(nb_joints);
        global_geometry[0] = skeleton.rest_pose_local[0];

        for (int k = 1; k < nb_joints; k++)
        {
            affine_rt inverse_transformation = inverse(global_geometry[k - 1]);
            vec3 ik_local_position = inverse_transformation * joint_positions[k];
            ik_local_position = normalize(ik_local_position);
            vec3 rest_local_position = normalize(skeleton.rest_pose_local[k].translation);
            rotation_transform rotation =
                    rotation_transform::from_vector_transform(rest_local_position, ik_local_position);
            local_geometry[k - 1].rotation = local_geometry[k - 1].rotation * rotation;
            global_geometry = skeleton_local_to_global(local_geometry, joint_parents);
        }
        numarray<affine_rt> first_frame = skeleton.animation_geometry_local[skeleton.animation_geometry_local.size() - 1];
        skeleton.animation_geometry_local.clear();
        skeleton.animation_geometry_local.push_back(first_frame);
        skeleton.animation_geometry_local.push_back(local_geometry);
        skeleton.animation_time = {0, animation_time};
    }
} // namespace cgp