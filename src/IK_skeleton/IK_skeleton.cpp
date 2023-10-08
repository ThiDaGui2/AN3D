#include "IK_skeleton.hpp"
namespace cgp
{
    IK_skeleton::IK_skeleton(cgp::skeleton_animation_structure &skeleton_, float epsilon_, size_t max_iter_)
    {
        epsilon = epsilon_;
        max_iter = max_iter_;
        join_parent = skeleton_.parent_index;
        join_length = cgp::numarray<float>(skeleton_.number_joint() - 1);
        for (int i = 0; i < skeleton_.number_joint() - 1; i++)
        {
            join_length[i] = cgp::norm(skeleton_.rest_pose_local[i + 1].translation);
        }

        numarray<affine_rt> rest_pose_global = skeleton_.rest_pose_global();
        join_positions = cgp::numarray<vec3>(skeleton_.number_joint());
        for (int i = 0; i < skeleton_.number_joint(); i++)
        {
            join_positions[i] = rest_pose_global[i].translation;
        }
    }
    
    bool IK_skeleton::is_reachable(vec3 target)
    {
        float dist = cgp::norm(target - join_positions[0]);
        float total_length = 0;
        for (int i = 0; i < join_length.size(); i++)
        {
            total_length += join_length[i];
        }
        return dist <= total_length;
    }

    // FABRIK algorithm
    void IK_skeleton::calculate_IK_joins(cgp::vec3 target_position)
    {
        // 1. Forward reaching
        // 2. Backward reaching
        // 3. Update joint positions

        if (!is_reachable(target_position))
        {
            for (int i = 0; i < join_positions.size() -1; i++)
            {
                float r = norm(target_position - join_positions[i]);
                float lambda = join_length[i] / r;
                join_positions[i + 1] = (1 - lambda) * join_positions[i] + lambda * target_position;
            }
        }

        else
        {
            // TODO: need to check if all joints and the target are aligned: maybe not able to converge if that is the
            //  case
            size_t nb_iter = 0;
            vec3 b = join_positions[0];
            float dif = norm(target_position - join_positions[join_positions.size() - 1]);
            while (dif > epsilon && nb_iter < max_iter)
            {
                join_positions[join_positions.size() - 1] = target_position;
                for (int i = join_positions.size() - 2; i >= 0; i--)
                {
                    float r = norm(join_positions[i + 1] - join_positions[i]);
                    float lambda = join_length[i] / r;
                    join_positions[i] = (1 - lambda) * join_positions[i + 1] + lambda * join_positions[i];
                }

                join_positions[0] = b;
                for (int i = 0; i < join_positions.size() - 1; i++)
                {
                    float r = norm(join_positions[i + 1] - join_positions[i]);
                    float lambda = join_length[i] / r;
                    join_positions[i + 1] = (1 - lambda) * join_positions[i] + lambda * join_positions[i + 1];
                }

                dif = norm(target_position - join_positions[join_positions.size() - 1]);
                nb_iter++;
            }
        }
    }

    void IK_skeleton::update_skeleton(float animation_time, skeleton_animation_structure &skeleton)
    {
        int nb_joints = join_positions.size();
        numarray<numarray<size_t>> children_index(nb_joints);
        for (int k = 0; k < nb_joints; k++)
        {
            for (int iter = 0; iter < nb_joints; iter++)
            {
                if (join_parent[iter] == k)
                    children_index[k].push_back(iter);
            }
        }

        numarray<affine_rt> rest_pose_global = skeleton.rest_pose_global();
        // update local geometry
        numarray<affine_rt> local_geometry(nb_joints);
        numarray<affine_rt> global_geometry(nb_joints);
        local_geometry[0] = rest_pose_global[0];
        global_geometry[0] = rest_pose_global[0];

        for (int k = 1; k < nb_joints; k++)
        {
            affine_rt inverse_transformation = inverse(global_geometry[k - 1]);
            vec3 ik_local_position = inverse_transformation * join_positions[k];
            ik_local_position = normalize(ik_local_position);
            vec3 rest_local_position = normalize(skeleton.rest_pose_local[k].translation);
            rotation_transform rotation =
                    rotation_transform::from_vector_transform(rest_local_position, ik_local_position);
            local_geometry[k - 1].rotation = rotation;
            local_geometry[k].translation = skeleton.rest_pose_local[k].translation;
            if (k == 1)
                global_geometry[0] = local_geometry[0];
            global_geometry = skeleton_local_to_global(local_geometry, join_parent);
        }
        numarray<affine_rt> first_frame = skeleton.animation_geometry_local[skeleton.animation_geometry_local.size() - 1];
        skeleton.animation_geometry_local.clear();
        skeleton.animation_geometry_local.push_back(first_frame);
        skeleton.animation_geometry_local.push_back(local_geometry);
        skeleton.animation_time = {0, animation_time};
    }
} // namespace cgp