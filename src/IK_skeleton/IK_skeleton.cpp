#include "IK_skeleton.hpp"
namespace cgp
{
    IK_skeleton::IK_skeleton(cgp::skeleton_animation_structure &skeleton, float epsilon)
    {
        this->epsilon = epsilon;
        join_parent = skeleton.parent_index;
        join_length = cgp::numarray<float>(skeleton.number_joint() - 1);
        for (int i = 0; i < skeleton.number_joint() - 1; i++)
        {
            join_length[i] = cgp::norm(skeleton.rest_pose_local[i + 1].translation);
        }

        numarray<affine_rt> rest_pose_global = skeleton.rest_pose_global();
        join_positions = cgp::numarray<vec3>(skeleton.number_joint());
        for (int i = 0; i < skeleton.number_joint(); i++)
        {
            join_positions[i] = rest_pose_global[i].translation;
        }
        this->epsilon = epsilon;

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
            vec3 b = join_positions[0];
            float dif = norm(target_position - join_positions[join_positions.size() - 1]);
            while (dif > epsilon)
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
            }
        }
    }

    void IK_skeleton::update_skeleton(float animation_time)
    {
    }
} // namespace cgp