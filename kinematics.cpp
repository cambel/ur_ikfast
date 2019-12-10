/*
 * IKFast UR3
 * 
 * Calculate FK from joint angles.
 * Calculates IK from rotation-translation matrix, or translation-quaternion pose.
 * Performance timing tests.
 *
 * Run the program to view command line parameters.
 * 
 * Author: Cristian Beltran
 * Date: March 2019
 * 
 * Based on IKFast Demo
 * https://github.com/rethink-rlinsalata/kaist-ros-pkg/blob/master/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp
 * 
 */
#include <kinematics.h>

#define IKFAST_HAS_LIBRARY  // Build IKFast with API functions
#define IKFAST_NO_MAIN

#define IK_VERSION 61
#include "ur3_arm_ikfast_solver.cpp"

#define IKREAL_TYPE IkReal // for IKFast 56,61


IKREAL_TYPE eerot[9], eetrans[3];
namespace kinematics
{
    float SIGN(float x)
    {
        return (x >= 0.0f) ? +1.0f : -1.0f;
    }

    float NORM(float a, float b, float c, float d)
    {
        return sqrt(a * a + b * b + c * c + d * d);
    }

    void forward(const double* joint_values, double* pose, char res_type)
    {
        // for IKFast 56,61
        unsigned int num_of_joints = GetNumJoints();
        unsigned int num_free_parameters = GetNumFreeParameters();

        IKREAL_TYPE joints[num_of_joints];

        // cout<<joint_values[2]<<endl;

        for (unsigned int i = 0; i < num_of_joints; i++)
        {
            joints[i] = joint_values[i];
        }
        // for IKFast 56,61
        ComputeFk(joints, eetrans, eerot); // void return

        if (res_type == 'm'){ // Returns pose as Rotation matrix + translation
            for (unsigned int i = 0; i < 3; i++)
            {
                *pose = eetrans[i]; pose++;
            }
            for (unsigned int i = 0; i < 9; i++)
            {
                *pose = eerot[i]; pose++;
            }
        }
        else
        { // Returns pose as Quaternion
            // cout<<"translation: "<<eetrans[0]<<" "<<eetrans[1]<<" "<<eetrans[2]<<endl;
            // Convert rotation matrix to quaternion (Daisuke Miyazaki)
            float q0 = (eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
            float q1 = (eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
            float q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
            float q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
            if (q0 < 0.0f)
                q0 = 0.0f;
            if (q1 < 0.0f)
                q1 = 0.0f;
            if (q2 < 0.0f)
                q2 = 0.0f;
            if (q3 < 0.0f)
                q3 = 0.0f;
            q0 = sqrt(q0);
            q1 = sqrt(q1);
            q2 = sqrt(q2);
            q3 = sqrt(q3);
            if (q0 >= q1 && q0 >= q2 && q0 >= q3)
            {
                q0 *= +1.0f;
                q1 *= SIGN(eerot[7] - eerot[5]);
                q2 *= SIGN(eerot[2] - eerot[6]);
                q3 *= SIGN(eerot[3] - eerot[1]);
            }
            else if (q1 >= q0 && q1 >= q2 && q1 >= q3)
            {
                q0 *= SIGN(eerot[7] - eerot[5]);
                q1 *= +1.0f;
                q2 *= SIGN(eerot[3] + eerot[1]);
                q3 *= SIGN(eerot[2] + eerot[6]);
            }
            else if (q2 >= q0 && q2 >= q1 && q2 >= q3)
            {
                q0 *= SIGN(eerot[2] - eerot[6]);
                q1 *= SIGN(eerot[3] + eerot[1]);
                q2 *= +1.0f;
                q3 *= SIGN(eerot[7] + eerot[5]);
            }
            else if (q3 >= q0 && q3 >= q1 && q3 >= q2)
            {
                q0 *= SIGN(eerot[3] - eerot[1]);
                q1 *= SIGN(eerot[6] + eerot[2]);
                q2 *= SIGN(eerot[7] + eerot[5]);
                q3 *= +1.0f;
            }
            else
            {
                printf("Error while converting to quaternion! \n");
            }
            float r = NORM(q0, q1, q2, q3);
            q0 /= r;
            q1 /= r;
            q2 /= r;
            q3 /= r;
            
            *pose = eetrans[0]; pose++;
            *pose = eetrans[1]; pose++;
            *pose = eetrans[2]; pose++;
            *pose = q1; pose++;
            *pose = q2; pose++;
            *pose = q3; pose++;
            *pose = q0; pose++;
        }

    }

    int inverse(const double* pose, double* q_sols)
    {
        // for IKFast 56,61
        unsigned int num_of_joints = GetNumJoints();
        unsigned int num_free_parameters = GetNumFreeParameters();

        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;

        std::vector<IKREAL_TYPE> vfree(num_free_parameters);
        eetrans[0] = pose[0];
        eetrans[1] = pose[1];
        eetrans[2] = pose[2];
        double qw = pose[6];
        double qx = pose[3];
        double qy = pose[4];
        double qz = pose[5];
        const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        qw *= n;
        qx *= n;
        qy *= n;
        qz *= n;
        eerot[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
        eerot[1] = 2.0f * qx * qy - 2.0f * qz * qw;
        eerot[2] = 2.0f * qx * qz + 2.0f * qy * qw;
        eerot[3] = 2.0f * qx * qy + 2.0f * qz * qw;
        eerot[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
        eerot[5] = 2.0f * qy * qz - 2.0f * qx * qw;
        eerot[6] = 2.0f * qx * qz - 2.0f * qy * qw;
        eerot[7] = 2.0f * qy * qz + 2.0f * qx * qw;
        eerot[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;

        // for IKFast 56,61
        bool b1Success = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

        std::vector<double> sols;

        if(b1Success){
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
            // printf("# Solution found: %d \n", num_of_solutions);
            
            std::vector<IKREAL_TYPE> solvalues(num_of_joints);

            // Max number of solutions
            num_of_solutions = (num_of_solutions > 13) ? 13 : num_of_solutions;

            for(std::size_t i = 0; i < num_of_solutions; ++i) {
                // for IKFast 56,61
                const IkSolutionBase<IKREAL_TYPE> &sol = solutions.GetSolution(i);
                int this_sol_free_params = (int)sol.GetFree().size(); 

                // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
                std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

                // for IKFast 56,61
                sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

                for( std::size_t j = 0; j < solvalues.size(); ++j)
                {
                    *q_sols = solvalues[j]; q_sols++;
                    // printf("%.15f, ", solvalues[j]);
                }
                // printf("\n");
            }

            return num_of_solutions;
        } else {
            return 0;
        }
    }

    const std::string getRobotName()
    {
        const char *hash = GetKinematicsHash();

        std::string part = hash;
        part.erase(0, 22);
        std::string name = part.substr(0, part.find(" "));
        return name;
    }
    
    // bool isIKSuccess(const std::vector<double> &pose, std::vector<double> &joints, int &numOfSolns)
    // {
    //     /* Input: pose
    //        Output: joints (if solution is found),
    //                numOfSolns number of solutions,
    //                bool true if solution is found
    //     */

    //     // for IKFast 56,61
    //     unsigned int num_of_joints = GetNumJoints();
    //     unsigned int num_free_parameters = GetNumFreeParameters();


    //     // for IKFast 56,61
    //     IkSolutionList<IKREAL_TYPE> solutions;

    //     std::vector<IKREAL_TYPE> vfree(num_free_parameters);
    //     eetrans[0] = pose[0];
    //     eetrans[1] = pose[1];
    //     eetrans[2] = pose[2];
    //     double qw = pose[6];
    //     double qx = pose[3];
    //     double qy = pose[4];
    //     double qz = pose[5];
    //     const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    //     qw *= n;
    //     qx *= n;
    //     qy *= n;
    //     qz *= n;
    //     eerot[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    //     eerot[1] = 2.0f * qx * qy - 2.0f * qz * qw;
    //     eerot[2] = 2.0f * qx * qz + 2.0f * qy * qw;
    //     eerot[3] = 2.0f * qx * qy + 2.0f * qz * qw;
    //     eerot[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    //     eerot[5] = 2.0f * qy * qz - 2.0f * qx * qw;
    //     eerot[6] = 2.0f * qx * qz - 2.0f * qy * qw;
    //     eerot[7] = 2.0f * qy * qz + 2.0f * qx * qw;
    //     eerot[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;

    //     // TODO: the user have to define the number of free parameters for the manipulator if it has more than 6 joints. So
    //     // currently more than 6 joints are not supported yet.

    //     // for IKFast 56,61
    //     bool b1Success = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    //     // for IKFast 56,61
    //     unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
    //     numOfSolns = num_of_solutions;

    //     joints.resize(num_of_joints);

    //     // for IKFast 56,61
    //     if (!b1Success)
    //     {
    //         return false;
    //     }
    //     else
    //     {
    //         // cout<<"Found ik solutions: "<< num_of_solutions<<endl;
    //         const IkSolutionBase<IKREAL_TYPE> &sol = solutions.GetSolution(0);
    //         int this_sol_free_params = (int)sol.GetFree().size();
    //         if (this_sol_free_params <= 0)
    //         {
    //             sol.GetSolution(&joints[0], NULL);
    //         }
    //         else
    //         {
    //             static std::vector<IKREAL_TYPE> vsolfree;
    //             vsolfree.resize(this_sol_free_params);
    //             sol.GetSolution(&joints[0], &vsolfree[0]);
    //         }
    //         return true;
    //     }
    // }


    // bool isIkSuccesswithTransformedBase(const geometry_msgs::Pose &base_pose,
    //                                                 const geometry_msgs::Pose &grasp_pose, std::vector<double> &joint_soln, int &numOfSolns)
    // {
    //     // Creating a transformation out of base pose
    //     tf2::Vector3 base_vec(base_pose.position.x, base_pose.position.y, base_pose.position.z);
    //     tf2::Quaternion base_quat(base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z,
    //                             base_pose.orientation.w);
    //     base_quat.normalize();
    //     tf2::Transform base_trns;
    //     base_trns.setOrigin(base_vec);
    //     base_trns.setRotation(base_quat);

    //     // Inverse of the transformation
    //     tf2::Transform base_trns_inv;
    //     base_trns_inv = base_trns.inverse();

    //     // Creating a transformation of grasp pose
    //     tf2::Vector3 grasp_vec(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
    //     tf2::Quaternion grasp_quat(grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z,
    //                             grasp_pose.orientation.w);
    //     grasp_quat.normalize();
    //     tf2::Transform grasp_trns;
    //     grasp_trns.setOrigin(grasp_vec);
    //     grasp_trns.setRotation(grasp_quat);

    //     // Transforming grasp pose to origin from where we can check for Ik
    //     tf2::Transform new_grasp_trns;
    //     // new_grasp_trns = grasp_trns * base_trns_inv;
    //     new_grasp_trns = base_trns_inv * grasp_trns;
    //     // Creating a new grasp pose in the origin co-ordinate
    //     std::vector<double> new_grasp_pos;
    //     tf2::Vector3 new_grasp_vec;
    //     tf2::Quaternion new_grasp_quat;
    //     new_grasp_vec = new_grasp_trns.getOrigin();
    //     new_grasp_quat = new_grasp_trns.getRotation();
    //     new_grasp_quat.normalize();
    //     new_grasp_pos.push_back(new_grasp_vec[0]);
    //     new_grasp_pos.push_back(new_grasp_vec[1]);
    //     new_grasp_pos.push_back(new_grasp_vec[2]);
    //     new_grasp_pos.push_back(new_grasp_quat[0]);
    //     new_grasp_pos.push_back(new_grasp_quat[1]);
    //     new_grasp_pos.push_back(new_grasp_quat[2]);
    //     new_grasp_pos.push_back(new_grasp_quat[3]);

    //     // Check the new grasp_pose for Ik
    //     Kinematics k;
    //     //std::vector< double > joints;

    //     //joints.resize(6);
    //     if (k.isIKSuccess(new_grasp_pos, joint_soln, numOfSolns))
    //         return true;
    //     else
    //         return false;
    // }
}; // namespace kinematics
