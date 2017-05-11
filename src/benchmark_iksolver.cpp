#define IKFAST_HAS_LIBRARY

#include <iiwa7/ikfast.h>
#include <omp.h>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include "reachability_map_tools/reachability_rotations.hpp"
#include <iomanip>
#include <arc_utilities/iiwa_7_fk_fast.hpp>
#include <arc_utilities/pretty_print.hpp>

#define L0_length    0.34
#define L1_length    0.40
#define L2_length    0.40
#define L3_length    0.126
#define PI M_PI

const std::vector<double> lower_limits = {
    -2.96705972839,
    -2.09439510239,
    -2.96705972839,
    -2.09439510239,
    -2.96705972839,
    -2.09439510239,
    -3.05432619099
};
const std::vector<double> upper_limits = {
    2.96705972839,
    2.09439510239,
    2.96705972839,
    2.09439510239,
    2.96705972839,
    2.09439510239,
    3.05432619099
};
const int free_ind = GetFreeParameters()[0];


bool CheckJointLimits(const std::vector<IkReal>& solution)
{
    bool in_imits = true;
    for (size_t j = 0; j < 7; ++j)
    {
        in_imits &= lower_limits[j] <= solution[j] && solution[j] <= upper_limits[j];
    }
    return in_imits;
}

std::vector<double> ConvertIKSolutionToStdVector(const ikfast::IkSolutionBase<IkReal>& solution_ikreal)
{
    std::vector<IkReal> solution_vals(7);
    solution_ikreal.GetSolution(&solution_vals[0], nullptr);

    #ifdef IKFAST_REAL
        std::vector<double> result(7);
        for (size_t j = 0; j < 7; ++j)
        {
            result[j] = solution_vals[j];
        }
        return result;
    #else
        return solution_vals;
    #endif
}

// std::vector<double> ComputeIKSolutions(const IkReal* trans, const Eigen::Matrix3d& rot_eigen)
// {
// //    std::cout << rot_eigen << std::endl;
//     IkReal rot[9] = {
//         rot_eigen.data()[0], rot_eigen.data()[1], rot_eigen.data()[2],
//         rot_eigen.data()[3], rot_eigen.data()[4], rot_eigen.data()[5],
//         rot_eigen.data()[6], rot_eigen.data()[7], rot_eigen.data()[8]
//     };

//     const double free_min = lower_limits[free_ind];
//     const double free_max = upper_limits[free_ind];
//     for (double free_val = free_min; free_val <= free_max; free_val += 0.001)
//     {
//         const IkReal free_joint_val = free_val;
//         ikfast::IkSolutionList<IkReal> solutions;
//         const bool success = ComputeIk(trans, rot, &free_joint_val, solutions);
//         if (success)
//         {
//             for (size_t sol_ind = 0; sol_ind < solutions.GetNumSolutions(); ++sol_ind)
//             {

//                 const std::vector<double> solution = ConvertIKSolutionToStdVector(solutions.GetSolution(sol_ind));
//                 const bool in_joint_limits = CheckJointLimits(solution);

//                 if (in_joint_limits)
//                 {
//                     return solution;
//                 }

//             }
//         }
//     }

//     return std::vector<double>(7, NAN);
// }

//compute all IK solutions
std::vector<std::vector<double>> ComputeIKSolutions_all(const IkReal* trans, const Eigen::Matrix3d& rot_eigen) {
    std::vector<std::vector<double>> solutions_all;

    IkReal rot[9] = {
        rot_eigen.data()[0], rot_eigen.data()[1], rot_eigen.data()[2],
        rot_eigen.data()[3], rot_eigen.data()[4], rot_eigen.data()[5],
        rot_eigen.data()[6], rot_eigen.data()[7], rot_eigen.data()[8]
    };

    std::cout<<"In ComputeIKSolutions:\n"<<PrettyPrint::PrettyPrint(rot_eigen.data())<<std::endl;
    std::cout<<PrettyPrint::PrettyPrint(rot)<<std::endl;

    const double free_min = lower_limits[free_ind];
    const double free_max = upper_limits[free_ind];
    for (double free_val = free_min; free_val <= free_max; free_val += 0.001)
    {
        const IkReal free_joint_val = free_val;
        ikfast::IkSolutionList<IkReal> solutions;
        const bool success = ComputeIk(trans, rot, &free_joint_val, solutions);
        if (success)
        {
            for (size_t sol_ind = 0; sol_ind < solutions.GetNumSolutions(); ++sol_ind)
            {

                const std::vector<double> solution = ConvertIKSolutionToStdVector(solutions.GetSolution(sol_ind));
                const bool in_joint_limits = CheckJointLimits(solution);

                if (in_joint_limits)
                {
                    solutions_all.push_back(solution);
                }

            }
        }
    }
    return solutions_all;
}

Eigen::Quaterniond construct_quaterniond_with_angle (const double& angle, const double& x, const double& y, const double& z) {
    const Eigen::Vector3d axis_vector = EigenHelpers::SafeNormal(Eigen::Vector3d(x, y, z));
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis_vector));
}

void print_matrix(const Eigen::Matrix3d& rot_eigen) {
    std::cout<<"Print_matrix\n";
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            std::cout<<rot_eigen.data()[i*3+j]<<'\t';
        }
        std::cout<<'\n';
    }
}

void print_solution (const std::vector<double>& this_solution) {
    for (size_t i=0; i<this_solution.size(); i++) {
        std::cout<<"Joint"<<i+1<<": "<<this_solution[i]<<"\t";
    }
    std::cout<<std::endl;
}

double get_angle_from_x_y (const double& x, const double& y) {
    if (x>0) {
        return atan(y/x);
    }

    if (x<0) {
        if (y>0) {
            return PI-asin(y/sqrt(x*x+y*y));
        }
        else {
            return -PI+asin(-y/sqrt(x*x+y*y));
        }
    }
    if (x==0) {
        return y>0? PI/2 : -PI/2;
    }

    return 0;
}

bool check_joint_angle_limit (std::vector<double>& solution, int start_ind, int end_ind) {
    for (int i=start_ind; i<=end_ind; i++) {
        solution[i]=fmod(solution[i]+PI, 2*PI) - PI;
        if ((solution[i]<lower_limits[i]) || (solution[i]>upper_limits[i])) {
            return false;
        }
    }
    return true;
}

void calculate_joint_angles_567 (const Eigen::Vector3d& L3, const Eigen::Vector3d& L4, std::vector<double> solution,
                                std::vector<std::vector<double>>& solutions_all) {
    for (int i=0; i<2; i++) {
        if (i==0) {
            solution[4]=get_angle_from_x_y(L3[0],L3[1]); //joint angle 5, acos(y/x)
            solution[5]=acos(L3[2]/L3.norm()); //joint angle 6, asin(z/length)
        }
        else {    //change parameter and caluculate again
            solution[4]=solution[4]+PI;
            solution[5]=-solution[5];            
        }
        // std::cout<<"Solution for 56:\n";
        // print_solution(solution);

        if (check_joint_angle_limit(solution, 4, 5)) {    
            Eigen::Quaterniond rot5 = construct_quaterniond_with_angle (solution[4], 0, 0, 1.0);
            Eigen::Quaterniond rot6 = construct_quaterniond_with_angle (solution[5], 0, 1.0, 0);
            Eigen::Matrix3d rotmat_L3 = rot5.matrix()*rot6.matrix();
            Eigen::Vector3d L4_new = rotmat_L3.inverse() * L4;
            solution[6] = get_angle_from_x_y(L4_new[0], L4_new[1]);
            // std::cout<<"Solution for 7:\n";
            // print_solution(solution);
            if (check_joint_angle_limit(solution, 6, 6))
                solutions_all.push_back(solution);
        }
    }
}

void calculate_joint_angles_34 ( const Eigen::Vector3d& L2, const Eigen::Vector3d& L3, const Eigen::Vector3d& L4, std::vector<double> solution,
                                std::vector<std::vector<double>>& solutions_all) {
    for (int i=0; i<2; i++) {
        if (i==0) {
            solution[2]=get_angle_from_x_y(L2[0],L2[1]); //joint angle 3, acos(y/x)
            solution[3]=-acos(L2[2]/L2.norm()); //joint angle 4, asin(z/length)
        }
        else {    //change parameter and caluculate again
            solution[2]=solution[2]+PI;
            solution[3]=-solution[3];    
        }
        // std::cout<<"Solution for 34:\n";
        // print_solution(solution);
        if (check_joint_angle_limit(solution, 2, 3)) { 
            Eigen::Quaterniond rot3 = construct_quaterniond_with_angle (solution[2], 0, 0, 1.0);
            Eigen::Quaterniond rot4 = construct_quaterniond_with_angle (solution[3], 0, -1.0, 0);
            Eigen::Matrix3d rotmat_L2 = rot3.matrix()*rot4.matrix();
            Eigen::Matrix3d inverse_mat = rotmat_L2.inverse();
            calculate_joint_angles_567 (inverse_mat*L3, inverse_mat*L4, solution, solutions_all);
        }
    }
}

void calculate_joint_angles_12 (const Eigen::Vector3d& L1, const Eigen::Vector3d& L2, const Eigen::Vector3d& L3, const Eigen::Vector3d& L4,
                                std::vector<std::vector<double>>& solutions_all) {
    std::vector<double> solution(7);
    for (int i=0; i<2; i++) {
        if (i==0) {
            solution[0]=get_angle_from_x_y(L1[0],L1[1]); //joint angle 1, acos(y/x)
            solution[1]=acos(L1[2]/L1.norm()); //joint angle 2, asin(z/length)
        }
        else{    //change parameter and caluculate again
            solution[0]=solution[0]+PI;
            solution[1]=-solution[1];           
        }
        // std::cout<<"Solution for 12:\n";
        // print_solution(solution);
        if (check_joint_angle_limit(solution, 0, 1)) {
            Eigen::Quaterniond rot1 = construct_quaterniond_with_angle (solution[0], 0, 0, 1.0);
            Eigen::Quaterniond rot2 = construct_quaterniond_with_angle (solution[1], 0, 1.0, 0);
            Eigen::Matrix3d rotmat_L1 =  rot1.matrix()*rot2.matrix();
            Eigen::Matrix3d inverse_mat = rotmat_L1.inverse();
            calculate_joint_angles_34 (inverse_mat*L2, inverse_mat*L3, inverse_mat*L4, solution, solutions_all);
        }
    }
}




void calculate_joint_angles (const Eigen::Vector3d& L1, Eigen::Vector3d L2, Eigen::Vector3d L3, const Eigen::Matrix3d& rot_eigen,
                                std::vector<std::vector<double>>& solutions_all) {
    calculate_joint_angles_12(L1, L2, L3, rot_eigen*Eigen::Vector3d(1, 0, 0), solutions_all);
}

//using 3line method to compute all IK solutions
std::vector<std::vector<double>> ComputeIKSolutions_all_3line(const Eigen::Vector3d& trans, const Eigen::Matrix3d& rot_eigen) {
    std::vector<std::vector<double>> solutions_all(0);
    //calculate line1 offset

    Eigen::Vector3d L3=rot_eigen*Eigen::Vector3d(0, 0, L3_length);
    Eigen::Vector3d L12 = trans-Eigen::Vector3d(0, 0, L0_length)-L3; //trans for line1+line2

    //throw overrange cases
    const double L12_length=L12.norm();
    if ((L12_length>L1_length+L2_length) || (L12_length<L1_length)) {
        return solutions_all;
    }

    //construct rotation matrix about L12
    const int num_of_angles=1000;
    const double rot_angle=2*PI/double(num_of_angles);
    Eigen::Quaterniond rotation_12 = construct_quaterniond_with_angle(rot_angle, 
        L12[0]/L12_length, L12[1]/L12_length, L12[2]/L12_length); //set to 60 degree, change later
    Eigen::Matrix3d rotmat_12=rotation_12.matrix();

    //from midpoint of line12 to point 2
    Eigen::Vector3d Lto2;
    if (L12[1]!=0) {
        Lto2 = Eigen::Vector3d(1,-L12[0]/L12[1],0); 
    }
    else if (L12[2]!=0) {
        Lto2 = Eigen::Vector3d(0,1,-L12[1]/L12[2]); 
    }
    else {
        Lto2 = Eigen::Vector3d(-L12[2]/L12[0],0,1); 
    }
    Eigen::Vector3d L12mid=L12/2;
    Lto2=Lto2/Lto2.norm()*sqrt(L1_length*L1_length-L12_length*L12_length/4);
    //construct point 2
    Eigen::Vector3d P2;
    for (int i=0; i<num_of_angles; i++) {
        P2=L12mid+Lto2;
        Eigen::Vector3d L1=P2;
        Eigen::Vector3d L2=L12-L1;
        // calculate joint angles
        calculate_joint_angles(L1,L2,L3,rot_eigen,solutions_all);
        Lto2=rotmat_12*Lto2;
    }
    return solutions_all;


}


double dist(const std::vector<double>& sol1, const std::vector<double>& sol2) {
    double dist=0;
    for (int i=0; i<6; i++) {
        dist+=pow(sol1[i]-sol2[i],2);
    }
    return sqrt(dist);
}

//compare between two sets of solutions
double compare_solution(const std::vector<std::vector<double>>& solution_IK, const std::vector<std::vector<double>>& solution_3line) {
    //compare each solution with the ground truth
    double min_max_dist=0;
    for (size_t i=0; i<solution_3line.size(); i++) {
        double min_dist=100000;
        for (size_t j=0; j<solution_IK.size(); j++) {
            double this_dist=dist(solution_3line[i],solution_IK[j]);
            if (this_dist<min_dist) {
                min_dist=this_dist;
            }
        }
        if (min_dist>min_max_dist) {
            min_max_dist=min_dist;
        }
        // std::cout<<"min_dist: "<<min_dist<<'\n';
    }
    return min_max_dist;
}

void show_results(const std::vector<double>& iiwa_7_base_config) { 

    const EigenHelpers::VectorAffine3d iiwa_7_link_transforms = IIWA_7_FK_FAST::GetLinkTransforms(iiwa_7_base_config);
    std::cout << "IIWA 7 Link transforms:\n" << PrettyPrint::PrettyPrint(iiwa_7_link_transforms, false, "\n") << std::endl;
}

int main()
{
    const auto orientations = ReachabilityMapTools::getRotations();
    Eigen::Quaterniond this_rotation = construct_quaterniond_with_angle(PI/2, 0, 1, 0);
    // Eigen::Quaterniond this_rotation_3line = construct_quaterniond_with_angle(0, 0, 1, 0);   
    std::cout <<"Transform matrix:\n" << this_rotation.matrix() << std::endl <<std::endl;
    std::cout <<"Transform quaterniond:\n" << PrettyPrint::PrettyPrint(this_rotation) << std::endl <<std::endl;

    std::vector<std::vector<double>> solution_IK;
    std::vector<std::vector<double>> solution_3line;
    std::cout<<std::fixed<<std::setprecision(3);
    const Eigen::Vector3d trans_vec(L1_length+L3_length, 0.0, L2_length+L0_length);
    const IkReal trans[3] = {trans_vec[0], trans_vec[1], trans_vec[2]};

    solution_IK = ComputeIKSolutions_all(trans, this_rotation.matrix());
    // solution_3line = ComputeIKSolutions_all_3line(trans_vec, this_rotation_3line.matrix());

    // std::cout<<"3line solution Size: "<<solution_3line.size()<<'\n';   
    std::cout<<"IK solution Size: "<<solution_IK.size()<<'\n';

    std::cout<<"Result for IK solver:\n";
    print_solution(solution_IK[0]);
    show_results(solution_IK[0]);

    // Eigen::Vector3d L1(L1_length*sqrt(3)/2, 0, L1_length/2);
    // Eigen::Vector3d L2(0, L2_length, 0);
    // Eigen::Vector3d L3(0, 0, L3_length);
    // this_rotation = construct_quaterniond_with_angle(PI/2, 0, 0, 1);
    // calculate_joint_angles(L1, L2, L3, this_rotation.matrix(), solution_3line);

    // std::cout<<"Final 3line solution:\n";
    // for (size_t i=0; i<solution_3line.size(); i++) {
    //     print_solution(solution_3line[i]);
    // }

    // std::cout<<"Final IK solution:\n";
    // for (size_t i=0; i<solution_IK.size(); i++) {
    //     print_solution(solution_IK[i]);
    // }




    // solution_3line = ComputeIKSolutions_all_3line(trans_vec, this_rotation.matrix());



    // std::cout<<"Following is a small portion of the result for the IK solver:\n";
    // for (size_t i=0; i<3; i++) {
    //     print_solution(solution_3line[i]);
    // }


    // std::cout<<"\nVerify a portion of solutoins using forward kinematics:\n";
    // for (size_t i=0; i<1; i++) {
    //     print_solution(solution_3line[i]);
    //     show_results(solution_3line[i]);
    // }



    // std::cout<<"\nHausdorff distance: Compare 3line solution to IK solution:\n";   
    // std::cout<<compare_solution(solution_IK, solution_3line)<<'\n';
    // std::cout<<"Hausdorff distance: Compare IK solution to 3line solution:\n";   
    // std::cout<<compare_solution(solution_3line, solution_IK)<<'\n';

    // std::cout<<"The rotation input is:\n"<<PrettyPrint::PrettyPrint(this_rotation)<<std::endl;
    return 0;


}
