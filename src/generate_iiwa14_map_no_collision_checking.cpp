#define IKFAST_HAS_LIBRARY

#include <iiwa14/ikfast.h>
#include <omp.h>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include "reachability_map_tools/reachability_rotations.hpp"

const std::vector<double> lower_limits = {
    -2.96705972839,
    -2.09439510239,
    -2.96705972839,
    -2.09439510239,
    -2.09439510239,
    -2.09439510239,
    -3.05432619099
};
const std::vector<double> upper_limits = {
    2.96705972839,
    2.09439510239,
    2.96705972839,
    2.09439510239,
    2.09439510239,
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

std::vector<double> ComputeIKSolutions(const IkReal* trans, const Eigen::Matrix3d& rot_eigen)
{
    IkReal rot[9] = {
        rot_eigen.data()[0], rot_eigen.data()[3], rot_eigen.data()[6],
        rot_eigen.data()[1], rot_eigen.data()[4], rot_eigen.data()[7],
        rot_eigen.data()[2], rot_eigen.data()[5], rot_eigen.data()[8]
    };

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
                    return solution;
                }

            }
        }
    }

    return std::vector<double>(7, NAN);
}

int main()
{
    const auto orientations = ReachabilityMapTools::getRotations();

    const double resolution = 0.04;
    const double radius = 1.3;
    VoxelGrid::VoxelGrid<std::vector<std::vector<double>>> grid(resolution, 2.0*radius, 2.0*radius, 2.0*radius, std::vector<std::vector<double>>(6, std::vector<double>(7, NAN)));

    const int max_ind = (int)std::ceil(radius/resolution);
    for (int x_ind = -max_ind; x_ind <= max_ind; ++x_ind)
    {
        const double x = x_ind*resolution;

        for (int y_ind = -max_ind; y_ind <= max_ind; ++y_ind)
        {
            const double y = y_ind*resolution;

            #pragma omp parallel for schedule(guided)
            for (int z_ind = -max_ind; z_ind <= max_ind; ++z_ind)
            {
                const double z = z_ind*resolution;

                if (x*x + y*y + z*z <= radius*radius)
                {
                    const IkReal trans[3] = {x, y, z};
                    std::vector<std::vector<double>> multiple_orientation_solutions(orientations.size());
                    for (size_t orientation_ind = 0; orientation_ind < orientations.size(); ++orientation_ind)
                    {
                        multiple_orientation_solutions[orientation_ind] = ComputeIKSolutions(trans, orientations[orientation_ind].matrix());
                    }

                    grid.SetValue(x, y, z, multiple_orientation_solutions);
                }
            }
        }
    }

    const auto solution_serializer = [](const std::vector<double>& joint_vals, std::vector<uint8_t>& buffer)
    {
        return arc_helpers::SerializeVector<double>(joint_vals, buffer, &arc_helpers::SerializeFixedSizePOD<double>);
    };
    const auto grid_cell_serializer = [&](const std::vector<std::vector<double>>& list, std::vector<uint8_t>& buffer)
    {
        return arc_helpers::SerializeVector<std::vector<double>>(list, buffer, solution_serializer);
    };

    std::vector<uint8_t> buffer;
    grid.SerializeSelf(buffer, grid_cell_serializer);

    // Compress and save to file
    const std::vector<uint8_t> compressed_serialized_data = ZlibHelpers::CompressBytes(buffer);
    std::ofstream output_file("iiwa14_reachabilty.map", std::ios::out | std::ios::binary);
    uint64_t serialized_size = compressed_serialized_data.size();
    output_file.write(reinterpret_cast<const char*>(compressed_serialized_data.data()), (std::streamsize)serialized_size);
    output_file.close();

    return 0;
}
