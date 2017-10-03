#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include "reachability_map_tools/reachability_rotations.hpp"


static std_msgs::ColorRGBA red_;
static std_msgs::ColorRGBA orange_;
static std_msgs::ColorRGBA yellow_;
static std_msgs::ColorRGBA green_;
static std_msgs::ColorRGBA blue_;
static std_msgs::ColorRGBA light_grey_;

static std::vector<double> reachability_thresholds_ = {0.9, 0.8, 0.5, 0.2, 0.1, 0.01};
//static std::map<uint32_t, std_msgs::ColorRGBA> reachability_threshold_color_values_;
static std::vector<std_msgs::ColorRGBA> reachability_threshold_color_values_;

void initializeStandardColors()
{
    reachability_threshold_color_values_.resize(6);

    red_.r = 1.0f;
    red_.g = 0.0f;
    red_.b = 0.0f;
    red_.a = 1.0f;
    reachability_threshold_color_values_[0] = red_;

    orange_.r = 1.0f;
    orange_.g = 0.5f;
    orange_.b = 0.0f;
    orange_.a = 0.3f;
    reachability_threshold_color_values_[1] = orange_;

    yellow_.r = 1.0f;
    yellow_.g = 1.0f;
    yellow_.b = 0.0f;
    yellow_.a = 0.25f;
    reachability_threshold_color_values_[2] = yellow_;

    green_.r = 0.0f;
    green_.g = 1.0f;
    green_.b = 0.0f;
    green_.a = 0.2f;
    reachability_threshold_color_values_[3] = green_;

    blue_.r = 0.0f;
    blue_.g = 0.0f;
    blue_.b = 1.0f;
    blue_.a = 0.15f;
    reachability_threshold_color_values_[4] = blue_;

    light_grey_.r = 0.8f;
    light_grey_.g = 0.8f;
    light_grey_.b = 0.8f;
    light_grey_.a = 0.1f;
    reachability_threshold_color_values_[5] = light_grey_;

    assert(reachability_threshold_color_values_.size() == reachability_thresholds_.size());
}


VoxelGrid::VoxelGrid<std::vector<std::vector<double>>> readReachabilityMap(const std::string& path)
{
    ROS_INFO_STREAM("Opening file: " << path);
    std::ifstream reachabilty_file(path, std::ios::binary | std::ios::in | std::ios::ate);
    if (!reachabilty_file.is_open())
    {
        throw_arc_exception(std::runtime_error, "Couldn't open file");
    }

    ROS_INFO_STREAM("Reading contents of file: " << path);
    std::streamsize size = reachabilty_file.tellg();
    reachabilty_file.seekg(0, std::ios::beg);
    std::vector<uint8_t> file_buffer((size_t)size);
    if (!(reachabilty_file.read(reinterpret_cast<char*>(file_buffer.data()), size)))
    {
        throw_arc_exception(std::runtime_error, "Unable to read entire contents of file");
    }
    std::vector<uint8_t> decompressed_map = ZlibHelpers::DecompressBytes(file_buffer);

    const auto solution_deserializer = [](const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        return arc_utilities::DeserializeVector<double>(buffer, current, &arc_utilities::DeserializeFixedSizePOD<double>);
    };
    const auto grid_cell_deserializer = [&](const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        return arc_utilities::DeserializeVector<std::vector<double>>(buffer, current, solution_deserializer);
    };

    return VoxelGrid::VoxelGrid<std::vector<std::vector<double>>>::Deserialize(decompressed_map, 0, grid_cell_deserializer).first;
}

visualization_msgs::MarkerArray generateMarkerArray(const std::string& base_frame, const std::string& base_marker_namespace, const VoxelGrid::VoxelGrid<std::vector<std::vector<double>>>& reachability_map)
{
    assert(reachability_map.GetCellSizes()[0] == reachability_map.GetCellSizes()[1]
            && reachability_map.GetCellSizes()[0] == reachability_map.GetCellSizes()[2]);

    ROS_INFO("Generating collision map grid");
    sdf_tools::TaggedObjectCollisionMapGrid grid_for_export(
                reachability_map.GetOriginTransform(),
                base_frame,
                reachability_map.GetCellSizes()[0],
                reachability_map.GetXSize(),
                reachability_map.GetYSize(),
                reachability_map.GetZSize(),
                sdf_tools::TAGGED_OBJECT_COLLISION_CELL());

    for (int64_t x_ind = 0; x_ind < reachability_map.GetNumXCells(); ++x_ind)
    {
        for (int64_t y_ind = 0; y_ind < reachability_map.GetNumXCells(); ++y_ind)
        {
            for (int64_t z_ind = 0; z_ind < reachability_map.GetNumXCells(); ++z_ind)
            {
                std::vector<std::vector<double>> reachability_cell = reachability_map.GetImmutable(x_ind, y_ind, z_ind).first;

                int count = 0;
                for (size_t orientation_ind = 0; orientation_ind < reachability_cell.size(); ++orientation_ind)
                {
                    if (!std::isnan(reachability_cell[orientation_ind][0]))
                    {
                        count++;
                    }
                }

                if (count > 0)
                {
                    const double percent_reachable = (double)count / (double)reachability_cell.size();

                    bool found_level_set = false;
                    for (size_t marker_ind  = 0; marker_ind < reachability_thresholds_.size() && !found_level_set; ++marker_ind)
                    {
                        if (percent_reachable >= reachability_thresholds_[marker_ind])
                        {
                            found_level_set = true;
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL cell_value(1.0, (uint32_t)marker_ind+1);
                            grid_for_export.Set(x_ind, y_ind, z_ind, cell_value);
                        }
                    }
                }
            }
        }
    }

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(reachability_thresholds_.size());
    for (size_t marker_ind = 0; marker_ind < reachability_thresholds_.size(); marker_ind++)
    {
        std::vector<uint32_t> objects_to_render = {(uint32_t)marker_ind+1};
        // marker_array.markers[marker_ind] = grid_for_export.ExportContourOnlyForDisplay(1.0, objects_to_render);
        marker_array.markers[marker_ind] = grid_for_export.ExportForDisplay(1.0, objects_to_render);
        marker_array.markers[marker_ind].ns = base_marker_namespace + std::to_string(reachability_thresholds_[marker_ind]);
        marker_array.markers[marker_ind].colors = std::vector<std_msgs::ColorRGBA>(marker_array.markers[marker_ind].points.size(), reachability_threshold_color_values_[marker_ind]);
    }

    return marker_array;
}

int main(int argc, char **argv)
{
    // Initialze ROS stuff
    ros::init(argc, argv, "reachability_map_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    initializeStandardColors();

    // Read the reachability map into a data structure
    ROS_INFO_STREAM("Private namespace is: " << ros::this_node::getName());
    const std::string data_file = ROSHelpers::GetParam<std::string>(ph, "data_file", "~/catkin_ws/src/reachability_map_tools/cached_maps/iiwa7_reachabilty_4cmgrid_64orientations.map");
    const VoxelGrid::VoxelGrid<std::vector<std::vector<double>>> reachability_map = readReachabilityMap(data_file);

    // Generate a marker array from the grid
    const std::string base_frame = ROSHelpers::GetParam<std::string>(ph, "base_frame", "world");
    const std::string base_marker_namespace = ROSHelpers::GetParam<std::string>(ph, "marker_namespace", "iiwa14_reachability");
    const visualization_msgs::MarkerArray marker_array = generateMarkerArray(base_frame, base_marker_namespace, reachability_map);

    // Publish the map
    ROS_INFO("Publishing marker");
    ros::Publisher reachability_map_pub = nh.advertise<visualization_msgs::MarkerArray>("reachability_map_marker_array", 0);
    ros::Rate loop_rate(2.0);
    while (ros::ok())
    {
        reachability_map_pub.publish(marker_array);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
