#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include "reachability_map_tools/reachability_rotations.hpp"


static std_msgs::ColorRGBA red_;
static std_msgs::ColorRGBA orange_;
static std_msgs::ColorRGBA yellow_;
static std_msgs::ColorRGBA green_;
static std_msgs::ColorRGBA blue_;
static std_msgs::ColorRGBA light_grey_;

static std::vector<double> reachability_thresholds_ = {0.9, 0.8, 0.5, 0.2, 0.1, 0.01};
static std::vector<std_msgs::ColorRGBA> reachability_threshold_color_values_;

void initializeStandardColors()
{
    red_.r = 1.0;
    red_.g = 0.0;
    red_.b = 0.0;
    red_.a = 1.0;
    reachability_threshold_color_values_.push_back(red_);

    orange_.r = 1.0;
    orange_.g = 0.5;
    orange_.b = 0.0;
    orange_.a = 1.0;
    reachability_threshold_color_values_.push_back(orange_);

    yellow_.r = 1.0f;
    yellow_.g = 1.0f;
    yellow_.b = 0.0f;
    yellow_.a = 1.0f;
    reachability_threshold_color_values_.push_back(yellow_);

    green_.r = 0.0;
    green_.g = 1.0;
    green_.b = 0.0;
    green_.a = 1.0;
    reachability_threshold_color_values_.push_back(green_);

    blue_.r = 0.0;
    blue_.g = 0.0;
    blue_.b = 1.0;
    blue_.a = 1.0;
    reachability_threshold_color_values_.push_back(blue_);

    light_grey_.r = 0.8f;
    light_grey_.g = 0.8f;
    light_grey_.b = 0.8f;
    light_grey_.a = 0.5f;
    reachability_threshold_color_values_.push_back(light_grey_);

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
        return arc_helpers::DeserializeVector<double>(buffer, current, &arc_helpers::DeserializeFixedSizePOD<double>);
    };
    const auto grid_cell_deserializer = [&](const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        return arc_helpers::DeserializeVector<std::vector<double>>(buffer, current, solution_deserializer);
    };

    return VoxelGrid::VoxelGrid<std::vector<std::vector<double>>>::Deserialize(decompressed_map, 0, grid_cell_deserializer).first;
}

visualization_msgs::Marker createTemplateMarker(ros::NodeHandle& nh)
{
    visualization_msgs::Marker template_marker;
    const auto time = ros::Time::now();

    template_marker.header.frame_id = ROSHelpers::GetParam<std::string>(nh, "base_frame", "world");;
    template_marker.header.stamp = time;

    template_marker.ns = "reachabilty_map";
    template_marker.id = 0;

    template_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    template_marker.action = visualization_msgs::Marker::MODIFY;

    template_marker.scale.x = 0.005;
    template_marker.scale.y = 0.0;
    template_marker.scale.z = 0.0;

    template_marker.color = red_;

    return template_marker;
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
    const std::string data_file = ROSHelpers::GetParam<std::string>(ph, "data_file", "reachabilty_map.map");
    const VoxelGrid::VoxelGrid<std::vector<std::vector<double>>> reachability_map = readReachabilityMap(data_file);

    // Convert the map into a marker array
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(reachability_thresholds_.size(), createTemplateMarker(ph));
    for (size_t marker_ind = 0; marker_ind < marker_array.markers.size(); ++marker_ind)
    {
        marker_array.markers[marker_ind].color = reachability_threshold_color_values_[marker_ind];
        marker_array.markers[marker_ind].ns += "_" + std::to_string(marker_ind);
    }


    ROS_INFO("Generating marker array");
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

                    size_t marker_ind;
                    for (marker_ind = 0; marker_ind < reachability_thresholds_.size(); marker_ind++)
                    {
                        if (percent_reachable >= reachability_thresholds_[marker_ind])
                        {
                            break;
                        }
                    }

                    if (marker_ind < reachability_thresholds_.size())
                    {
                        std::vector<double> location = reachability_map.GridIndexToLocation(x_ind, y_ind, z_ind);
                        geometry_msgs::Point point;
                        point.x = location[0];
                        point.y = location[1];
                        point.z = location[2];
                        marker_array.markers[marker_ind].points.push_back(point);
                    }
                }
            }
        }
    }

    // Publish the map
    ROS_INFO("Publishing marker array");

    ros::Publisher reachability_map_pub = nh.advertise<visualization_msgs::MarkerArray>("reachability_map_marker_array", 0);
    ros::Rate loop_rate(2.0);
    while (ros::ok())
    {
        const auto time = ros::Time::now();
        for (size_t ind = 0; ind < marker_array.markers.size(); ++ind)
        {
            marker_array.markers[ind].header.stamp = time;
        }

        reachability_map_pub.publish(marker_array);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
