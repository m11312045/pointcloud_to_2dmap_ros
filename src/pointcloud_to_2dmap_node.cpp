#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MapGenerator
{
public:
    double resolution;
    double m2pix;
    int map_width;
    int map_height;
    int min_points_in_pix;
    int max_points_in_pix;
    double min_height;
    double max_height;
    std::string dest_directory;
    std::string input_pcd;
    std::string map_name;

    cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ> &cloud) const
    {
        cv::Mat map(map_height, map_width, CV_8UC1, cv::Scalar::all(0));
        for (const auto &point : cloud)
        {
            if (point.z < min_height || point.z > max_height)
            {
                continue;
            }
            int x = static_cast<int>(point.x * m2pix + map_width / 2);
            int y = static_cast<int>(-point.y * m2pix + map_height / 2);
            if (x < 0 || x >= map_width || y < 0 || y >= map_height)
            {
                continue;
            }
            map.at<uchar>(y, x)++;
        }
        map -= min_points_in_pix;
        map.convertTo(map, CV_8UC1, -255.0 / (max_points_in_pix - min_points_in_pix), 255);

        return map;
    }
};

class PointCloudTo2DMapNode : public rclcpp::Node
{
public:
    PointCloudTo2DMapNode(): Node("pointcloud_to_2dmap_node")
    {
        declare_parameters();
        if (!load_parameters()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameters.");
            rclcpp::shutdown();
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_generator_.input_pcd, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the input cloud: %s", map_generator_.input_pcd.c_str());
            rclcpp::shutdown();
        }
        RCLCPP_DEBUG(this->get_logger(), "Read PointCloud OK!");
        cv::Mat map = map_generator_.generate(*cloud);
        save_map(map);
    }

private:
    MapGenerator map_generator_;

    void declare_parameters()
    {
        declare_parameter("resolution", 0.1);
        declare_parameter("map_width", 10240);
        declare_parameter("map_height", 10240);
        declare_parameter("min_points_in_pix", 2);
        declare_parameter("max_points_in_pix", 5);
        declare_parameter("min_height", 0.5);
        declare_parameter("max_height", 3.0);
        declare_parameter("dest_directory", "");
        declare_parameter("input_pcd", "");
        declare_parameter("map_name", "map");
    }

    bool load_parameters()
    {
        get_parameter("resolution", map_generator_.resolution);
        get_parameter("map_width", map_generator_.map_width);
        get_parameter("map_height", map_generator_.map_height);
        get_parameter("min_points_in_pix", map_generator_.min_points_in_pix);
        get_parameter("max_points_in_pix", map_generator_.max_points_in_pix);
        get_parameter("min_height", map_generator_.min_height);
        get_parameter("max_height", map_generator_.max_height);
        get_parameter("dest_directory", map_generator_.dest_directory);
        get_parameter("input_pcd", map_generator_.input_pcd);
        get_parameter("map_name", map_generator_.map_name);

        map_generator_.m2pix = 1.0 / map_generator_.resolution;

        RCLCPP_INFO(this->get_logger(), "Resolution: %f", map_generator_.resolution);
        RCLCPP_INFO(this->get_logger(), "Map width: %d", map_generator_.map_width);
        RCLCPP_INFO(this->get_logger(), "Map height: %d", map_generator_.map_height);
        RCLCPP_INFO(this->get_logger(), "Destination directory: %s", map_generator_.dest_directory.c_str());
        RCLCPP_INFO(this->get_logger(), "Input PCD file: %s", map_generator_.input_pcd.c_str());
        RCLCPP_INFO(this->get_logger(), "Map name: %s", map_generator_.map_name.c_str());

        return true;
    }

    void save_map(const cv::Mat& map)
    {
        std::filesystem::create_directories(map_generator_.dest_directory);
        std::string map_path = map_generator_.dest_directory + "/" + map_generator_.map_name + ".png";
        cv::imwrite(map_path, map);
        RCLCPP_INFO(this->get_logger(), "Saved 2D map as PNG: %s", map_path.c_str());

        std::string yaml_path = map_generator_.dest_directory + "/map.yaml";
        std::ofstream ofs(yaml_path);
        ofs << "image: " << map_generator_.map_name << ".png" << std::endl;
        ofs << "resolution: " << map_generator_.resolution << std::endl;
        ofs << "origin: [" << -map_generator_.resolution * map_generator_.map_width / 2
            << ", " << -map_generator_.resolution * map_generator_.map_height / 2 << ", 0.0]" << std::endl;
        ofs << "occupied_thresh: 0.5" << std::endl;
        ofs << "free_thresh: 0.2" << std::endl;
        ofs << "negate: 0" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Saved YAML file: %s", yaml_path.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTo2DMapNode>());
    rclcpp::shutdown();
    return 0;
}
