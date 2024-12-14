#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


using namespace pcl;
using namespace std;

class PointCloudPrinter : public rclcpp::Node
{
public:
    PointCloudPrinter()
        : Node("point_cloud_printer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered",
            10,
            std::bind(&PointCloudPrinter::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lane_point_cloud", 10);
        publisher_bl = this->create_publisher<sensor_msgs::msg::PointCloud2>("lane_point_cloud_bl", 10);

        this->declare_parameter<float>("bt_low", 200.0);
        this->declare_parameter<float>("bt_high", 255.0);
        this->get_parameter("bt_low", bt_low_);
        this->get_parameter("bt_high", bt_high_);

        got_transform = false;
    }

private:
    float bt_low_;
    float bt_high_;
    bool got_transform = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped transform_stamped;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_bl;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    uint8_t convertRGBtoGray(uint8_t r, uint8_t b, uint8_t g) {
        return 0.299 * r + 0.587 * g + 0.114 * b;
    }

    pcl::PointXYZ transformToBaseLink(pcl::PointXYZ point, geometry_msgs::msg::TransformStamped transform_stamped) {
        geometry_msgs::msg::PointStamped point_in, point_out;
        // point_in.header.frame_id = "base_link";
        point_in.point.x = point.x;
        point_in.point.y = point.y;
        point_in.point.z = point.z;

        tf2::doTransform(point_in, point_out, transform_stamped);

        // Update the point cloud with transformed points
        point.x = point_out.point.x;
        point.y = point_out.point.y;
        point.z = point_out.point.z;

        return point;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the incoming PointCloud2 message to a PCL point cloud
        if (!got_transform) {
            try {
                transform_stamped = tf_buffer_.lookupTransform("base_link", "zed_left_camera_frame", tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could NOT transform: %s", ex.what());
                return;
            }
            got_transform = true;

        }

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Create a new point cloud for publishing
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud, base_link_cloud;

        // Process each point and append to the filtered cloud if it meets the criteria
        for (auto &point : cloud.points)
        {
            float x = point.x, y = point.y, z = point.z;
            std::uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
            uint8_t r = (rgb >> 16) & 0xFF;
            uint8_t g = (rgb >> 8) & 0xFF;
            uint8_t b = rgb & 0xFF;
            uint8_t gray = convertRGBtoGray(r, g, b);

            if (gray > bt_low_ && gray < bt_high_)
            {
                pcl::PointXYZ filtered_point, point_base_link;
                filtered_point.x = x;
                filtered_point.y = y;
                filtered_point.z = z;
                // tf2::doTransform(filtered_point, point_base_link, transform_stamped);
                point_base_link = transformToBaseLink(filtered_point, transform_stamped);
                filtered_cloud.push_back(filtered_point);
                base_link_cloud.push_back(point_base_link);
            }
        }

        // Convert the filtered cloud back to a ROS message
        sensor_msgs::msg::PointCloud2 output, output_baselink;
        pcl::toROSMsg(filtered_cloud, output);
        pcl::toROSMsg(base_link_cloud, output_baselink);

        output.header.frame_id = msg->header.frame_id;
        output_baselink.header.frame_id = "base_link";

        // Publish the filtered point cloud
        publisher_->publish(output);
        publisher_bl->publish(output_baselink);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPrinter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
