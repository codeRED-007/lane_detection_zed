#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("point_cloud_transform")
    {
        // Create a subscriber for the input point cloud
        rclcpp::QoS qos_profile(10); // 10 is the depth of the queue
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points", qos_profile,
            std::bind(&PointCloudTransformer::cloud_callback, this, std::placeholders::_1));

        // Create a publisher for the output point cloud
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lane_detection/cloud_transformed", 10);

        // Create a TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Spin periodically to handle TF lookups and callbacks
    //     timer_ = this->create_wall_timer(
    //         std::chrono::milliseconds(30),
    //         std::bind(&PointCloudTransformer::timer_callback, this));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        // Create a container for the data
        if (!got_transform) {
            try {
                transform = tf_buffer_->lookupTransform("base_link", "zed_left_camera_optical_frame", rclcpp::Time(0));
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could NOT transform: %s", ex.what());
                return;
            }
            got_transform = true;

        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        
        
        
        pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);
        
        

        // Convert the transformed cloud to ROS message and publish
        sensor_msgs::msg::PointCloud2 cloud_publish;
        
        pcl::toROSMsg(*cloud_transformed, cloud_publish);
        cloud_publish.header = input->header;
        cloud_publish.header.frame_id = "base_link";

        pub_->publish(cloud_publish);
    }

    
    bool got_transform = false;
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<PointCloudTransformer>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
