#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;
using std::placeholders::_1;

class BinaryThresholding : public rclcpp::Node {
public:
    BinaryThresholding() : Node("lane_follower") {   
        this->declare_parameter("bt_low", 200);
        this->declare_parameter("bt_high", 255);

        bt_low = this->get_parameter("bt_low").as_int();
        bt_high = this->get_parameter("bt_high").as_int();

        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/rgb/image_rect_color", 10, 
            std::bind(&BinaryThresholding::binary_thresholding, this, std::placeholders::_1));
        
        subscription_depth = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/depth/depth_registered", 10, 
            std::bind(&BinaryThresholding::depthCallback, this, _1));

        subscription_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed/zed_node/depth/camera_info", 10, 
            std::bind(&BinaryThresholding::cameraInfoCallback, this, _1));

        depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection/thresholded_depth", 10);
        camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("/lane_detection/camera_info", 10);
    }

private:
    void binary_thresholding(const sensor_msgs::msg::Image::SharedPtr msg) {   
        if (!depthCallCheck || !cam_info_received) return;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
        if (!cv_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert the image");
            return;
        }

        cv::Mat cv_image = cv_ptr->image, gray_image, thresholded_image;
        int rows = cv_image.rows;
        int cols = cv_image.cols;

        cv::medianBlur(cv_image, cv_image, 5);
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY); 

        // Mask out the top quarter of the image
        for (int y = 0; y < rows / 4; ++y) {
            for (int x = 0; x < cols; ++x) {
                gray_image.at<uchar>(y, x) = 0;
            }
        }   
        cv::threshold(gray_image, thresholded_image, bt_low, bt_high, cv::THRESH_BINARY);
        cv::Mat mask;
        thresholded_image.convertTo(mask, CV_8U); // Ensure mask is 8-bit

        depthImage.setTo(0, ~mask);

        // Add timestamp to the published depth image
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "zed_left_camera_optical_frame"; // Set frame ID if needed

        auto depth_msg = cv_bridge::CvImage(header, "32FC1", depthImage).toImageMsg();
        depth_publisher->publish(*depth_msg);

        // Update and publish camera info
        camera_info.header.stamp = header.stamp;
        camera_info_publisher->publish(camera_info);

        cv::imshow("Depth Image", depthImage);
        cv::imshow("Thresholded Image", thresholded_image);

        depthCallCheck = false;
        cam_info_received = false;
        cv::waitKey(1);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            depthImage = cv_bridge::toCvCopy(msg, "32FC1")->image; // Or "32FC1" if depth is in float
            depthCallCheck = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert Depth image: %s", e.what());
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info = *msg;
        cam_info_received = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_info;
    sensor_msgs::msg::CameraInfo camera_info;
    bool cam_info_received = false;

    cv::Mat depthImage;
    bool depthCallCheck = false;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;

    int bt_low;
    int bt_high;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BinaryThresholding>());
    rclcpp::shutdown();
    return 0;
}
