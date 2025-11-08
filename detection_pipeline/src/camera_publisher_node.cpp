#include "../include/armor_detector/camera_publisher_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

CameraPublisherNode::CameraPublisherNode() : Node("camera_publisher_node"), frame_count(0)
{
    cap = std::make_unique<cv::VideoCapture>(videoPath, cv::CAP_FFMPEG);

    if (!cap->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open the video camera: %s", videoPath.c_str());
        rclcpp::shutdown();
        return;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    read_timer = this->create_wall_timer(
        std::chrono::milliseconds(33), // ~30 FPS
        std::bind(&CameraPublisherNode::publish_frame, this));
}

void CameraPublisherNode::publish_frame()
{
    cv::Mat frame;
    if (!cap->read(frame)) {
        // Loop the video instead of stopping
        RCLCPP_WARN_ONCE(this->get_logger(), "End of video reached. Looping back to start.");
        cap->set(cv::CAP_PROP_POS_FRAMES, 0);

        // Shut down if the video cannot restart
        if (!cap->read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Cannot restart video. Shutting down.");
            rclcpp::shutdown();
            return;
        }
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";

    publisher_->publish(*msg);

    frame_count++;
    RCLCPP_INFO(this->get_logger(), "Published frame %d", frame_count);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

