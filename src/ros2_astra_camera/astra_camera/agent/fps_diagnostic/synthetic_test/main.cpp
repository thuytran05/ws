#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "synthetic_camera");

    auto qos = rclcpp::SensorDataQoS();

    auto rgb_pub = node->create_publisher<Image>(
        "/camera/color/image_raw", qos);

    auto depth_pub = node->create_publisher<Image>(
        "/camera/depth/image_raw", qos);

    auto rgb_info_pub = node->create_publisher<Info>(
        "/camera/color/camera_info", qos);

    auto depth_info_pub = node->create_publisher<Info>(
        "/camera/depth/camera_info", qos);

    Image rgb;
    rgb.width = 640;
    rgb.height = 480;
    rgb.encoding = "rgb8";
    rgb.step = 640 * 3;
    rgb.data.resize(640 * 480 * 3);

    Image depth;
    depth.width = 640;
    depth.height = 480;
    depth.encoding = "16UC1";
    depth.step = 640 * 2;
    depth.data.resize(640 * 480 * 2);

    Info info;
    info.width = 640;
    info.height = 480;

    int count = 0;
    auto last = Clock::now();

    auto timer = node->create_wall_timer(
        33333333ns,
        [&]() {
            auto stamp = node->now();

            rgb.header.stamp = stamp;
            depth.header.stamp = stamp;
            info.header.stamp = stamp;

            rgb.data[0]++;
            depth.data[0]++;

            rgb_pub->publish(rgb);
            rgb_info_pub->publish(info);

            depth_pub->publish(depth);
            depth_info_pub->publish(info);

            count++;

            auto now = Clock::now();

            double elapsed =
                std::chrono::duration<double>(
                    now - last).count();

            if (elapsed >= 5.0) {
                std::cout
                    << "Publisher FPS: "
                    << count / elapsed
                    << std::endl;

                count = 0;
                last = now;
            }
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
