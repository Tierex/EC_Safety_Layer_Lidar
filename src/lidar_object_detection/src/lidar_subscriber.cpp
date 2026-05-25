#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarSubscriber : public rclcpp::Node
{
public:
    LidarSubscriber() : Node("lidar_subscriber")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&LidarSubscriber::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Lidar subscriber gestart!");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "PointCloud ontvangen! Points: width=%d height=%d",
                    msg->width, msg->height);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
