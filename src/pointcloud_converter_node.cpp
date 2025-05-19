#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter() : Node("pointcloud_converter_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered", 10,
      std::bind(&PointCloudConverter::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/output_pointcloud", 10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    sensor_msgs::msg::PointCloud cloud;
    cloud.header = msg->header;

    for (const auto& pt : pcl_cloud.points) {
      geometry_msgs::msg::Point32 p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      cloud.points.push_back(p);
    }

    pub_->publish(cloud);
    RCLCPP_INFO(this->get_logger(), "Published legacy PointCloud with %lu points", cloud.points.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
