#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <cmath>  // for std::isnan
#include <Eigen/Dense>

class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter() : Node("pointcloud_converter_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered", 10,
      std::bind(&PointCloudConverter::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/output_pointcloud", 10);
    last_time_ = this->now();  // 초기화
  }

private:
  rclcpp::Time last_time_;
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto current_time = this->now();
    double time_diff_sec = (current_time - last_time_).seconds();
    last_time_ = current_time;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // 1. 좌표변환 행렬 정의
    Eigen::Matrix4f transform;
    transform << 1, 0, 0,  0.165,
                 0, 1, 0,  0.000,
                 0, 0, 1,  0.320,
                 0, 0, 0,  1;

    // 2. 변환 적용
    pcl::PointCloud<pcl::PointXYZ> pcl_transformed;
    pcl::transformPointCloud(pcl_cloud, pcl_transformed, transform);
    
    // 3. 변환된 결과를 sensor_msgs::PointCloud로 변환
    sensor_msgs::msg::PointCloud cloud;
    cloud.header = msg->header;

    for (const auto& pt : pcl_transformed.points) {
      if (std::isnan(pt.x)) {
        continue;  // NaN 포인트는 제외
      }
      geometry_msgs::msg::Point32 p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      cloud.points.push_back(p);
    }

    pub_->publish(cloud);
    RCLCPP_INFO(this->get_logger(), "Published legacy PointCloud with %lu points | Δt = %.3f Hz", cloud.points.size(), 1/time_diff_sec);
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
