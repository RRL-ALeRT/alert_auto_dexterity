#include <mutex>
#include <functional>
#include <memory>

#include "depth_image_proc/visibility.h"
#include "image_geometry/pinhole_camera_model.h"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <depth_image_proc/conversions.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace enc = sensor_msgs::image_encodings;

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

class PointCloudXyzNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PointCloudXyzNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : LifecycleNode("PointCloudXyzNode", options)
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

    depth_image_sub_ = create_subscription<Image>("image_rect", 1, 
      std::bind(&PointCloudXyzNode::depthCb, this, std::placeholders::_1));

    depth_image_info_sub_ = create_subscription<CameraInfo>("camera_info", 1, 
      std::bind(&PointCloudXyzNode::depthInfoCb, this, std::placeholders::_1));

    pub_point_cloud_ = create_publisher<PointCloud2>("points", 1);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    LifecycleNode::on_activate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&)
  {
    depth_image_sub_.reset();
    depth_image_info_sub_.reset();
    pub_point_cloud_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state)
  {
    depth_image_sub_.reset();
    depth_image_info_sub_.reset();
    pub_point_cloud_.reset();
    
    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());
      
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void depthInfoCb(const CameraInfo::SharedPtr info_msg)
  {
    // Update camera model
    model_.fromCameraInfo(info_msg);

    info_msg_updated = true;
  }

  void depthCb(const Image::SharedPtr depth_msg)
  {
    if (!pub_point_cloud_->is_activated()) return;
    if (!info_msg_updated) return;

    auto cloud_msg = std::make_shared<PointCloud2>();
    cloud_msg->header = depth_msg->header;
    cloud_msg->height = depth_msg->height;
    cloud_msg->width = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    // Convert Depth Image to Pointcloud
    if (depth_msg->encoding == enc::TYPE_16UC1) {
      depth_image_proc::convertDepth<uint16_t>(depth_msg, cloud_msg, model_);
    } else if (depth_msg->encoding == enc::TYPE_32FC1) {
      depth_image_proc::convertDepth<float>(depth_msg, cloud_msg, model_);
    } else {
      RCLCPP_ERROR(
        get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
      return;
    }

    pub_point_cloud_->publish(*cloud_msg);
  };

private:
  // Subscriptions
  rclcpp::Subscription<Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_image_info_sub_;

  // Publications
  std::mutex connect_mutex_;
  rclcpp_lifecycle::LifecyclePublisher<PointCloud2>::SharedPtr pub_point_cloud_;

  bool info_msg_updated = false;
  image_geometry::PinholeCameraModel model_;
};

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<PointCloudXyzNode> node = std::make_shared<PointCloudXyzNode>();

  exe.add_node(node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}