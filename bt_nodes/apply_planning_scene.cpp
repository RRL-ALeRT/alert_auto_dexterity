#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "octomap_msgs/msg/octomap.hpp"

using namespace BT;

class ApplyPlanningScene: public RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>
{
public:
  ApplyPlanningScene(const std::string& name,
          const NodeConfig& conf,
          const RosNodeParams& params)
    : RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<std::string>("objects"),
      InputPort<std::string>("operation"),
      InputPort<std::shared_ptr<octomap_msgs::msg::Octomap>>("octomap")
    });
  }

  bool setRequest(Request::SharedPtr& request) override
  {
    auto operation = getInput<std::string>("operation").value();

    if (operation != "octomap")
    {
      auto objects = getInput<std::string>("objects").value();
      auto object_vec = splitString(objects, ';');
      float CUBE_SIZE = 0.2;

      for (const auto& object: object_vec)
      {
        moveit_msgs::msg::CollisionObject scene_object;
        scene_object.header.frame_id = object;
        scene_object.id = object;
        if (operation == "add")
          scene_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        else
          scene_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        // Define the geometry of the cube
        shape_msgs::msg::SolidPrimitive cube_primitive;
        cube_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        cube_primitive.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE}; // (x, y, z)

        float NEGATIVE_OFFSET = 0.05;

        // Define the pose of the cube
        geometry_msgs::msg::Pose cube_pose;
        cube_pose.position.x = 0.0;
        cube_pose.position.y = 0.0;
        cube_pose.position.z = (CUBE_SIZE / 2) - NEGATIVE_OFFSET;

        scene_object.primitives.push_back(cube_primitive);
        scene_object.primitive_poses.push_back(cube_pose);

        request->scene.world.collision_objects.push_back(scene_object);
      }
    }
    else
    {
      auto octomap_ptr = getInput<std::shared_ptr<octomap_msgs::msg::Octomap>>("octomap").value();
      auto octomap = *octomap_ptr;
      request->scene.world.octomap.header = octomap.header;
      request->scene.world.octomap.octomap = octomap;
    }
    request->scene.is_diff = true;

    return true;
  };

  void halt() override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  };

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onResponseReceived", name().c_str());

    if (response.get()->success)
      return NodeStatus::SUCCESS;
      
    return NodeStatus::FAILURE;
  };

  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(ApplyPlanningScene, "ApplyPlanningScene");