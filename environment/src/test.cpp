#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

// import mesh into the rviz
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <tf/transform_broadcaster.h>

moveit_msgs::AttachedCollisionObject add_object(std::string obj_attached_link, std::string obj_frame_id, std::string obj_id, std::string mesh_file_path, double x, double y, double z, double roll, double pitch, double yaw){
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = obj_attached_link;
  attached_object.object.header.frame_id = obj_frame_id;
  attached_object.object.id = obj_id;
  // Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);
  shapes::Mesh* m = shapes::createMeshFromResource(mesh_file_path);
  ROS_INFO("mesh loaded");
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf::Quaternion objectsQuaternionTransformation;
  objectsQuaternionTransformation.setRPY(roll,pitch,yaw);
  pose.orientation.w = objectsQuaternionTransformation.w();
  pose.orientation.x = objectsQuaternionTransformation.x();
  pose.orientation.y = objectsQuaternionTransformation.y();
  pose.orientation.z = objectsQuaternionTransformation.z();

  attached_object.object.meshes.resize(1);
  attached_object.object.mesh_poses.resize(1);
  attached_object.object.meshes.push_back(mesh);
  attached_object.object.mesh_poses.push_back(pose);

  attached_object.object.operation = attached_object.object.ADD;

  // by default - the link_name is already considered by default
  attached_object.touch_links = std::vector<std::string>{"link_6"};

  return attached_object;
}

moveit_msgs::AttachedCollisionObject add_regular_shape(std::string obj_attached_link, std::string obj_frame_id, std::string obj_id, double x, double y, double z, double l, double h, double w){
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = obj_attached_link;
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = obj_frame_id;

  attached_object.object.id = obj_id;

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = l;
  primitive.dimensions[1] = h;
  primitive.dimensions[2] = w;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  attached_object.object.operation = attached_object.object.ADD;

  attached_object.touch_links = std::vector<std::string>{"link_6"};
  return attached_object;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_environment");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  moveit_msgs::AttachedCollisionObject attached_object = add_object("", "base_link", "conveyor", "file:///home/chionn/ma4825_ws/src/environment/stl/conveyert_belt_with_rack_recentered.stl",-0.25, -0.07, 0.0, 0.0, 0.0, 1.571);
  // moveit_msgs::AttachedCollisionObject attached_object2 = add_regular_shape("", "base_link", "box", 0.0,0.0,0.1,3,3,3);
  moveit_msgs::AttachedCollisionObject attached_object2 = add_object("", "base_link", "container", "file:///home/chionn/ma4825_ws/src/environment/stl/basin.stl",0,0.225,0.0,0.0,0.0,1.571);

  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.world.collision_objects.push_back(attached_object2.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  ros::shutdown();
  return 0;
}

  // from Tiong Hee
  // moveit_msgs::CollisionObject collision_object;

  // collision_object.header.frame_id = "base_link";
  // collision_object.id = "cup";
  // Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);
  // shapes::Mesh* m = shapes::createMeshFromResource("file:///home/jayden99/Documents/robotics_v5/src/environment/stl/cup.stl", vectorScale); 
  // ROS_INFO("table mesh loaded");

  // shape_msgs::Mesh mesh;
  // shapes::ShapeMsg mesh_msg;  
  // shapes::constructMsgFromShape(m, mesh_msg);
  // mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  // collision_object.meshes.resize(1);
  // collision_object.mesh_poses.resize(1);
  // collision_object.meshes.push_back(mesh);
  // collision_object.mesh_poses.push_back(pose);
  // std::vector<moveit_msgs::CollisionObject> collision_objects_; 
  // collision_objects_.push_back(collision_object);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // planning_scene_interface.addCollisionObjects(collision_objects_); 
