/* Author: Mancus Cristian */
#ifndef ROBISH_SCENE_CLASS
#define ROBISH_SCENE_CLASS
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <string.h>
#include <vector>
#include <iostream>

#include "Eigen/Core"

// MoveIt messeges
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <geometric_shapes/shape_operations.h>

class Robish_Scene
{
public:

  Robish_Scene();
//  void Spin();
  moveit_msgs::PlanningScene Spin();
  void attachObject(std::string blk_name, std::string address, std::string frame_attach, double pose[], double orientation[], Eigen::Vector3d scala);
  void makeBox(std::string blk_name, std::string frame_attach, double dimensions[], double pose[]);
  void addMesh1(std::string blk_name, std::string address, std::string frame_attach, double pose[], double orientation[]);
  void addMesh(std::string blk_name, std::string address, std::string frame_attach, double pose[], double orientation[]);
//  void addObjects();
  void removeObjects();

private:

  ros::NodeHandle node_handle;
  ros::Publisher planning_scene_diff_publisher;
  ros::ServiceClient planning_scene_diff_client;
  ros::WallDuration sleep_t;

  moveit_visual_tools::MoveItVisualTools visual_tools;

  moveit_msgs::PlanningScene planning_scene;
  moveit_msgs::ApplyPlanningScene srv;

  tf2::Quaternion q;

};

#endif /* ROBISH_SCENE_CLASS */
