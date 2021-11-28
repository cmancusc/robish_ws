#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometric_shapes/shape_operations.h>

#include <std_msgs/ColorRGBA.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/Pose.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string.h>

namespace rvt = rviz_visual_tools;

class PlanningRobish
{
      public:
        PlanningRobish();

        void goToPoseGoal(geometry_msgs::Pose &pose);
        void PickJoint();
        void PickCartesian();
        void goToJointState();
        void PickandPlace();
        void resetValues();
        void attachObject(std::string blk_name, std::string address, std::string frame_attach, double pose[], double orientation[], Eigen::Vector3d scala);
        void addObjects();
        void makeBox(std::string blk_name, double dimensions[], double pose[]);
        void addMesh1(std::string blk_name, std::string address, std::string frame_attach, double pose[], double orientation[]);
        void addMesh(std::string blk_name, std::string address, std::string frame_attach, double pose[], double orientation[]);
        void removeObjects();

        //            namespace rvt = rviz_visual_tools;

//        std::ofstream traj_file;


      private:

        ros::NodeHandle nh;

        const std::string PLANNING_GROUP = "fanuc_arm";

        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface virtual_world;
        const robot_state::JointModelGroup* joint_model_group;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::vector<double> joint_positions;

        const std::string ROOT_LINK = "world";
        moveit_visual_tools::MoveItVisualTools visual_tools;
        //            rviz_visual_tools::RvizVisualTools visual_tools;

        geometry_msgs::Pose target_pose1;
        geometry_msgs::Pose target_pose_home;

        tf2::Quaternion q;
};

