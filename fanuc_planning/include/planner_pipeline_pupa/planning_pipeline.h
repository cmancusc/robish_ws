/* Author: Mancus Cristian */
#ifndef PLANNING_PIPELINE_H_
#define PLANNING_PIPELINE_H_
#define _USE_MATH_DEFINES

#include "planner_pipeline_pupa/robish_scene_class.h"

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

namespace rvt = rviz_visual_tools;

class MyPlanningPipeline
{
      public:
        MyPlanningPipeline();
        ~MyPlanningPipeline();

        Robish_Scene* scena_rviz;

        void Spin();

//        std::ofstream traj_file;


      private:  

        // ros::NodeHandle node_handle("~");
        ros::NodeHandle node_handle;
        ros::Publisher display_publisher;

        std::vector<double> joint_positions;
        std::vector<std::string> joint_names;

        tf2::Quaternion q;

        moveit_msgs::PlanningScene scena_caricata;

        //############ MoveIt Robot Model ################//

        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr robot_model;
        const robot_state::JointModelGroup *joint_model_group;

        const robot_model::JointModel* joint_model;

        planning_scene::PlanningScenePtr planning_scene;
        planning_pipeline::PlanningPipelinePtr planning_pipeline;

        moveit_visual_tools::MoveItVisualTools visual_tools;

        moveit_msgs::DisplayTrajectory display_trajectory;
};

#endif /* PLANNING_PIPELINE_H_ */
