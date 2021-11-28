#ifndef CINEMATICA_FANUC_H
#define CINEMATICA_FANUC_H
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

//// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <tf2/convert.h>
//#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

//#include <eigen3/Eigen/Core>


typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;
typedef Eigen::Matrix<double, 8, 6> Matrix86d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

class Cinematica {

    public:
      Cinematica();

      void JointStateCallback(const sensor_msgs::JointState::ConstPtr &);
      Matrix4d compute_arm_fk (double joint_position[], double joint_velocity[]);
      Eigen::MatrixXd compute_arm_jacobian (double joint_position[], double joint_velocity[]);

      void Spinner();

      double cycle_time;

      double joint_real_position[6];
      double joint_real_velocity[6];

//      trajectory_msgs::JointTrajectory arm_message;

    private:

      ros::NodeHandle nh;
      //Subscribers
      ros::Subscriber joints_state_sub;
      //Publishers
//      ros::Publisher arm_pub;
//      ros::Publisher mobile_pub;

      sensor_msgs::JointState joint_state;

      // ---- MoveIt Robot Model ---- //
      robot_model_loader::RobotModelLoader robot_model_loader;
      robot_model::RobotModelPtr kinematic_model;
      robot_state::RobotStatePtr kinematic_state;
      const robot_state::JointModelGroup *joint_model_group;
      std::vector<std::string> joint_names;


};

#endif /* CINEMATICA_FANUC_H */
