cmake_minimum_required(VERSION 2.8.3)
project(fanuc_planning)


add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
eigen_conversions
moveit_core
moveit_ros_planning_interface
moveit_visual_tools
moveit_planners_ompl
interactive_markers
moveit_ros_planning
moveit_ros_perception
pluginlib
geometric_shapes
pcl_ros
pcl_conversions
rosbag
tf2_ros
tf2_eigen
tf2_geometry_msgs)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
  	${Boost_INCLUDE_DIR})

add_executable(planner src/path_planner_standalone.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(trajsender src/moveit_trajectory_publisher.cpp)
target_link_libraries(trajsender ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(run src/planner_01/run.cpp src/planner_01/planning_robish.cpp include/planner_01/planning_robish.h)
target_link_libraries(run ${catkin_LIBRARIES})

add_executable(move_group_interface_tutorial src/planner_00/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(traj_record src/traj_record.cpp)
target_link_libraries(traj_record ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cinematica_fanuc_node src/cinematica/cinematica_fanuc_node.cpp
                                     src/cinematica/cinematica_fanuc.cpp)
target_link_libraries(cinematica_fanuc_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(motion_planning_tutorial src/planner_00/motion_planning.cpp)
target_link_libraries(motion_planning_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(planning_node src/planner_02/planning_node.cpp src/planner_02/planning.cpp include/planner_02/planning.h)
target_link_libraries(planning_node  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(planning_pipeline_node src/planner_pipeline_pupa/planning_pipeline_node.cpp
                                      src/planner_pipeline_pupa/planning_pipeline.cpp
				      src/planner_pipeline_pupa/robish_scene_class.cpp
                                      include/planner_pipeline_pupa/planning_pipeline.h
				      include/planner_pipeline_pupa/robish_scene_class.h)
target_link_libraries(planning_pipeline_node  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(planning_scene_prova_node src/planner_pipeline/planning_scene_prova_node.cpp)
target_link_libraries(planning_scene_prova_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(planning_prova_node src/planner_pipeline/planning_prova_node.cpp
                                   src/planner_pipeline_pupa/robish_scene_class.cpp
                                   include/planner_pipeline_pupa/robish_scene_class.h)
target_link_libraries(planning_prova_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(planning_prova_node_02 src/planner_pipeline/planning_prova_node_02.cpp
                                   src/planner_pipeline_pupa/robish_scene_class.cpp
                                   include/planner_pipeline_pupa/robish_scene_class.h)
target_link_libraries(planning_prova_node_02 ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(planning_prova_node_03 src/planner_pipeline/planning_prova_node_03.cpp
                                   src/planner_pipeline_pupa/robish_scene_class.cpp
                                   include/planner_pipeline_pupa/robish_scene_class.h)
target_link_libraries(planning_prova_node_03 ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(robish_planning_node src/robish_planning/robish_planning_node.cpp
                                    src/planner_pipeline_pupa/robish_scene_class.cpp
                                    include/planner_pipeline_pupa/robish_scene_class.h)
target_link_libraries(robish_planning_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})