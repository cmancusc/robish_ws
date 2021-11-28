# fanuc_control -----------------------------------------------------------------------------------------------------------------

Package that contains all the controllers that is needed to move and control the motions of the following robots: 
	- Fanuc M-710iC/45M
	- Fanuc R-2000iC/270F 


# fanuc_description -------------------------------------------------------------------------------------------------------------

Package that contains the URDF fie and all the meshes of the Fanuc M-710iC/45M and R-2000iC/270F robot. 
You can use this package to spawn the robot in Rviz and move the joints with joint state publisher gui.


# fanuc_gazebo ------------------------------------------------------------------------------------------------------------------

Package that contains the gazebo world and a launch file to spawn the Fanuc M-710iC/45M and R-2000iC/270F robot in Gazebo.


# fanuc_m710ic45m_moveit --------------------------------------------------------------------------------------------------------

Package that contains all the files that is needed to move and control the Fanuc M-710iC/45M robot through the Moveit package.


# fanuc_r2000ic270f_moveit ------------------------------------------------------------------------------------------------------

Package that contains all the files that is needed to move and control the Fanuc R-2000iC/270F robot through the Moveit package.


### Dependencies ----------------------------------------------------------------------------------------------------------------

 * rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic

 * gazebo9


## Running the tests ------------------------------------------------------------------------------------------------------------

Launch simulation in Gazebo:

	```roslaunch fanuc_gazebo fanuc_m710ic45m_world.launch```

equivantly

	```roslaunch fanuc_gazebo fanuc_r2000ic270f_world.launch```


Launch simulation in Rviz:

	```roslaunch fanuc_m710ic45m_moveit moveit_planning_execution.launch```

equivantly

	```roslaunch fanuc_m710ic45m_moveit moveit_planning_execution.launch```


## Version ----------------------------------------------------------------------------------------------------------------------

* **ROS --> Melodic**

## Authors ----------------------------------------------------------------------------------------------------------------------

* **Mancus Cristian**

## Contact ----------------------------------------------------------------------------------------------------------------------

* **cristian.mancus@unimore.it**




https://github.com/MatthewVerbryke/inmoov_ros

https://github.com/AS4SR/general_info/wiki/Basic-ROS-MoveIt!-and-Gazebo-Integration

https://gramaziokohler.github.io/compas_fab/latest/examples/03_backends_ros/07_ros_create_urdf_ur5_with_measurement_tool.html

http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html


