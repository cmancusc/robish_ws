# robish_ws
Git clone this repository in order to download and install all the controllers and planning pipeline code for the following industrial manipulators:
<table>
    <tr>
        <td>Fanuc M710iC-45M robot</td>
    </tr>
</table>
<table>
    <tr>
	<td>Fanuc R2000iC-270f robot</td>
    </tr>
</table>


## Packages
This workspace contains different packages that allow you to describe the configuration of the manipulator, its controller system and other useful things that makes you able to planning optimized trajectories for the manipulator


### fanuc_control
Package that contains all the controllers that is needed to move and control the motions of the following robots: 
	- Fanuc M-710iC/45M
	- Fanuc R-2000iC/270F 


### fanuc_description 
Package that contains the URDF fie and all the meshes of the Fanuc M-710iC/45M and R-2000iC/270F robot. 
You can use this package to spawn the robot in Rviz and move the joints with joint state publisher gui.


### fanuc_gazebo
Package that contains the gazebo world and a launch file to spawn the Fanuc M-710iC/45M and R-2000iC/270F robot in Gazebo.


### fanuc_m710ic45m_moveit
Package that contains all the files that is needed to move and control the Fanuc M-710iC/45M robot through the Moveit package.


### fanuc_r2000ic270f_moveit 
Package that contains all the files that is needed to move and control the Fanuc R-2000iC/270F robot through the Moveit package.


## Build 
In order to build the overall workspace you can run the following commands through the command line: <br />
>	rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic <br />
>	cd ~/your_catkin_ws <br />
>	catkin_make <br />


## Running the tests 
Launch simulation in Gazebo:

	```roslaunch fanuc_gazebo fanuc_m710ic45m_world.launch```

equivantly

	```roslaunch fanuc_gazebo fanuc_r2000ic270f_world.launch```


Launch simulation in Rviz:

	```roslaunch fanuc_m710ic45m_moveit moveit_planning_execution.launch```

equivantly

	```roslaunch fanuc_m710ic45m_moveit moveit_planning_execution.launch```


## Version 
* **ROS --> Melodic**

## Authors 
* **Mancus Cristian**

## Contact 
* **cristian.mancus@unimore.it**

