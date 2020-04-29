# DD_robot
This is a ROS robot implementation, that use Gazebo simulation environment.
The description of robot is given in /urdf/my_robot_x.urdf.xacro file, that include other xacro files and run gazebo tag.

If you want to spawn robot and all controller in gazebo model:
    $ roslaunch my_robot_urdf my_robot_gazebo.launch

If you want to command robot by keyboard arrows:
    $ sudo apt-get install ros-melodic-teleop-twist-keyboard
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=DD_controller/cmd_vel 

For the trajectory tracking:
    $ rosrun my_robot_urdf trajectory_control
