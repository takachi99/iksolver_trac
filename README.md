# Introduction

## Demo:



[![Alt text](https://github.com/haxhimitsu/iksolver_track/blob/master/readme_material/preview.png)](https://www.youtube.com/watch?v=Cliyr5ubmo0)


## Dependencies
* Ubuntu 18.04, 20.04
* ROS melodic
* [universal_robot](https://github.com/ros-industrial/universal_robot)
## Installation
~~~
cd ~/catkin_ws/src
git clone git@github.com:haxhimitsu/iksolver_track.git
rosdep install -i --from-paths path-to-ros-package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
~~~

## Launch file
* Cartesian position controller.
  * input : target frame(pose,orientation) from joy stick
  * output : each targt joit potision solved track_ik
```
roslaunch trac_ik_examples send_joy_frame.launch mode:=position
```
mode:=position\
upload joy node and realtime ik solver.\
target frame input from joy stick is directory sent realtime_ik node.\

 realtime ik solver node is``trac_ik_jointpub.cpp``

* Cartesian position and force controller
  * input : target frame(pose,orientation) and force(each axis x,y,z)
  * target frame treat with pose_and_force_PID_controller

  ```
  roslaunch trac_ik_examples send_joy_frame.launch mode:=position_and_force
  ```
  and run position and force PID controller.
  ```
  rosrun trac_ik_examples pos_force_controller
  ```
  
  ##  Requirements
  In order to publish target frame, you have to prepare [joystick controller](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f310-gamepad.940-000137.html)
  
  If you don't have joy stick controller, you can publish message from terminal
  ### for position controll
    * send target frame
      - msg_type:``geometry_msgs/PoseStamped``
      - send to : ``/end_effector_pose``

    ```
    rostopic pub -r 500 /end_effector_pose geometry_msgs/PoseStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: 'base_link'
    pose:
    position:
        x: 0.56
        y: 0.028
        z: 0.611
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
    ```
    and you run 
    ```
    rosrun  trac_ik_examples trac_ik_jointpub
    ```
    instead of ```roslaunch trac_ik_examples send_joy_frame.launch mode:=position```

### for position and force controller
 * send target frame and force.
   * msg_type:``std_msgs_Float32MultiArray``
   * send to :```\array```
    ```
    rostopic pub  -r 500 /array std_msgs/Float32MultiArray "layout:
    dim:
    - label: ''
        size: 10
        stride: 0
    data_offset: 0
    data:
    - 0.56
    - 0.028
    - 0.611
    - 0.0
    - 0.0
    - 0.0
    - 1.0
    - 0.0
    - 0.0
    - 0.00
    "
    ```
    and run
    ```
    rosrun trac_ik_examples pos_force_controller
    ```
    finally  run ik_solver
    ```
    rosrun trac_ik_examples trac_ik_jointpub
    ```
    instead of ```roslaunch trac_ik_examples send_joy_frame.launch mode:=position_and_force```

*  Note : each valuable set as
      ```
        rostopic pub  -r 500 /array std_msgs/Float32MultiArray "layout:
        dim:
        - label: ''
            size: 10
            stride: 0
        data_offset: 0
        data:
        - pose.x
        - pose.y
        - pose.z
        - rotation.x
        - rotation.y
        - rotation.z
        - rotation.w
        - force.x
        - force.y
        - force.z
        "
      ```
      force ranges limited -9<value<9 at ```pos_force_controller.cpp```
## How to set joint limit

if you use ```roslaunch ur_gazebo ur5.launch limited:=true```, IK provide multiple result.
For this reason, Robot may be vibrate.

One of the most simple solution is  editing  joint limits at ```ur5_joint_limited_robot.urdf.xacro```.

```
  <!-- arm -->
  <xacro:ur5_robot prefix="" joint_limited="true"
    shoulder_pan_lower_limit="${-pi/4.0}" shoulder_pan_upper_limit="${pi/4.0}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi*0.0}"
    elbow_joint_lower_limit="${-pi*0.0}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi*1.3}" wrist_1_upper_limit="${-pi/3.0}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi*0.0}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />
```


  ## Reference
* This repository reference [trac_ik](https://bitbucket.org/traclabs/trac_ik/src/master/)