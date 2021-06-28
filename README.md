# Introduction

## Demo:



[![Alt text](https://github.com/haxhimitsu/iksolver_track/blob/master/readme_material/preview.png)](https://www.youtube.com/watch?v=Cliyr5ubmo0)


## Dependencies
* Ubuntu 18.04, 20.04
* ROS melodic
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
mode:=0\
upload joy node and raltime ik solver.\
target frame from joy stick is directory sent realtime_ik node.


* Cartesian position and force controller
  * input : target frame(pose,orientation) and force(each axis x,y,z)
  * target frame treat with pose_and_force_PID_controller

  ```
  roslaunch trac_ik_examples send_joy_frame.launch mode:=position_and_force
  ```
  and run position and force PID controller.
  ```
  rosrun trac_ik_examples poa_force_controller
  ```
  
  ##  Requirements
  In order to publish target frame, you have to prepare [joystick controller](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f310-gamepad.940-000137.html)
  
  If you don't have joy stick controller, you can publish message from terminal
  ### for position controll
    * send target frame
      - msg_type:``geometry_msgs/PoseStamped``
      - send to : ``/end_effector_pose``

    ```
    rostopic pub -r 10500 /end_effector_pose geometry_msgs/PoseStamped "header:
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
    rosrun  trac_ik_examples trac_ik_jointpub03
    ```
    instead of ```roslaunch trac_ik_examples send_joy_frame.launch mode:=mode0```

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
    rosrun trac_ik_examples trac_ik_jointpub03
    ```
    instead of ```roslaunch trac_ik_examples send_joy_frame.launch mode:=mode1```

*  note:each valiable set as

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
  ## Reference
* This repository referenced [trac_ik](https://bitbucket.org/traclabs/trac_ik/src/master/)