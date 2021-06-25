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
* cartesian position controller.
  * input : target frame(pose,orientation) from joy stick
  * output : each targt joit potision solved track_ik
```
roslaunch trac_ik_examples send_joy_frame.launch mode:=mode0
```
mode:=0\
upload joy node and raltime ik solver.\
target frame from joy stick is directory sent realtime_ik node.


* cartesian position and force controller
  * input : target frame(pose,orientation) and force(each axis x,y,z)
  * target frame treat with pose_and_force_PID_controller

  ```
  roslaunch trac_ik_examples send_joy_frame.launch mode:=mode1
  ```
  and run position and force PID controller.
  ```
  rosrun trac_ik_examples poa_force_controller
  ```
## Reference
* This repository referenced [trac_ik](https://bitbucket.org/traclabs/trac_ik/src/master/)