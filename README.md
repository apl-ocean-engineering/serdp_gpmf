# serdp_gpmf

Contains libraries and sample binaries for encoding and decoding SERDP sensor (e.g., [liboculus]()) data 
in GPMF streams.



<!--
Serdp_player .mov decoder to read .movs/.mp4s and decode them into GPMF and image formats display, publish as ROS messages, and encode as ROS bags.   

Code consists of both [ROS](https://www.ros.org/) and (limited-functionality) [fips](https://github.com/floooh/fips) compatabile tools.

## ROS
### Install
To install, clone [dependent repositories](https://gitlab.com/apl-ocean-engineering/aploe_ros_repos/blob/master/blackmagic_oculus.repos) to a catkin_ws/src and import using [vcs](https://github.com/dirk-thomas/vcstool). Building using catkin_make or catkin_tools (catkin build).  

*NOTE* You need to manually initialize the gpmf submodule:  
$ cd <ws>/src/gpmf  
$ git submodule init  
$ git submodule update  

### Running
The ROS code consists on two nodes: serdp_player_ros and mov_to_bag.   
1. serdp_plyaer_ros: Parses a specified  .mov into GPMF and imgs, (optionally) displays images, and publishes proper ROS messages.  
2. mov_to_bag: Subscribes to ROS image types and sonar_image message types and writes to specified bags.  

To run, use launch file and point to specified .mov locations:  

1. serdp_player_ros: roslaunch serdp_player serdp_player.launch mov_filename:=<path_to_mov/.mp4>  
2. roslaunch serdp_player mov_to_bag.launch mov_filename:=<path_to_mov/.mp4> bag_filename:=<pah_to_bag_save_location>/out.bag  

Run roslaunch --ros-args serdp_player <launch_name> for full argument list.  


## fips
Main tool only reads and displays .mov code

### Install
Clone this repostory into a workspace  
Fips makes it easy to build:
$./fips set config linux_make_release
$./fips build

### Running
$./fips run serdp_player -- <path_to_mov>  

The mov should now be displayed -->

# License

Licensed under [BSD 3-clause license](LICENSE).
