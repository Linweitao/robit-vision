# robit-vision
Robocup 中型组(Middle-Size League）robot vision

该程序包仅在Ubuntu系统上使用
需安装ROS机器人操作系统

本人使用Ubuntu16.04   ROSkinetic版本

需先独立建立ROS工作空间，在src中加入gige_cap image_processor localization三个程序包，
在工作空间文件夹中加入其他txt或图片视频文件。
（安装ROS并建立工作空间的详细步骤可在ROS官网http://www.ros.org 上获得）

进入工作空间
~$ cd catkin_ws/
编译程序包
~$ catkin_make
