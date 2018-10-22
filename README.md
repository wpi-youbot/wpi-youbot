# wpi-youbot
Robot Control Grad Project Fall 2018  


Installation
------------

### System requirements: 
* Linux 16.04 LTS
* ROS Kinetic
* Gazebo 7


## Installation instructions


Go to your catkin workspace.  
Clone the kinetic-devel branch of the repository we forked from the Team Luhbots:

    git clone -b kinetic-devel https://github.com/wpi-youbot/luh_youbot_os


Clone this repository to your catkin workspace:

    git clone -b kinetic-devel https://github.com/wpi-youbot/wpi-youbot

Install the following dependencies:

    sudo apt-get install libnlopt-dev   
    sudo apt-get install gksu

#####Please report any additional dependencies missing.


Build your workspace

Launch the YouBot base simulation program from the LUH package:  

    roslaunch luh_youbot_gazebo base_only.launch

Launch a sample program from our package:  

        roslaunch wpi-youbot launch.launch
        
