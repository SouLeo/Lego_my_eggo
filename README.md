# EE 382V: Human Robot Interaction

### Authors: Christina Petlowany, Chris Suarez, Selma Wanna
### email: slwanna@utexas.edu

## Table of Contents
1. [About](#about)
2. [Installation](#installation)

## About
Lorem Ipsum nonsense

## Installation
1. Navigate to your home directory and type
    ```
    git clone https://github.com/UTNuclearRobotics/Lego_my_eggo
    ```

2. Then Install SMACH packages from ROS, open a terminal and type
    ```
    sudo apt-get install ros-kinetic-smach*
    ```

3. Compile the packages
    ```
    cd ~/Lego_my_eggo
    catkin build
    ```
    *It's all python right now, so if it doesn't compile, email me...

4. Adjust your bashrc
    ```
    cd ~/
    vi ~/.bashrc
    ```
   Write the following at the end of the file
   ```
   source ~/Lego_my_eggo/devel/setup.bash
   ```
## Run
1. To run just the FSM
    ```
    roslaunch Lego_my_eggo FSM.launch
    ```
    *Currently you need to publish to is_block_placed topic in order for it to
    change states
