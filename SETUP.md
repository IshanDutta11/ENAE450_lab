# Cleanup
1) Disconnect from everything
2) Turn off robot
3) Disconnect battery and put in charging bin
4) Put robot on shelf

# Turtlebot3 Setup
Get Ubuntu or a VM or something, I don't know how to fix Windows, Mac might actually be fine. Have ROS2 Humble and get the following packages. 
```
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install python3-colcon-common-extensions

echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

# Repo Setup (DO SECOND STEP IF YOU HAVE CLONED ALREADY)
Make sure you have Humble and the turtlebot set up. Source your ROS2. Clone this respository. You will need to build the workspace from the turtlebot3_ws directory then source the overlay. Source overlay each time you build. 
```
source ~/.bashrc #Assuming the gazebo and humble sources are here
git clone --recurse-submodules https://github.com/IshanDutta11/ENAE450_lab.git #clones repo and updates submodules
cd ENAE450_lab/turtlebot3_ws
colcon build
source install/setup.bash
```

Run at least once if you cloned without doing the previous step, the submodules need to be cloned and updated separately. You want to be in the root folder.
```
source ~/.bashrc
git init 
git pull
git submodule init
git submodule update

cd turtlebot3_ws
colcon build
source install/setup.bash
```


# Connecting to Lab Wifi
login/ssid: RAL_robots

alternate login/ssid: RAL_robots_5G

password: 
```
RAL2022robots
```

# Environment Configuration and Turtlebot3 Connection
Call the following commands. Take note of the bot ID. 
```
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=waffle_Pi
export ROS_DOMAIN_ID=(x) #where (x) is replaced by the ID number of the bot 
```
Make sure you aren't connected to a VPN. SSH into the turtlebot. 
```
ssh mrcTB@192.168.50.10(x) #where (x) is replaced by the ID number of the bot 
```
password:
```
turtlebot
```

Exit from SSH with ctrl-d 


# Run Stuff (on turtlebot terminal)
Some commands we have used so far
```
ros2 launch turtlebot3_bringup robot_rplidar.launch.py

ros2 run turtlebot3_teleop teleop_key
```


# Run Stuff (on host laptop terminal)
Some commands we have used so far
```
rqt

ros2 run lab2_package move_node
```


# Setup bashrc with commands, custom functions, and aliases
Some of mine:
```
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh

turtlesetup () {
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=waffle_Pi
export ROS_DOMAIN_ID="$1"
printf "ID: $1 \n"
}

turtleconnect () {
turtlesetup "$1"
ssh mrcTB@192.168.50.10"$1"
}

turtlebuild () {
colcon build
source install/setup.bash
}

alias sourcews="source install/local_setup.bash"
alias editbash="gedit ~/.bashrc"
alias bashrc="source ~/.bashrc"
```
