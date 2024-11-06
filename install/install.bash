sudo apt update
sudo apt install ros-$ROS_DISTRO-moveit
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-$ROS_DISTRO-combined-robot-hw
sudo apt install ros-$ROS_DISTRO-moveit-servo
sudo apt install ros-$ROS_DISTRO-moveit-visual-tools
sudo apt install ros-$ROS_DISTRO-ros-controllers

git clone https://github.com/xArm-Developer/xarm_ros2.git ../../xarm_ros -b $ROS_DISTRO --recursive 
#git clone https://github.com/ros-planning/moveit_task_constructor.git ../../moveit_task_constructor -b $ROS_DISTRO

cd ../../xarm_ros
git pull
git submodule sync
git submodule update --init --remote

cd ../..
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

