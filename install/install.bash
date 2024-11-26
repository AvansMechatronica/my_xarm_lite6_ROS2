
sudo apt update
sudo apt -y install ros-$ROS_DISTRO-moveit
sudo apt -y install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt -y install ros-$ROS_DISTRO-combined-robot-hw
sudo apt -y install ros-$ROS_DISTRO-moveit-servo
sudo apt -y install ros-$ROS_DISTRO-moveit-visual-tools
sudo apt -y install ros-$ROS_DISTRO-ros-controllers
sudo apt -y install ros-$ROS_DISTRO-controller-manager
sudo apt -y install ros-$ROS_DISTRO-controller-manager-msgs


if ros2 pkg list | grep -q "xarm_description"; then
    echo "xarm packages alredy installed"
else
    echo "cloning xarm packages"
    git clone https://github.com/xArm-Developer/xarm_ros2.git ../../xarm_ros -b $ROS_DISTRO --recursive 
    #git clone https://github.com/ros-planning/moveit_task_constructor.git ../../moveit_task_constructor -b $ROS_DISTRO

    cd ../../xarm_ros
    git pull
    git submodule sync
    git submodule update --init --remote
    cp ../my_lite6_driver_config/xarm_user_params.yaml ../../xarm_ros/xarm_api/config/
fi

if ros2 pkg list | grep -q "pymoveit2"; then
    echo "pymoveit2 packages alredy installed"
else
    echo "cloning xarm pymoveit2"
    git clone https://github.com/AvansMechatronica/pymoveit2.git ../../pymoveit2 
fi

if ros2 pkg list | grep -q "my_moveit_python"; then
    echo "my_moveit_python packages alredy installed"
else
    echo "cloning my_moveit_python"
    git clone https://github.com/AvansMechatronica/my_moveit_python.git ../../my_moveit_python 
fi


if ros2 pkg list | grep -q "flexbe_behavior_engine"; then
    echo "flexbe_behavior_engine packages alredy installed"
else
    echo "cloning flexbe_behavior_engine"
    git clone https://github.com/FlexBE/flexbe_behavior_engine.git -b humble ../../flexbe_behavior_engine
fi

if ros2 pkg list | grep -q "flexbe_app"; then
    echo "flexbe_app packages alredy installed"
else
    echo "cloning flexbe_app"
    git clone https://github.com/FlexBE/flexbe_app.git -b humble ../../flexbe_app 
fi


if ros2 pkg list | grep -q "flexbe_manipulation_states"; then
    echo "generic_flexbe_states packages alredy installed"
else
    echo "cloning generic_flexbe_states"
    git clone https://github.com/FlexBE/generic_flexbe_states.git -b ros2-devel ../../generic_flexbe_states 
fi

#run after build
#ros2 run flexbe_app nwjs_install



cd ../..
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

