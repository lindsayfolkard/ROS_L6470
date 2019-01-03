#!/bin/bash

# A simple bash script to handle deployment of code to the robot

# Typical operations
# 1. Build , install and source new code on my machine
# 2. Build, install and deploy to a remote machine. Start the remote machine up
# 3. Start node(s) up on my other machine

# Arguments
# build,deploy,deploy_and_run,run

if [ $# -lt 1 ];then
    echo "Usage $0 [build &| deploy &| run | stop | edit | cleanbuild | remote-build | remote-clean]"
    exit 1
fi

if [ $1 == "-h" ];then
    echo "Usage $0 [build &| deploy &| run | stop | edit | cleanbuild | remote-build | remote-clean]"
    exit 0
fi

# Source if we need to
if ! [ -x "$(command -v ros2)" ]; then 
  source /opt/ros/bouncy/setup.bash
fi

# Ascii Colour codes
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

INFO=$YELLOW
SUCCESS=$GREEN

# Filepaths
sourceDir="/home/lindsay/git/ROS_STSPIN_ANGULAR/"

# Process the arguments
for arg in $@; do

    if [ $arg == "build" ]; then
        echo -e "${INFO} Building source ...${NC}"
        currDir=`pwd`
        cd $sourceDir
        colcon build --symlink-install
        cd $currDir 
        echo -e "${SUCCESS} Finished build! ${NC}"
    fi

    if [ $arg == "deploy" ]; then
    
        # Check we can talk to the robot

        # Rsync the files across
        echo -e "${INFO} Deploy source and binaries to remote host ... ${NC}"
        rsync --info=progress2 -rlt /home/lindsay/git foodrobot:
        #rsync --info=progress2 -rlt /home/lindsay/git/ROS_STSPIN_ANGULAR/src lindsay@FoodRobot:/home/lindsay/git/ROS_STSPIN_ANGULAR
        echo -e "${SUCCESS} Finished binary transfer to remote host ${NC}"
    fi

    if [ $arg == "run" ]; then
        
        echo -e "${INFO} Check for already running ROS2 nodes${NC}"
        
        if [ `ps aux | grep ros | wc -l` -gt 2 ]; then
            echo "ROS2 is already running"
            pkill -f 'ros2'
            pkill -f 'joyproxy'
        fi
        
        # Create the out directory
        outDir="/home/lindsay/out"
        mkdir -p $outDir
        sudo touch $outDir"/su.txt"
        echo -e "${INFO} Start ROS2 and required nodes on this machine ... ${NC}"
        
        # Create a new process to run ds4drv
        if [ `ps aux | grep ds4drv | wc -l` -gt 1 ]; then
            echo -e "${INFO}ds4drv is already running. Not starting again ${NC}"
        else
            echo -e "${INFO}Create PS4 bluetooth controller driver ${NC}"
            sudo ds4drv > $outDir"/ds4drv.out" &
            echo -e "${SUCCESS}Created PS4 bluetooth controller driver ${NC}"
        fi

        # Hack startup script
        ros2 run joy joy_node __params:=/home/lindsay/joy_params.yaml &> $outDir"/joy.out" &
        #source $sourceDir"install/setup.sh"
        #$sourceDir"install/joyproxy/bin/joyproxy_py" &> $outDir"/joyproxy.out" &

        echo -e "${SUCCESS} Started ROS2 nodes on this machine successfully ${NC}"

        # Run the launch script for this machine
        echo -e "${INFO} Start ROS2 and the required nodes up on the remote machine ... ${NC}"
        #ssh foodrobot -t "echo 'source ros' ; source /home/lindsay/ros2_bouncy/install/setup.sh ; source /home/lindsay/git/ROS_STSPIN_ANGULAR/install/setup.sh ; echo 'run joyproxy' ; $sourceDir'install/joyproxy/lib/joyproxy/joyproxy_py'" &> $outDir"/joyproxy.out" &
        #sleep 2
        #ssh foodrobot -t "echo 'source ros' && source /home/lindsay/ros2_bouncy/install/setup.sh && source /home/lindsay/git/ROS_STSPIN_ANGULAR/install/setup.sh && echo 'run ros_l6470' && /home/lindsay/git/ROS_STSPIN_ANGULAR/build/ros_l6470/l6470_main" &> $outDir"/foodrobot_ros_l6470.out" &
        echo -e "${SUCCESS} Started ROS2 nodes on remote machine successfully ${NC}"

        sleep 2
        echo -e "${INFO} Concatenating output of all the processes ${NC}"
        tail -f /home/lindsay/out/ds4drv.out /home/lindsay/out/foodrobot_ros_l6470.out /home/lindsay/out/joy.out /home/lindsay/out/joyproxy.out
    fi

    if [ $arg == "stop" ]; then
        echo -e "${INFO} Stop ROS2 and associated nodes on this machine ... ${NC}"
        pkill -f 'ros'
        sudo pkill -f 'ds4drv'
        pkill -f 'joyproxy'
        echo -e "${SUCCESS} Stopped ROS2 and associated nodes on this machine${NC}"

        echo -e "${INFO} Stop ROS2 and associated nodes on the remote machine ... ${NC}"
        #ssh lindsay@foodrobot -t 'pkill -f ros && pkill -f joyproxy'
        echo -e "${SUCCESS} Stopped ROS2 and associated nodes on remote machine"
    fi

    if [ $arg == "remote-clean" ]; then
        echo -e "${INFO} Clean remote build directory ...${NC}"
        ssh foodrobot -t "cd /home/lindsay/git/ROS_STSPIN_ANGULAR ; rm -rf build install"
        echo -e "${INFO} Remote build complete ${NC}"
    fi

    if [ $arg == "remote-build" ]; then
        echo -e "${INFO} Start clean remote build of ros2 packages ...${NC}"
        ssh foodrobot -t "echo 'source ros' && source /home/foodrobot/ros2_bouncy/install/setup.sh && echo 'finished sourcing' && cd $sourceDir && colcon build --symlink-install"
        echo -e "${INFO} Remote build complete ${NC}"
    fi

    if [ $arg == "edit" ]; then
        echo -e "${INFO} Open up editor and environment for ROS2 ... ${NC}"
        cmakeFile=$sourceDir"src/ROS_L6470/node/CMakeLists.txt"
        echo $cmakeFile 
        export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH
        qtcreator "$sourceDir/src/ROS_L6470/node/CMakeLists.txt" "$sourceDir/src/ROS_L6470/joyproxy_cpp/CMakeLists.txt" "$sourceDir/src/ROS_L6470/msg" "$sourceDir/src/ROS_L6470/srv"
        echo -e "${INFO}  ... ${NC}"
    fi

    if [ $arg == "clean-build" ]; then
       echo -e "${INFO} Cleanbuild src on remote machine ... ${NC}"
       rsync --info=progress2 -rlt /home/lindsay/git/ROS_STSPIN_ANGULAR/src foodrobot:/home/lindsay/git/ROS_STSPIN_ANGULAR
       ssh foodrobot -t "echo 'source ros' && source /home/lindsay/ros2_ws/install/setup.sh && echo 'finished sourcing' && cd $sourceDir && rm -rf build/ install/ log/ && colcon build --symlink-install"
       echo -e "${INFO}  Finished clean build ... ${NC}"
    fi

done
