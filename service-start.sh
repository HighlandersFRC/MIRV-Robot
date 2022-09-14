#!/bin/bash
export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
source /opt/ros/melodic/setup.bash
source ~/mirv_ws/devel/setup.bash

export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh
. "$HOME/.cargo/env"
source /home/nvidia/mirv_ws/src/MIRV-Robot/setenv.sh
export OPENBLAS_CORETYPE=ARMV8
export ROS_LOG_DIR=/mnt/SSD/ROS

roslaunch mirv_real mirv.launch
