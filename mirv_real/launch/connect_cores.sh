#!/bin/bash
export ROS_MASTER_URI="http://$(hostname -I | cut -f2 -d' '):11311"
export ROS_IP=$(hostname -I | cut -f2 -d' ')
echo $ROS_IP
echo $ROS_MASTER_URI