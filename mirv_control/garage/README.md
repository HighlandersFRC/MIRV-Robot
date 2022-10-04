# Garage Docking

## detectGarage.py

ROS node which runs ArUco detection to search for garage in frame. Runs only when neuralNetworkSelector is set to "aruco"

## moveToGrage.py

actionlib server which drives rover into back of garage. Constantly drives towards ArUco tag 0 (back of garage) until limit switches are hit or it is close enough
