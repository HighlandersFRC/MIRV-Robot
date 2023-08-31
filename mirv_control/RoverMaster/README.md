# Rover Master

Contains high level rover operations and commands

## constants.py

Helpful constants related to helpful_functions_lib.py

## helpful_functions_lib.py

Helpful functions for common rotations and transformations

## PiLitController.py

ROS node which controls PiLit illumination

## placement.py

Generates PiLit placement locations from lane detections

## RoverController.py

ROS node, Runs Interface and Macros

## RoverInterface.py

Controls rover state transition and processes received commands. Also holds low level commands and pub/sub objects (drive to a location, pickup 1 pi-lit, ...)

## RoverMacros.py

Holds high level orchestrated commands (dock, deploy a pi-lit sequence, ...)

## RoverState.py

HELP ME JOHN
