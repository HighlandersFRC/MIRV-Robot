# Multi-Incident Response Vehicle

The Multi-Incident Response Vehicle (MIRV) Rover is an autonomous rover capable of deploying from the tire bay of an F-150 Pick-up truck and placing Pi-Lit flares along the roadside. This rover is developed by the students of the Highlanders FRC team in conjunction with Neaera Consulting for the Virginia Tech Transportation Institute (VTTI).


## Installation
To setup the MIRV codebase on your system, please see the installation instructions [here](installation.md).

To setup an AGX for a new MIRV Rover, please see the installation instructions [here](agx_setup.md).

## Usage
Currently the only launch file is road_slight_curve.launch in mirv_simulation, which launches the vehicle with sensors on a road in gazebo. 

## Folder Structure
recommended folder structure:
- mirv (metapackage)
  - mirv_control (real world/simulation-agnostic nodes)
  - mirv_real (real-world sensor drivers/calibration)
  - mirv_description (simulation meshes/sensor plugins/robot description,  just robot specifics. If someone else wanted to use the robot for a different task, they would want these files)
  - mirv_simulation (simulation launch files/worlds)

optional other packages:
  - mirv_msgs (any custom messages that we define)
  - mirv_tutorials (tutorials for using the rover/simulator)
  - mirv_documentation
