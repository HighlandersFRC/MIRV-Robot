# Multi-Incident Response Vehicle

The Multi-Incident Response Vehicle (MIRV) Rover is an autonomous rover capable of deploying from the tire bay of an F-150 Pick-up truck and placing Pi-Lit flares along the roadside. This rover is developed by the students of the Highlanders FRC team in conjunction with Neaera Consulting for the Virginia Tech Transportation Institute (VTTI).

## Installation

To setup the MIRV codebase on your system and prepare a local development environment, please see the installation instructions [here](installation.md).

To setup an AGX for a new MIRV Rover, please see the installation instructions [here](agx_setup.md).

## Usage
Mirv will automatically start the Mirv service when the system boots. This is convienient for users who just want to use the MIRV Rover. Once the MIRV codebase is launched, it will automatically connect to the MIRV-Cloud API and can then be connected to via the MIRV App or similar. The Mirv application can be managed via systemmd as follows:

### Stop the running MIRV service
```
sudo systemctl stop mirv
```

### Start Running the MIRV program as a service
```
sudo systemctl start mirv
```

### Restart the MIRV service
```
sudo systemctl restart mirv
```

### Start running MIRV from the command line
```
roslaunch mirv_real mirv.launch
```

### Shutdown the agx
sudo poweroff


## Folder Structure

recommended folder structure:

- mirv (metapackage)
  - mirv_control (real world/simulation-agnostic nodes)
  - mirv_real (real-world sensor drivers/calibration)
  - utilities (Various scripts useful for debugging and configuring MIRV)

  The Below packages have been removed from the MIRV Project, and moved to a new repository to support easier management. 
  - mirv_description (simulation meshes/sensor plugins/robot description, just robot specifics. If someone else wanted to use the robot for a different task, they would want these files)
  - mirv_simulation (simulation launch files/worlds)






