# Actionlib message definitions

Contains message definitions (goal, results, feedback) for all actionlib servers

## Database.action

Query database for positions of place Pi-Lits

### Goal

None

### Result

| Field     | Type                            | Description                         |
| --------- | ------------------------------- | ----------------------------------- |
| latitude  | float64                         | List of Pi-Lit locations, latitude  |
| longitude | float64                         | List of Pi-Lit locations, longitude |
| altitude  | List Pi-Lit locations, altitude |

Publishers:

- mirv_real/Database/DatabaseController.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## DetectLanes.action

Run lane detection on the most recent camera frame

### Goal

| Field      | Type    | Description                                   |
| ---------- | ------- | --------------------------------------------- |
| position_x | float64 | Current rover X position in truck coordinates |
| position_y | float64 | Current rover Y position in truck coordinates |
| heading    | float64 | Current rover angle in truck coordinates      |

### Result

| Field           | Type                    | Description                                                 |
| --------------- | ----------------------- | ----------------------------------------------------------- |
| net_heading     | float64                 | Net detected lane heading, in truck coordinates, in degrees |
| width           | float64                 | Lane width, in meters (currently hardcoded to 3m)           |
| lane_detections | single_lane_detection[] | List of detected lane line positions/angles                 |

Publishers:

- mirv_real/Camera/tools/detectLaneLines.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## DriveDistance.action

Drive straight forward/backward a specified distance

### Goal

| Field                | Type    | Description                            |
| -------------------- | ------- | -------------------------------------- |
| targetDistanceMeters | float64 | Distance to drive, in meters           |
| velocityMPS          | float64 | Target velocity, im m/s                |
| successThreshold     | float64 | Accuracy of ending position, in meters |

### Result

| Field         | Type    | Description                         |
| ------------- | ------- | ----------------------------------- |
| finished      | bool    | Did action successfully complete    |
| distanceError | float64 | Ending error in position, in meters |

Publishers:

- mirv_control/generalCommands/driveDistance.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## Garage.action

Drive into garage

### Goal

| Field                | Type    | Description                         |
| -------------------- | ------- | ----------------------------------- |
| estimatedGarageAngle | float64 | Current angle to garage, in degrees |

### Result

| Field    | Type | Description                      |
| -------- | ---- | -------------------------------- |
| finished | bool | Did action successfully complete |

Publishers:

- mirv_control/garage/moveToGarage.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## general.action

Teleoperated (manual) driving

### Goal

| Field | Type   | Description                  |
| ----- | ------ | ---------------------------- |
| goal  | string | JSON command from client app |

### Result

| Field  | Type   | Description |
| ------ | ------ | ----------- |
| result | string | not used    |

Publishers:

- mirv_control/TeleOp/TeleopDrive.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## general.action

Teleoperated (manual) driving

### Goal

| Field | Type   | Description                  |
| ----- | ------ | ---------------------------- |
| goal  | string | JSON command from client app |

### Result

| Field  | Type   | Description |
| ------ | ------ | ----------- |
| result | string | not used    |

Publishers:

- mirv_control/TeleOp/TeleopDrive.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## IMUCalibration.action

Calibration sequence

### Goal

| Field     | Type | Description |
| --------- | ---- | ----------- |
| calibrate | bool | not used    |

### Result

| Field     | Type | Description                      |
| --------- | ---- | -------------------------------- |
| succeeded | bool | Did action successfully complete |

Publishers:

- mirv_control/coordinateConversion/HeadingFilter.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## NavSatToTruck.action

Convert lat/long/elev to truck coordinates

### Goal

| Field     | Type    | Description             |
| --------- | ------- | ----------------------- |
| latitude  | float64 | GPS latitude            |
| longitude | float64 | GPS longitude           |
| altitude  | float64 | GPS altitude (not used) |

### Result

| Field            | Type    | Description                                 |
| ---------------- | ------- | ------------------------------------------- |
| truckCoordX      | float64 | X position, in truck coordinates, in meters |
| truckCoordY      | float64 | X position, in truck coordinates, in meters |
| distanceToOrigin | float64 | Distance from origin, in meters             |

Publishers:

- mirv_control/coordinateConversion/globalToTruck.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## PickupPilit.action

Drive to Pi-Lit location (uses intakeSide)

### Goal

| Field      | Type   | Description                                                         |
| ---------- | ------ | ------------------------------------------------------------------- |
| intakeSide | string | side of intake to grab Pi-Lit with, ["switch_left", "switch_right"] |

### Result

| Field    | Type | Description                      |
| -------- | ---- | -------------------------------- |
| finished | bool | Did action successfully complete |

Publishers:

- mirv_control/piLitRetrieval/pickupPILit.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## PointTurn.action

Turn in place by a specific angle

### Goal

| Field            | Type    | Description                            |
| ---------------- | ------- | -------------------------------------- |
| targetAngle      | float64 | Relative angle to turn, in degrees     |
| successThreshold | float64 | Accuracy of ending position, in meters |

### Result

| Field      | Type    | Description                       |
| ---------- | ------- | --------------------------------- |
| finished   | bool    | Did action successfully complete  |
| angleError | float64 | Ending error in angle, in degrees |

Publishers:

- mirv_control/generalCommands/pointTurnRelative.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py

## PurePursuit.action

Turn in place by a specific angle

### Goal

| Field           | Type      | Description                                     |
| --------------- | --------- | ----------------------------------------------- |
| TargetPoints    | float64[] | List of points to move to, in truck coordinates |
| NumTargetPoints | int32     | Number of requested points                      |

### Result

| Field            | Type    | Description                                                    |
| ---------------- | ------- | -------------------------------------------------------------- |
| AtTargetPoint    | bool    | Did rover end within the distance threshold of the final point |
| errorToPoint     | float64 | Ending error to final target point, in meters                  |
| finalTargetPoint | float64 | Location of final target point, in meters                      |
| angleToTarget    | float64 | Estimated angle to final target point, in degrees              |

Publishers:

- mirv_control/PurePursuit/purePursuit.py

Subscribers:

- mirv_control/RoverMaster/RoverInterface.py
