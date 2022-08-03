import json
import math

FEET_TO_METERS = 0.3048
RADIANS_TO_DEGREES = 180 / math.pi
DEGREES_TO_RADIANS = math.pi / 180


# Taper: Taper consisting of 5 pucks spaced at 10 feet longitudinally, equally spaced in the lane or shoulder laterally
# | --------- | 10 feet                            | lane width
# >                                                |
#             >                                    |
#                         >                        |
#                                     >            |
#                                                 >|


# Spear: Spear consisting of 7 pucks spaced at 10 feet longitudinally, equally spaced in the lane or shoulder laterally
# | --------- | 10 feet
# >                                                | lane width
#             >                                    |
#                         >                        |
#                                     >            |
#                         >                        |
#             >                                    |
# >                                                |


# >
# |   \
# |       \
# ------------>
# width: lane width / (num Pi-lits - 1)
# height: 10 feet
# andle from longitudinal direction: +math.atan2(10ft, lane_width/(num_pi_lits - 1)) (negative if on right side of spear)


def getEndPoint(lat1, lon1, bearing, d):
    R = 6371.0*1000  # Radius of the Earth in meters
    brng = math.radians(bearing)  # convert degrees to radians
    dist = d  # convert distance in meters
    lat1 = math.radians(lat1)  # Current lat point converted to radians
    lon1 = math.radians(lon1)  # Current long point converted to radians
    lat2 = math.asin(math.sin(lat1)*math.cos(d/R) +
                     math.cos(lat1)*math.sin(d/R)*math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R) *
                             math.cos(lat1), math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    return lat2, lon2


def get_taper_pi_lit_locations(lat, long, heading, lane_width):
    # 5 pi-lits
    NUMBER_PI_LITS = 5
    LONGITUDINAL_DISTANCE_METERS = 10 * FEET_TO_METERS
    LATERAL_DISTANCE_METERS = lane_width / (NUMBER_PI_LITS - 1)

    start_lat = lat
    start_long = long

    pi_lit_angle_relative = math.atan2(
        LONGITUDINAL_DISTANCE_METERS, LATERAL_DISTANCE_METERS) * RADIANS_TO_DEGREES
    pi_lit_angle = heading + pi_lit_angle_relative
    pi_lit_distance = math.sqrt(
        LONGITUDINAL_DISTANCE_METERS**2 + LATERAL_DISTANCE_METERS**2)

    pi_lit_locations = [(start_lat, start_long)]
    for i in range(1, NUMBER_PI_LITS):
        location = getEndPoint(start_lat, start_long,
                               pi_lit_angle, pi_lit_distance * i)
        pi_lit_locations.append(location)

    return pi_lit_locations


def get_spear_pi_lit_locations(lat, long, heading, lane_width):
    # 7 pi-lits
    NUMBER_PI_LITS = 7
    NUMBER_PI_LEFT = 3
    NUMBER_PI_RIGHT = 3
    LONGITUDINAL_DISTANCE_METERS = 10 * FEET_TO_METERS
    LATERAL_DISTANCE_METERS = lane_width / (NUMBER_PI_LITS - 1)

    # Start location: Left hand side, at start of formation
    start_lat = lat
    start_long = long

    pi_lit_angle_relative = math.atan2(
        LONGITUDINAL_DISTANCE_METERS, LATERAL_DISTANCE_METERS) * RADIANS_TO_DEGREES
    pi_lit_angle = heading + pi_lit_angle_relative
    pi_lit_distance = math.sqrt(
        LONGITUDINAL_DISTANCE_METERS**2 + LATERAL_DISTANCE_METERS**2)

    pi_lit_locations = [(start_lat, start_long)]
    for i in range(1, NUMBER_PI_LITS):
        location = getEndPoint(start_lat, start_long,
                               pi_lit_angle, pi_lit_distance * i)
        pi_lit_locations.append(location)

    return pi_lit_locations
