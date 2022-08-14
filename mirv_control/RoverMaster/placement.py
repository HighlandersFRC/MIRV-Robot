import json
import math

FEET_TO_METERS = 0.3048
RADIANS_TO_DEGREES = 180 / math.pi
DEGREES_TO_RADIANS = math.pi / 180

LANE_TYPES = ["general", "right-shoulder",
              "left-shoulder", "right-lane", "left-lane"]


# 3 Taper: Taper consisting of 3 pucks spaced at 10, 100, and 200 feet longitudinally, at the outside, center, and center of the lane, respectively
# Source: https://schneiderjobs.com/blog/how-to-place-emergency-triangles, Divided highways and one-way roads
# | 200 feet | ---------              100 feet | ---------          10 feet | --------- | lane width
#                                                                           >           |
#                                                                                       |
#            >                                 >                                        |
#                                                                                       |
#                                                                                       |


# 5 Taper: Taper consisting of 5 pucks spaced at 10 feet longitudinally, equally spaced in the lane or shoulder laterally
# | --------- | 10 feet                            | lane width
#                                                 >|
#                                     >            |
#                         >                        |
#             >                                    |
# >                                                |


# 7 Spear: Spear consisting of 7 pucks spaced at 10 feet longitudinally, equally spaced in the lane or shoulder laterally
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


# taper_right_3, taper_left_3, taper_right_5, taper_left_5, spear_7

def generate_pi_lit_formation(lat_long, heading, lane_width, formation_type):
    if formation_type == "taper_right_3":
        return taper_3(lat_long, heading, lane_width)
    elif formation_type == "taper_left_3":
        return taper_3(lat_long, heading, lane_width, left_side=True)
    elif formation_type == "taper_right_5":
        return taper_5(lat_long, heading, lane_width)
    elif formation_type == "taper_left_5":
        return taper_5(lat_long, heading, lane_width, left_side=True)
    elif formation_type == "spear_7":
        return spear_7(lat_long, heading, lane_width)


def taper_3(lat_long, heading, lane_width, left_side=False):

    lat, long = lat_long

    # Start location: Left hand side, at start of formation
    start_lat = lat
    start_long = long

    # Positive for to the right of start, negative for to the left of start
    sign = -1 if left_side else -1

    # longitudinal_dist: longitudal distance from start point to pi-lit, in feet
    # lateral_dist: lateral (sideways) distance from the edge of the lane, in lane widths
    PI_LIT_LOCATIONS = [
        {'longitudinal_dist': 10, 'lateral_dist': 0.128},
        {'longitudinal_dist': 100, 'lateral_dist': 0.5},
        {'longitudinal_dist': 200, 'lateral_dist': 0.5},
    ]

    pi_lit_locations = []
    for loc in PI_LIT_LOCATIONS:
        # Convert units
        longitudinal_dist_meters = loc['longitudinal_dist'] * FEET_TO_METERS
        lateral_dist_meters = loc['lateral_dist'] * \
            lane_width * FEET_TO_METERS * sign

        # Generate angle and hypotenuse
        pi_lit_angle_relative = math.atan2(lateral_dist_meters,
                                           longitudinal_dist_meters) * RADIANS_TO_DEGREES
        pi_lit_angle = heading + pi_lit_angle_relative
        pi_lit_distance = math.sqrt(
            longitudinal_dist_meters**2 + lateral_dist_meters**2)

        # Get end of vector
        location = getEndPoint(start_lat, start_long,
                               pi_lit_angle, pi_lit_distance)

        # Save value to array
        loc_rev = [location[0], location[1]]
        pi_lit_locations.append(loc_rev)

    return pi_lit_locations


def taper_5(lat_long, heading, lane_width, left_side=False):
    NUMBER_PI_LITS = 5
    LONGITUDINAL_DISTANCE_METERS = 10 * FEET_TO_METERS
    LATERAL_DISTANCE_METERS = lane_width / (NUMBER_PI_LITS - 1)

    lat, long = lat_long

    # Start location: Location of first Pi-Lit
    start_lat = lat
    start_long = long

    sign = -1 if left_side else -1

    pi_lit_angle_relative = sign * math.atan2(LATERAL_DISTANCE_METERS,
                                              LONGITUDINAL_DISTANCE_METERS) * RADIANS_TO_DEGREES
    pi_lit_angle = heading + pi_lit_angle_relative
    pi_lit_distance = math.sqrt(
        LONGITUDINAL_DISTANCE_METERS**2 + LATERAL_DISTANCE_METERS**2)

    pi_lit_locations = [[start_lat, start_long]]
    for i in range(1, NUMBER_PI_LITS):
        location = getEndPoint(start_lat, start_long,
                               pi_lit_angle, pi_lit_distance * i)
        location = [location[0], location[1]]
        pi_lit_locations.append(location)

    return pi_lit_locations


def spear_7(lat_long, heading, lane_width):
    NUMBER_PI_LITS = 7
    NUMBER_PI_LIT_LEFT = int((NUMBER_PI_LITS - 1) / 2 + 1)
    NUMBER_PI_LIT_RIGHT = int((NUMBER_PI_LITS - 1) / 2 + 1)
    LONGITUDINAL_DISTANCE_METERS = 10 * FEET_TO_METERS
    LATERAL_DISTANCE_METERS = lane_width / (NUMBER_PI_LITS - 1)

    lat, long = lat_long

    # Start location: Left hand side, at start of formation
    start_lat = lat
    start_long = long

    pi_lit_angle_relative = math.atan2(LATERAL_DISTANCE_METERS,
                                       LONGITUDINAL_DISTANCE_METERS) * RADIANS_TO_DEGREES
    pi_lit_angle_start = heading + pi_lit_angle_relative  # Start from left-hand side
    pi_lit_angle_end = heading - pi_lit_angle_relative - 180
    pi_lit_distance = math.sqrt(
        LONGITUDINAL_DISTANCE_METERS**2 + LATERAL_DISTANCE_METERS**2)

    pi_lit_locations = [[start_lat, start_long]]
    for i in range(1, NUMBER_PI_LIT_LEFT):
        location = getEndPoint(start_lat, start_long,
                               pi_lit_angle_start, pi_lit_distance * i)
        location = [location[0], location[1]]
        pi_lit_locations.append(location)

    point_lat = pi_lit_locations[-1][0]
    point_long = pi_lit_locations[-1][1]
    for i in range(1, NUMBER_PI_LIT_RIGHT):
        location = getEndPoint(point_lat, point_long,
                               pi_lit_angle_end, pi_lit_distance * i)
        location = [location[0], location[1]]
        pi_lit_locations.append(location)

    return pi_lit_locations


def test():
    lat = 40.47411757814349
    long = -104.9694349989295
    heading = 45
    lane_width = 3
    lane_type = "right-lane"
    print(generate_pi_lit_formation((lat, long), heading, lane_width, lane_type))


if __name__ == "__main__":
    test()
