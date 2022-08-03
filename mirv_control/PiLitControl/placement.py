import json
import math

FEET_TO_METERS = 0.3048
RADIANS_TO_DEGREES = 180 / math.pi
DEGREES_TO_RADIANS = math.pi / 180

LANE_TYPES = ["general", "right-shoulder",
              "left-shoulder", "right-lane", "left-lane"]


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


def generate_pi_lit_formation(lat_long, heading, lane_width, lane_type):
    if lane_type == "general":
        return get_spear_pi_lit_locations(lat_long, heading, lane_width)
    elif lane_type == "right-shoulder" or lane_type == "right-lane":
        return get_taper_pi_lit_locations(lat_long, heading, lane_width)
    elif lane_type == "left-shoulder" or lane_type == "left-lane":
        return get_taper_pi_lit_locations(lat_long, heading, lane_width, left_side=True)


def get_taper_pi_lit_locations(lat_long, heading, lane_width, left_side=False):
    NUMBER_PI_LITS = 5
    LONGITUDINAL_DISTANCE_METERS = 10 * FEET_TO_METERS
    LATERAL_DISTANCE_METERS = lane_width / (NUMBER_PI_LITS - 1)

    lat, long = lat_long

    # Start location: Left hand side, at start of formation
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
        loc_rev = [location[0], location[1]]
        pi_lit_locations.append(loc_rev)

    return pi_lit_locations


def get_spear_pi_lit_locations(lat_long, heading, lane_width):
    NUMBER_PI_LITS = 7
    NUMBER_PI_LIT_LEFT = int((NUMBER_PI_LITS - 1) / 2 + 1)
    NUMBER_PI_LIT_RIGHT = int((NUMBER_PI_LITS - 1) / 2)
    LONGITUDINAL_DISTANCE_METERS = 10 * FEET_TO_METERS
    LATERAL_DISTANCE_METERS = lane_width / (NUMBER_PI_LITS - 1)

    lat, long = lat_long

    # Start location: Left hand side, at start of formation
    start_lat = lat
    start_long = long

    pi_lit_angle_relative = math.atan2(
        LONGITUDINAL_DISTANCE_METERS, LATERAL_DISTANCE_METERS) * RADIANS_TO_DEGREES
    pi_lit_angle_start = heading + pi_lit_angle_relative  # Start from left-hand side
    pi_lit_angle_end = 180 + heading - pi_lit_angle_relative
    pi_lit_distance = math.sqrt(
        LONGITUDINAL_DISTANCE_METERS**2 + LATERAL_DISTANCE_METERS**2)

    pi_lit_locations = [(start_lat, start_long)]
    for i in range(1, NUMBER_PI_LIT_LEFT):
        location = getEndPoint(start_lat, start_long,
                               pi_lit_angle_start, pi_lit_distance * i)
        loc_rev = [location[0], location[1]]
        pi_lit_locations.append(loc_rev)
    point_lat = pi_lit_locations[-1][0]
    point_long = pi_lit_locations[-1][1]
    for i in range(1, NUMBER_PI_LIT_RIGHT):
        location = getEndPoint(point_lat, point_long,
                               pi_lit_angle_start, pi_lit_distance * i)
        loc_rev = [location[0], location[1]]
        pi_lit_locations.append(loc_rev)

    return pi_lit_locations

    #   [
    #     -104.9694299697876,
    #     40.47414792840873
    #   ],
    #   [
    #     -104.9693588912487,
    #     40.474197917051015
    #   ]


def test():
    lat = 40.47410559106016
    long = -104.96941991150379
    heading = 45
    lane_width = 2
    lane_type = "general"
    print(generate_pi_lit_formation((lat, long), heading, lane_width, lane_type))


if __name__ == "__main__":
    test()
