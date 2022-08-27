import json
import math

FEET_TO_METERS = 0.3048
RADIANS_TO_DEGREES = 180 / math.pi
DEGREES_TO_RADIANS = math.pi / 180
# 3 Taper: Taper consisting of 3 pucks spaced at 10, 100, and 200 feet longitudinally, at the outside, center, and center of the lane, respectively
# Source: https://schneiderjobs.com/blog/how-to-place-emergency-triangles, Divided highways and one-way roads
# | 200 feet | ---------              100 feet | ---------          10 feet | --------- | lane width
#                                                                           >           |
#                                                                                       |
#            >                                 >                                        |
#                                                                                       |
#                                                                                       |

# Traffic Direction: >>>>

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


###
# detected_lanes: {'left': (lat, long), 'right': (lat, long)}
# heading: global heading to place pi-lits in, relative to detected_lanes
# lane_width: Width of lane, in meters
# formation_type: enum of pi-lit formation type (taper_right_3, taper_left_3, taper_right_5, taper_left_5, spear_7)
def generate_pi_lit_formation(detected_lanes, heading, lane_width, formation_type):
    if formation_type == "taper_right_3":
        return taper_3(detected_lanes, heading, lane_width)
    elif formation_type == "taper_left_3":
        return taper_3(detected_lanes, heading, lane_width, left_side=True)
    elif formation_type == "taper_right_5":
        return taper_5(detected_lanes, heading, lane_width)
    elif formation_type == "taper_left_5":
        return taper_5(detected_lanes, heading, lane_width, left_side=True)
    elif formation_type == "spear_7":
        return spear_7(detected_lanes, heading, lane_width)


def taper_3(detected_lanes, heading, lane_width, left_side=False):
    start_lat_long = get_center_coordinates(
        detected_lanes, heading, lane_width)
    if start_lat_long == (None, None):
        return []

    # Positive for to the right of start, negative for to the left of start
    sign = -1 if left_side else 1

    # longitudinal_dist: longitudal distance from start point to pi-lit, in feet
    # lateral_dist: lateral (sideways) distance from the edge of the lane, in lane widths
    PI_LIT_LOCATIONS = [
        {'longitudinal_dist': 0, 'lateral_dist': 0.375 * sign},
        {'longitudinal_dist': 100, 'lateral_dist': 0.0 * sign},
        {'longitudinal_dist': 200, 'lateral_dist': 0.0 * sign},
    ]

    return generate_pi_lit_locations(start_lat_long, heading, lane_width, PI_LIT_LOCATIONS)


def taper_5(detected_lanes, heading, lane_width, left_side=False):
    start_lat_long = get_center_coordinates(
        detected_lanes, heading, lane_width)
    if start_lat_long == (None, None):
        return []

    # Positive for to the right of start, negative for to the left of start
    sign = -1 if left_side else 1

    # longitudinal_dist: longitudal distance from start point to pi-lit, in feet
    # lateral_dist: lateral (sideways) distance from the edge of the lane, in lane widths
    PI_LIT_LOCATIONS = [
        {'longitudinal_dist': 0,  'lateral_dist': 0.5 * sign},
        {'longitudinal_dist': 10, 'lateral_dist': 0.25 * sign},
        {'longitudinal_dist': 20, 'lateral_dist': 0.0 * sign},
        {'longitudinal_dist': 30, 'lateral_dist': -0.25 * sign},
        {'longitudinal_dist': 40, 'lateral_dist': -0.5 * sign},
    ]

    return generate_pi_lit_locations(start_lat_long, heading, lane_width, PI_LIT_LOCATIONS)


def spear_7(detected_lanes, heading, lane_width, left_side=False):
    start_lat_long = get_center_coordinates(
        detected_lanes, heading, lane_width)
    if start_lat_long == (None, None):
        return []

    # Positive for to the right of start, negative for to the left of start
    sign = -1 if left_side else 1

    # longitudinal_dist: longitudal distance from start point to pi-lit, in feet
    # lateral_dist: lateral (sideways) distance from the edge of the lane, in lane widths
    PI_LIT_LOCATIONS = [
        {'longitudinal_dist': 0,  'lateral_dist': 0.5 * sign},
        {'longitudinal_dist': 10, 'lateral_dist': 0.333 * sign},
        {'longitudinal_dist': 20, 'lateral_dist': 0.167 * sign},
        {'longitudinal_dist': 30, 'lateral_dist': 0 * sign},
        {'longitudinal_dist': 20, 'lateral_dist': -0.167 * sign},
        {'longitudinal_dist': 10, 'lateral_dist': -0.333 * sign},
        {'longitudinal_dist': 0,  'lateral_dist': -0.5 * sign},
    ]

    return generate_pi_lit_locations(start_lat_long, heading, lane_width, PI_LIT_LOCATIONS)


def get_center_coordinates(detected_lanes, heading, lane_width):
    if len(detected_lanes.values()) == 2:
        return ((detected_lanes['left'][0] + detected_lanes['right']
                [0])/2, (detected_lanes['left'][1] + detected_lanes['right'][1])/2)
    elif detected_lanes.get('left'):
        return getEndPoint(
            detected_lanes['left'][0], detected_lanes['left'][1], heading + 90, lane_width/2)
    elif detected_lanes.get('right'):
        return getEndPoint(
            detected_lanes['right'][0], detected_lanes['right'][1], heading - 90, lane_width/2)
    else:
        return (None, None)


def generate_pi_lit_locations(start_lat_long, heading, lane_width, distances):
    start_lat, start_long = start_lat_long
    pi_lit_locations = []
    for loc in distances:
        # Convert units
        longitudinal_dist_meters = loc['longitudinal_dist'] * FEET_TO_METERS
        lateral_dist_meters = loc['lateral_dist'] * lane_width

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


def test():
    lat_long_right = (40.47413211566721, -104.96944840997458)
    lat_long_left = (40.474113038173684, -104.96942333108585)
    detected_lanes = {'right': lat_long_right, 'left': lat_long_left}
    heading = 225  # Opposite traffic direction
    lane_width = 3  # meters
    lane_type = "spear_7"
    print(generate_pi_lit_formation(detected_lanes, heading, lane_width, lane_type))


if __name__ == "__main__":
    test()