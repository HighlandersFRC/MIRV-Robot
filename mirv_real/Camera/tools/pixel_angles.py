import math
import numpy as np
from scipy import stats

HFOV = 63

scale = 0.5

X = 640/2/scale
Y = 480/2/scale
aspect_ratio = Y/X

VFOV = HFOV * aspect_ratio

W = 1
H = 1 * aspect_ratio

D = H/math.tan(math.radians(VFOV/2))

CENTER_ANGLE = 0, -15

theta_h_corner = math.degrees(math.atan(W/math.sqrt(D**2 + H**2)))
theta_v_corner = math.degrees(math.atan(H/math.sqrt(D**2 + W**2)))


def rotateAxes2d(x, y, theta):
    x_r_x = x * math.sin(theta)
    x_r_y = x * math.cos(theta)
    y_r_x = y * math.sin(math.pi/2 + theta)
    y_r_y = y * math.cos(math.pi/2 + theta)
    x_r = x_r_x + y_r_x
    y_r = x_r_y + y_r_y
    return x_r, y_r


# Assumptions: Bottom left is 0,0
def get_angle(x, y):
    ratio_h = (x-X)/X
    ratio_v = (Y-y)/Y

    theta_h = (HFOV/2 + (theta_h_corner - HFOV/2)
               * abs(ratio_v))*abs(ratio_h)*get_sign(ratio_h)
    theta_v = (VFOV/2 + (theta_v_corner - VFOV/2)
               * abs(ratio_h))*abs(ratio_v)*get_sign(ratio_v)

    return theta_h, theta_v


def get_real_position(x, y, imu_offset, rover_position, center_angle, height):
    center_h, center_v = center_angle

    theta_h_r, theta_v_r = get_angle(x, y)
    theta_h = theta_h_r + center_h
    theta_v = theta_v_r + center_v

    if theta_v >= 0:
        print("ABOVE HORIZON")
        return None, None
    longitudinal_offset = height * math.tan(math.radians(90 + theta_v))
    lateral_offset = longitudinal_offset * math.tan(math.radians(theta_h))

    # print(longitudinal_offset, lateral_offset)

    x, y = rotateAxes2d(lateral_offset, longitudinal_offset, imu_offset)
    rover_pos_x, rover_pos_y = rover_position
    y = -y  # Reversing y axis

    # Now, this is in robot coordinates
    x += rover_pos_x
    y += rover_pos_y

    return round(x, 4), round(y, 4)


def estimate_coef(x, y):
    # number of observations/points
    n = np.size(x)

    # mean of x and y vector
    m_x = np.mean(x)
    m_y = np.mean(y)

    # calculating cross-deviation and deviation about x
    SS_xy = np.sum(y*x) - n*m_y*m_x
    SS_xx = np.sum(x*x) - n*m_x*m_x

    # calculating regression coefficients
    b_1 = SS_xy / SS_xx
    b_0 = m_y - b_1*m_x

    print(SS_xx, SS_xy)

    a = math.degrees(math.atan2(SS_xx, SS_xy))

    return (b_0, b_1, a)


# Robot coordinates
def get_angle_robot_frame(points_x, points_y):
    x0 = points_x[0]
    x1 = points_x[-1]
    y0 = points_y[0]
    y1 = points_y[-1]

    return math.degrees(math.atan2(y1 - y0, x1 - x0))


# Pixel coordinates
# Ensure line points upwards
def get_intercept_pixels(line):
    x0 = line[0][0]
    y0 = line[0][1]
    x1 = line[-1][0]
    y1 = line[-1][1]

    dy = y1 - y0
    dx = x1 - x0

    # Force line to point upwards (meaning y decreases)
    if dy > 0:
        line.reverse()
        x0 = line[0][0]
        y0 = line[0][1]
        x1 = line[-1][0]
        y1 = line[-1][1]
        dy = y1 - y0
        dx = x1 - x0
    if dx == 0:
        print("VERTICAL")
        # Line is vertical
        return x0, 0
    if dy == 0:
        print("HORIZONTAL")
        # Line is vertical
        return None, get_sign(dx)

    # Want to see what the x value is a y=480 (bottom of frame). Can be outside of image
    m = dy/dx
    dy_0 = Y*2 - y0
    dx_0 = dy_0 / m

    x_y480 = x0 + dx_0

    return x_y480, get_sign(dx)


def get_line_equations(line, imu_offset, rover_position, center_angle, height):
    points_x = []
    points_y = []

    x_intercept, dx_sign = get_intercept_pixels(line)

    for x, y in line:

        p = get_real_position(x, y, imu_offset,
                              rover_position, center_angle, height)
        if p[0] != None and p[1] != None:
            points_x.append(p[0])
            points_y.append(p[1])

    if points_x and points_y:

        angle = get_angle_robot_frame(points_x, points_y)

        return x_intercept, dx_sign, angle, points_x[0], points_y[0]
    else:
        return x_intercept, dx_sign, None, None, None


def get_sign(val):
    return 1 if val >= 0 else -1


# print(HFOV/2, VFOV/2)
# print(theta_h_corner, theta_v_corner)
# print(get_angle(0, 0))
# print(get_angle(640, 480))
# print(get_angle(320, 240))
# print(get_angle(160, 240))
# print(get_angle(320, 120))
# print(get_angle(160, 120))


# print(get_line_equations([[321, 480], [320, 240]], 0, (0, 0), (0, -15), 1))
# print(get_line_equations([[320, 240], [321, 480]], 0, (0, 0), (0, -15), 1))
# line = [[320, 240], [100, 100]]
# line = [[400, 100], [320, 240]]
# print(get_intercept_pixels(line))
# print(line)

# x = [0, 0]
# y = [0, -1]
# print(get_angle(x, y))
