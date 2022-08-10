import cv2
import glob
import numpy as np
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import dse_lib


hFOV = 63  # 63
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels

# cameraMatrix = np.array([[345.8860714,    0,         302.95331804],
#                          [0,         341.3646748,  283.84150363],
#                          [0,           0,           1]])
cameraMatrix = np.array([[403,    0,         313],
                         [0,         416,  226],
                         [0,           0,           1]])

# DEPTH_SCALING_FACTOR = 1.27/0.829

distCoeffs = np.array(
    [[.2,  -.9, -.18, .03,  .5]])
# distCoeffs = np.array(
#     [[0, 0, 0, 0, 0]])


images = glob.glob('./detection_images/*.jpg')


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters_create()


def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
        gray, cv2.aruco_dict, parameters=parameters)

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.2, matrix_coefficients,
                                                                          distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # print(rvec, tvec)
            pose = PoseStamped()
            # pose.header.stamp = rospy.Time.now()
            # pose.header.frame_id = self.ros_prefix[1:] + \
            #     '/camera_rgb_frame'

            pose.pose.position.x = tvecs[0][0][2]
            pose.pose.position.y = -tvecs[0][0][0]
            pose.pose.position.z = -tvecs[0][0][1]

            # x forward, y left, z up

            # Swap the angles around to correctly represent our coordinate system
            # Aruco puts zero at the tag, with z facing out...
            # We want x forward, y left, z up, euler order zyx = ypr
            rvecs_reordered = [rvecs[0][0][2],
                               rvecs[0][0][0], rvecs[0][0][1]]
            r = R.from_rotvec(rvecs_reordered)
            est_ypr = r.as_euler('zyx')
            quat = dse_lib.eul2quat(est_ypr[:, None])
            # r = R.from_euler('zyx', est_ypr + [np.pi, 0, np.pi])
            # quat = r.as_quat
            # print(quat[:])
            # print(pose.pose.orientation)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            poseZero = PoseStamped()
            poseZero.pose.position.x = 0
            poseZero.pose.position.y = 0
            poseZero.pose.position.z = 0
            poseZero.pose.orientation.x = 0
            poseZero.pose.orientation.y = 0
            poseZero.pose.orientation.z = 0
            poseZero.pose.orientation.w = 1

            output_frame = get_frame_transform(poseZero, pose)
            print(output_frame)

            # Draw Axis
            # cv2.drawFrameAxes(frame, matrix_coefficients,
            #                   distortion_coefficients, rvec, tvec, 0.01)
            # cv2.waitKey(500)

    return frame


def get_frame_transform(from_frame, to_frame):
    position = [0, 0, 0]
    position[0] = from_frame.position.x
    position[1] = from_frame.position.y
    position[2] = from_frame.position.z
    quat = [0, 0, 0, 0]
    quat[0] = from_frame.orientation.x
    quat[1] = from_frame.orientation.y
    quat[2] = from_frame.orientation.z
    quat[3] = from_frame.orientation.w
    t1 = position
    r1 = R.from_quat(quat)
    R1 = r1.as_dcm()

    position = [0, 0, 0]
    position[0] = to_frame.position.x
    position[1] = to_frame.position.y
    position[2] = to_frame.position.z
    quat = [0, 0, 0, 0]
    quat[0] = to_frame.orientation.x
    quat[1] = to_frame.orientation.y
    quat[2] = to_frame.orientation.z
    quat[3] = to_frame.orientation.w
    t2 = position
    r2 = R.from_quat(quat)
    R2 = r2.as_dcm()

    tz = (np.transpose(R1).dot(t2) - np.transpose(R1).dot(t1))
    Rz = np.transpose(R1).dot(R2)
    rz = R.from_dcm(Rz)
    rz = rz.as_quat()

    result = Pose()
    result.position.x = tz[0]
    result.position.y = tz[1]
    result.position.z = tz[2]
    result.orientation.x = rz[0]
    result.orientation.y = rz[1]
    result.orientation.z = rz[2]
    result.orientation.w = rz[3]
    return result


def detectArUcoMarkers(image):
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image, arucoDict, parameters=arucoParams)

    distance = 0
    angle = 0

    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2, 1))
            ret = cv2.aruco.estimatePoseSingleMarkers(
                markerCorner, 0.09906, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)

            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            angle = (cX - horizontalPixels/2) * degreesPerPixel
            # depth = (depthFrame[cY][cX])/1000

            # tvec[2] = tvec[2] * DEPTH_SCALING_FACTOR

            print("ANGLE: ", rvec, " DEPTH: ", tvec)

            # if(depth != 0):
            #     location = [depth, angle]

            #     locationMsg = Float64MultiArray()
            #     locationMsg.data = location

            #     garageLocationPub.publish(locationMsg)


for fname in images:
    img = cv2.imread(fname)
    # detectArUcoMarkers(img)
    pose_esitmation(img, cv2.aruco.DICT_6X6_50, cameraMatrix, distCoeffs)
