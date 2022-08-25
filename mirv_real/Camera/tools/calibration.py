from dis import dis
import cv2
import numpy as np
import glob
from mirv_control.msg import camera_calibration as CameraCalibrationMsg
import rospy


class CameraCalibration:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    CALIBRATION_DIRETORY = "../auto_calibration_images"

    camera_matrix = []
    distortion_matrix = []

    hFOV = 63
    horizontalPixels = 640
    verticalPixels = 480
    degreesPerPixel = hFOV/horizontalPixels

    cameraCalibrationMsg = CameraCalibrationMsg()

    publisher = rospy.Publisher(
        'cameraCalibration', CameraCalibrationMsg, queue_size=1)

    def __init__(self):
        savedCoeffs = self.loadCoefficients()
        matrix = None
        distortion = None

        if savedCoeffs:
            matrix = savedCoeffs[0]
            distortion = savedCoeffs[1]
        else:
            ret, mtx, dist, rvecs, tvecs = self.calibrate()
            self.saveCoefficients(mtx, dist)
            matrix = mtx
            distortion = dist
        self.publishCameraCoefficients(matrix, distortion)

    def publishCameraCoefficients(self, matrix, distortion):
        self.cameraCalibrationMsg.matrix = matrix
        self.cameraCalibrationMsg.distortion = distortion
        self.cameraCalibrationMsg.hFOV = self.hFOV
        self.cameraCalibrationMsg.horizontalPixels = self.horizontalPixels
        self.cameraCalibrationMsg.verticalPixels = self.verticalPixels
        self.cameraCalibrationMsg.degreesPerPixel = self.degreesPerPixel

        self.publisher.publish(self.cameraCalibrationMsg)

    def saveCoefficients(self, mtx, dist):
        cv_file = cv2.FileStorage(
            f"{self.CALIBRATION_DIRETORY}/calibrationCoefficients.yaml", cv2.FILE_STORAGE_WRITE)
        cv_file.write("camera_matrix", mtx)
        cv_file.write("dist_coeff", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()

    def loadCoefficients(self):
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(
            f"{self.CALIBRATION_DIRETORY}/calibrationCoefficients.yaml", cv2.FILE_STORAGE_READ)

        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()

        # Debug: print the values
        # print("camera_matrix : ", camera_matrix.tolist())
        # print("dist_matrix : ", dist_matrix.tolist())

        cv_file.release()
        return [camera_matrix, dist_matrix]

    def calibrate(self):
        n = 7
        m = 7
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((m*n, 3), np.float32)
        objp[:, :2] = np.mgrid[0:n, 0:m].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        images = glob.glob(f'{self.CALIBRATION_DIRETORY}/*.jpg')

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (n, m), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (n, m), corners2, ret)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)

        return [ret, mtx, dist, rvecs, tvecs]


if __name__ == '__main__':
    try:
        calibration = CameraCalibration()
    except rospy.ROSInterruptException:
        pass
