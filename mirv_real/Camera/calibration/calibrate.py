import numpy as np
import cv2 as cv
import glob
# termination criteria
INCHES_TO_METERS = 0.0254

n = 9
m = 7
side_length = 0.78 * INCHES_TO_METERS
# side_length = 1000

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((m*n, 3), np.float32)
objp[:, :2] = np.mgrid[0:n, 0:m].T.reshape(-1, 2)
objp *= side_length

print(objp)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('./calibration_images/*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (n, m), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (n, m), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
    else:
        print("BOARD NOT FOUND!!", fname)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# print("RET:", ret)
print("MTX:", mtx)
print("DIST:", dist)
# print("RVECS:", rvecs)
# print("TVECS:", tvecs)

cv.destroyAllWindows()
