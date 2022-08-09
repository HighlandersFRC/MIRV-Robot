import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)
objp *= 0.019812 # size of calibration chessboard in meters
print("OBJP: ", objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('src/cameraFrame.jpg')
for fname in images:
    print("loop")
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,9), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("true")
        objpoints.append(objp)
        # corners2 = cv.cornerSubPix(gray,corners, (1,1), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,9), corners, ret)
        cv.imwrite(r'src/checkerboard.jpg', img)
        cv.waitKey(500)
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        print("RET: ", ret)
        print("MTX: ", mtx)
        print("DIST: ", dist)
        print("RVECS: ", rvecs)
        print("TVECS: ", tvecs)
        
cv.destroyAllWindows()