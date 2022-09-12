# import numpy as np
# import cv2 as cv
# import glob
# # termination criteria
# criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objp = np.zeros((7*9,3), np.float32)
# objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# objp *= 0.019812 # size of calibration chessboard in meters
# print("OBJP: ", objp)
# # Arrays to store object points and image points from all the images.
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.
# images = glob.glob('src/cameraFrame.jpg')
# for fname in images:
#     print("loop")
#     img = cv.imread(fname)
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv.findChessboardCorners(gray, (9,7), None)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         print("true")
#         objpoints.append(objp)
#         # corners2 = cv.cornerSubPix(gray,corners, (1,1), (-1,-1), criteria)
#         imgpoints.append(corners)
#         # Draw and display the corners
#         cv.drawChessboardCorners(img, (9,7), corners, ret)
#         cv.imwrite(r'src/checkerboard.jpg', img)
#         cv.waitKey(500)
#         ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#         print("RET: ", ret)
#         print("MTX: ", mtx)
#         print("DIST: ", dist)
#         print("RVECS: ", rvecs)
#         print("TVECS: ", tvecs)
        
# cv.destroyAllWindows()

'''
Performs calibration using data pickled by calibrationDataCollector.py
Allows for RPi camera to be calibrated on a more capable computer.
'''

import time
import cv2.aruco as A
import cv2
import numpy as np
import pickle

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,8,.025,.0125,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

#Start capturing images for calibration
cap = cv2.VideoCapture(0)

allCorners = []
allIds = []
decimator = 0
in_between_images = 5
for i in range(400 * in_between_images):

    ret, frame = cap.read()
    if i % in_between_images == 0:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = cv2.aruco.detectMarkers(gray, dictionary)

        if len(res[0]) > 0:
            res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 4 == 0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

            cv2.aruco.drawDetectedMarkers(gray, res[0], res[1])

    cv2.imshow('frame', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator += 1

imsize = gray.shape

cap.release()
cv2.destroyAllWindows()

print(allIds)

print("calibrating now")
startTime = time.time()
#print(startTime)


try:
    print("something else")
    cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
    print("something")
except:
    print("failure")
    raise
else:
    print("triumph") # huge success, hard to overstate my satisfaction
    deltaTime = time.time() - startTime
    print("calibration took " + str(deltaTime) + " seconds")
    pickle.dump(cal, open( "calibrationSave.p", "wb" ))
    #retval, cameraMatrix, distCoeffs, rvecs, tvecs = cal
'''
cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
print("triumph") # huge success, hard to overstate my satisfaction
pickle.dump(cal, open( "calibrationSave.p", "wb" ))
'''