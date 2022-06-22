import cv2

img = cv2.imread("PXL_20220610_215236808.jpg")

cv2.rectangle(img, (555, 1439), (717, 1705), (0,255,0), 3)
cv2.rectangle(img, (959, 792), (1107, 967), (0,255,0), 3)
cv2.rectangle(img, (492, 2034), (747, 2289), (0,255,0), 3)
cv2.rectangle(img, (471, 9), (625, 179), (0,255,0), 3)


# cv2.putText(img, 'PILIT: 0.997', (344, 277-10), cv2.FONT_HERSHEY_COMPLEX, 
                #    2, (0,255, 0), 2, cv2.LINE_AA)

# cv2.circle(img, (344, 277), 15, (0,0,255), -1)
# cv2.circle(img, (1199, 1170), 15, (0,0,255), -1)

img = cv2.resize(img, (640, 480))

while True:
    cv2.imshow("img", img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break