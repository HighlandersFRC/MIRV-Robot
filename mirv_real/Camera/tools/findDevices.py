import cv2
import depthai as dai

pipeline = dai.Pipeline()

camRgb = pipeline.createColorCamera()

xoutRgb = pipeline.createXLinkOut()
xoutRgb.setStreamName("rgb")
camRgb.preview.link(xoutRgb.input)

found, device_info = dai.Device.getDeviceByMxId("10.0.10.3")

with dai.Device(pipeline, device_info) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    while True:
        cv2.imshow("rgb", qRgb.get().getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break

# import depthai
# for device in depthai.Device.getAllAvailableDevices():
#     print(f"{device.getMxId()} {device.state}")