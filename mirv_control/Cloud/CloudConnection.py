#!/usr/bin/env python3

import socketio
import json
import asyncio
import logging
import uuid
import cv2
import requests
import os
import sys
import rospy
import ros_numpy
import numpy
import math
from std_msgs.msg import String
from aiohttp import web
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR+'/../')
#from custom_messages.msg import depth_and_color_msg as depthAndColorFrame
from mirv_description.msg import depth_and_color_msg as depthAndColorFrame


CLOUD_HOST = os.getenv('API_HOST')
CLOUD_PORT = os.getenv('API_PORT')
ROVER_COMMON_NAME = os.getenv('MIRV_COMMON_NAME')


if CLOUD_HOST is None:
    print("Please set the API_HOST Environment Variable to the IP of the cloud server")
    exit()

if CLOUD_PORT is None:
    print("Please set the API_PORT Environment Variable to the PORT of the cloud server")
    exit()

if ROVER_COMMON_NAME is None:
    print("Please set the ROVER_COMMON_NAME Environment Variable to the proper Rover ID in order to connect to cloud resources")
    exit()

# Setup webtrc connection components
pcs = set()
channels = []

#Cache incoming frames for sending
frames = []


def send_rtc(msg):
    for channel in channels:
        print("Sending: " + msg)
        channel.send(msg)

def frameSubscriber(data):
    frame = ros_numpy.numpify(data.color_frame)
    frames.append(frame)
    if len(frames) >5:
        frames.remove(frames[0])

def statusSubscriber(data):
    send_rtc(str(data))


# Setup Socket Connection with the cloud
sio = socketio.Client()

rospy.init_node("CloudConnection")
rospy.Subscriber("CameraFrames", depthAndColorFrame, frameSubscriber)
rospy.Subscriber("RobotStatus", String, statusSubscriber)
command_pub = rospy.Publisher('CloudCommands', String, queue_size=1)



# Class Describing how to send VideoStreams to the Cloud
class RobotVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()  # don't forget this!
        self.counter = 0
        
    async def recv(self):
        pts, time_base = await self.next_timestamp()
        if len(frames) > 0:
            frame = VideoFrame.from_ndarray(frames[0])
            frames.remove(frames[0])
        else:
            blank_image = np.zeros((480,640,3), np.uint8)
            frame = VideoFrame.from_ndarray(blank_image)
        
        frame.pts = pts
        frame.time_base = time_base
        return frame

# Website posts an offer to python server
async def offer(request):
    params = await request.json()
    print("Received Params")
    print(params)
    offer = RTCSessionDescription(sdp=params["offer"]["sdp"], type=params["offer"]["type"])

    pc = RTCPeerConnection()
    channel = pc.createDataChannel("RoverStatus");
    channels.append(channel)
    pc.addTrack(RobotVideoStreamTrack())
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)
    

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            print("Received: ", message)
            command_pub.publish(message)
            #channel.send("I Like Turtles")
            #send_rtc("I Really Like Turtles!")


    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        #log_info("Connection state is %s", pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)


    # handle offer
    await pc.setRemoteDescription(offer)

    # send answer
    answer = await pc.createAnswer()
    print(answer)
    await pc.setLocalDescription(answer)
    print("Returning Answer")
    print(json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}))


    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"answer": json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})}
        ),
    )

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    sio.disconnect()

  

@sio.on('connect')
def connect():
    print('connection established')


@sio.on('message')
def message(data):
    print('message received with ', data)
    # sio.emit('my response', {'response': 'my response'})


@sio.on('exception')
def exception(data):
    print('exception received with ', data)
    # sio.emit('my response', {'response': 'my response'})
    

@sio.on('connection_offer')
def connection_offer(data):
    # I was able to get the WebRTC offer / answer exchange working through asyncio. However, despite valid signals being returned to the client. It was unable to form a valid connection
    # There appears to be a bug in the aiortc implementation that prevents this from working without either the aiortc signaling server, or an http server.
    # Long story short. A local http server is being used to relay the signal. 
    x = requests.post('http://localhost:8080/offer', json = data)
    return x.json()



@sio.on('disconnect')
def disconnect():
    print('disconnected from server')


def send(type: str, msg):
    sio.emit(type, msg)

    






# Send sample Rover Status to Cloud
print(f"http://{CLOUD_HOST}:{CLOUD_PORT}/ws")
sio.connect(f"ws://{CLOUD_HOST}:{CLOUD_PORT}/ws", headers={"roverId": "rover_6"},
            auth={"password": None}, socketio_path="/ws/socket.io")
send("data", {
    "roverId": f"{ROVER_COMMON_NAME}",
    "state": "docked",
    "status": "available",
    "battery-percent": 12,
    "battery-voltage": 18,
    "health": {
        "electronics": "healthy",
        "drivetrain": "healthy",
        "intake": "healthy",
        "sensors": "healthy",
        "garage": "healthy",
        "power": "healthy",
        "general": "healthy"
    },
    "telemetry": {
        "lat": 39,
        "long": -105,
        "heading": 90,
        "speed": 0
    }
})

app = web.Application()
app.on_shutdown.append(on_shutdown)
app.router.add_post("/offer", offer)

web.run_app(
    app, access_log=None, host="0.0.0.0", port=8080
)
#send("disconnect","disconnect")


