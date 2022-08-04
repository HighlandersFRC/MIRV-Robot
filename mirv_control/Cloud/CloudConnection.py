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
import numpy as np
import math
import json
from std_msgs.msg import String
from aiohttp import web
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from time import sleep
from threading import Thread
from signal import signal, SIGINT


CLOUD_HOST = os.getenv('API_HOST')
CLOUD_PORT = os.getenv('API_PORT')
ROVER_COMMON_NAME = os.getenv('MIRV_COMMON_NAME')
USERNAME = os.getenv('API_USERNAME')
PASSWORD = os.getenv('API_PASSWORD')

rospy.init_node("CloudConnection")

CLOUD_HOST = rospy.get_param('api_host', CLOUD_HOST)
CLOUD_PORT = rospy.get_param('api_port', CLOUD_PORT)
ROVER_COMMON_NAME = rospy.get_param('mirv_common_name', ROVER_COMMON_NAME)
USERNAME = rospy.get_param('api_username', USERNAME)
PASSWORD = rospy.get_param('api_password', PASSWORD)

if CLOUD_HOST is None:
    rospy.logerr("Please set the API_HOST Environment Variable to the IP of the cloud server")
    exit()

if CLOUD_PORT is None:
    rospy.logerr("Please set the API_PORT Environment Variable to the PORT of the cloud server")
    exit()

if ROVER_COMMON_NAME is None:
    rospy.logerr("Please set the ROVER_COMMON_NAME Environment Variable to the proper Rover ID in order to connect to cloud resources")
    exit()


DEFAULT_STATUS_MESSAGE = {
    "roverId": f"{ROVER_COMMON_NAME}",
    "state": "docked",
    "status": "available",
    "battery-percent": -1,
    "battery-voltage": -1,
    "health": {
        "electronics": "unavailable",
        "drivetrain": "unavailable",
        "intake": "unavailable",
        "sensors": "unavailable",
        "garage": "unavailable",
        "power": "unavailable",
        "general": "unavailable"
    },
    "telemetry": {
        "lat": 0,
        "long": 0,
        "heading": 0,
        "speed": 0
    }
}


token = ""

# Setup webtrc connection components
pcs = set()
channels = []

#Cache incoming frames for sending
frames = []
status_messages = []

def get_webrtc_state():
    if len(pcs) > 0:
        return pcs[0].connectionState
    else:
        return "closed"


# Tries to connect to Cloud API and form Socket Connection
def connect_to_api():
    try:
        #sio.connect(f"ws://{CLOUD_HOST}:{CLOUD_PORT}/ws", headers={"ID": ROVER_COMMON_NAME, "device_type": "rover"}, auth={"token": token}, socketio_path="/ws/socket.io")
        sio.connect(f"ws://{CLOUD_HOST}:{CLOUD_PORT}/ws", headers={"ID": ROVER_COMMON_NAME, "device_type": "rover", "token": token}, socketio_path="/ws/socket.io")
 
        rospy.loginfo("Cloud Connection Succeeded")
    except socketio.exceptions.ConnectionError as e:
        rospy.logwarn(f"Unable to Connect to API: {CLOUD_HOST}:{CLOUD_PORT}")
        rospy.logwarn(e)

# Sends a Message to the API
def send_to_api(message, type="data"):
    try:
        if api_connected:
            sio.emit(type, message)
        else:
            rospy.logwarn("API Disconnected. Cannot Send Message")
    except socketio.exceptions.BadNamespaceError:
        rospy.logwarn("Unable to send message to socket server")


# Send a Message on Webrtc Channel
def send_to_webrtc(message):
    try:
        str_msg = json.dumps(msg)
        for channel in channels:
            rospy.loginfo("Sending", + str_msg)
            channel.send(str_msg)
    except Exception as e:
        print("Unable to send message to webrtc")
    
    
# Receives ROS frames and adds them to internal buffer to be sent to the cloud
def frameSubscriber(data):
    #print("Got a new frame")
    frame = ros_numpy.numpify(data.color_frame)
    frames.append(frame)
    if len(frames) >5:
        frames.remove(frames[0])

# Receives data about robot status and passes it on to the Cloud
def statusSubscriber(data):
    if data is not None and len(str(data.data)) > 0:
        status = json.loads(str(data.data))
        rospy.loginfo(status)
        if api_connected:
            send_to_api(status)
        
        if get_webrtc_state == "connected":
            send_to_webrtc(status)

    else:
        print("Received Data with Type None", data)

def update_remote():
    while True:
        global socket_connection
        if socket_connection is None:
            print("Retrying Connection")
            socket_connection = connect_to_api()
        elif not socket_connection.connected:
            print("Connection to API has been lost")
        else:
            pass
        print("test")
        sleep(5)

    

# Setup Socket Connection with the cloud
sio = socketio.Client()
api_connected = False


# Activate ROS Nodes
rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame, frameSubscriber)
rospy.Subscriber("RoverStatus", String, statusSubscriber)
command_pub = rospy.Publisher('CloudCommands', String, queue_size=1)
availability_pub = rospy.Publisher('RoverAvailable', String, queue_size=1)


# Class Describing how to send VideoStreams to the Cloud
class RobotVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()  # don't forget this!
        self.counter = 0
        
    async def recv(self):
        
        pts, time_base = await self.next_timestamp()
        #print("test") 
        if len(frames) > 0:
            #print("sending real frame")
            cv_frame = frames[0]
            corrected_frame = cv2.cvtColor(cv_frame, cv2.COLOR_RGB2BGR)
            frame = VideoFrame.from_ndarray(corrected_frame)
            if len(frames) > 1:
                frames.remove(frames[0])
        else:
            #print("sending blank frame")
            blank_image = np.zeros((480,640,3), np.uint8)
            frame = VideoFrame.from_ndarray(blank_image)
        
        frame.pts = pts
        frame.time_base = time_base
        return frame

# Website posts an offer to python server
async def offer(request):
    '''
    if not get_webrtc_state() == 'closed':
        print("Testing")
        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"error": "Robot not available."}
            ),
        )
    '''
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
            print(message)
            command_pub.publish(message)
            channel.send("")
            #send_rtc("I Really Like Turtles!")


    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)
        state = get_webrtc_state()
        if state == "connected":
            availability_pub.publish("Unavailable")
        else:
            availability_pub.publish("Available")
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

def get_token():

    login_data = {'username': USERNAME, 'password': PASSWORD}
    print(login_data)
    try:
        response = requests.post(f"http://{CLOUD_HOST}:{CLOUD_PORT}/token", data=login_data,timeout=20)
        contents = json.loads(response.content.decode('utf-8'))
        token = contents.get('access_token')
        print(token)
        return token
    except requests.exceptions.ReadTimeout:
        print("Unable to Acquire Token")
        return ""
    except requests.exceptions.ConnectionError as e:
        print("Unable to Acquire Token", e)

    

# Periodically tries to connect to the cloud if the system is not connected
async def update_connection():
    while True:
        print("Update Connection")
        if token == "":
            get_token()
        if not api_connected: 
            connect_to_api()
        await asyncio.sleep(5)
'''
async def update_status(): 
    while True:        
        print("Update Status")
        if len(status_messages) > 0:
            status = status_messages[-1]
        else:
            status = DEFAULT_STATUS_MESSAGE
        
        if api_connected:
            send_to_api(status)
        
        if get_webrtc_state == "connected":
            send_to_webrtc(status)
        await asyncio.sleep(5)
'''

def stop():
    connection_task.cancel()
    status_task.cancel()
  

@sio.on('connect')
def connect():
    print('API connection established')
    global api_connected
    api_connected = True

@sio.event
def disconnect():
    print("API connection lost")
    global api_connected
    api_connected = False

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


def run_webserver():
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_post("/offer", offer)
    web.run_app(
        app, access_log=None, host="0.0.0.0", port=8080
    )




async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    sio.disconnect()
    stop()
    exit()


# Get Token from API
token = get_token()
connect_to_api()



loop = asyncio.get_event_loop()

#status_task = loop.create_task(update_status())
connection_task = loop.create_task(update_connection())


#loop.run_in_executor(None, status_task)
loop.run_in_executor(None, connection_task)

run_webserver()

