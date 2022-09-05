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
import time
from std_msgs.msg import String, Bool
from aiohttp import web
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from mirv_control.msg import depth_and_color_msg as depthAndColorFrame
from mirv_control.msg import garage_state_msg as garage_state
from time import sleep
from threading import Thread
from signal import signal, SIGINT


CLOUD_HOST = os.getenv('API_HOST')
CLOUD_PORT = os.getenv('API_PORT')
ROVER_COMMON_NAME = os.getenv('MIRV_COMMON_NAME')
USERNAME = os.getenv('API_USERNAME')
PASSWORD = os.getenv('API_PASSWORD')
GARAGE_ID = os.getenv('GARAGE_ID')

rospy.init_node("CloudConnection")

CLOUD_HOST = rospy.get_param('api_host', CLOUD_HOST)
CLOUD_PORT = rospy.get_param('api_port', CLOUD_PORT)
ROVER_COMMON_NAME = rospy.get_param('mirv_common_name', ROVER_COMMON_NAME)
USERNAME = rospy.get_param('api_username', USERNAME)
PASSWORD = rospy.get_param('api_password', PASSWORD)
GARAGE_ID = rospy.get_param('garage_id', GARAGE_ID)


input_vars = [CLOUD_HOST,ROVER_COMMON_NAME,USERNAME,PASSWORD,GARAGE_ID]
env_vars = ['API_HOST', 'MIRV_COMMON_NAME', 'API_PASSWORD', 'GARAGE_ID']
ros_params = ['api_host', 'mirv_common_name', 'api_username','api_password', 'garage_id']

for var, env_var_name, ros_param_name in zip(input_vars, env_vars, ros_params):
    if var is None:
        rospy.logerr("{env_var_name} variable is not set. Please set the f{env_var_name} environment variable or the {ros_param_name} ros param.")
        exit()

if CLOUD_PORT is None:
    endpoint = f"{CLOUD_HOST}"
else:
    endpoint = f"{CLOUD_HOST}:{CLOUD_PORT}"    


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
pcs = []
channels = []

#Cache incoming frames for sending
frames = []
status_messages = []

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


def get_token():
    login_data = {'username': USERNAME, 'password': PASSWORD}
    print(login_data)
    try:
        response = requests.post(f"https://{endpoint}/token", data=login_data,timeout=20)
        contents = json.loads(response.content.decode('utf-8'))
        token = contents.get('access_token')
        print(token)
        return token
    except requests.exceptions.ReadTimeout:
        print("Unable to Acquire Token")
        return ""
    except requests.exceptions.ConnectionError as e:
        print("Unable to Acquire Token", e)
        return ""

def get_webrtc_state():
    if len(pcs) > 0:
        return pcs[0].connectionState
    else:
        return "closed"


# Tries to connect to API and form Socket Connection
def connect_to_api():
    try:
        sio.reconnection = True
        sio.reconnection_delay = 0.5
        sio.reconnection_delay_max = 1
        sio.reconnection_attempts = 0
        sio.handle_sigint = True
        sio.connect(f"ws://{endpoint}/ws", headers={"ID": ROVER_COMMON_NAME, "device-type": "rover", "token": token}, socketio_path="/ws/socket.io", wait_timeout = 30)
        rospy.loginfo("Cloud Connection Succeeded")
    except socketio.exceptions.ConnectionError as e:
        global api_connected
        api_connected = False
        rospy.logwarn(f"Unable to Connect to API: {endpoint}")
        rospy.logwarn(e)

# Sends a Message to the API
def send_to_api(message, type="data"):
    try:
        if sio.connected:
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
        global token
        if token == "":
            token = get_token()
        
        if sio.connected:
            send_to_api(status)
            get_garage_state(token)
        #else:
        #    connect_to_api()       
        if get_webrtc_state == "connected":
            send_to_webrtc(status)
        #if len(status_messages) == 0:
        status_messages.append(data.data)
    else:
        print("Received Data with Type None", data)

def garageSubscriber(data):
    if data is not None and len(str(data.data)) > 0:
        cmd = str(data.data)
        print(cmd)
        headers= {'Authorization': f'Bearer {token}'}
        body = {"command": cmd}
        print("Sending Command")
        response = requests.post(f"https://{endpoint}/garages/{GARAGE_ID}/command",json = body, headers=headers)
        print(response.status_code, response.text)


    

# def update_remote():
#     while True:
#         global socket_connection
#         if socket_connection is None:
#             print("Retrying Connection")
#             socket_connection = connect_to_api()
#         elif not socket_connection.connected:
#             print("Connection to API has been lost")
#         else:
#             pass
#         print("test")
#         sleep(5)

    

# Setup Socket Connection with the cloud
sio = socketio.Client()
api_connected = False


# Activate ROS Nodes
rospy.Subscriber("IntakeCameraFrames", depthAndColorFrame, frameSubscriber, queue_size = 1)
rospy.Subscriber("RoverStatus", String, statusSubscriber, queue_size =1)
rospy.Subscriber("GarageCommands", String, garageSubscriber, queue_size = 1)
command_pub = rospy.Publisher('CloudCommands', String, queue_size=1)
garage_pub = rospy.Publisher('GarageStatus', garage_state, queue_size=1)


# Class Describing how to send VideoStreams to the Cloud
class RobotVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()  # don't forget this!
        self.counter = 0
    
    
    # async def recv(self):
    #     pts, time_base = await self.next_timestamp()
    #     #print("test") 
    #     if len(frames) > 0:
    #         #print("sending real frame")
    #         cv_frame = frames[0]
    #         corrected_frame = cv2.cvtColor(cv_frame, cv2.COLOR_RGB2BGR)
    #         frame = VideoFrame.from_ndarray(corrected_frame)
    #         if len(frames) > 1:
    #             frames.remove(frames[0])
    #     else:
    #         #print("sending blank frame")
    #         blank_image = np.zeros((480,640,3), np.uint8)
    #         frame = VideoFrame.from_ndarray(blank_image)
        
    #     frame.pts = pts
    #     frame.time_base = time_base
    #     return frame
    
    async def recv(self):
        startTime = time.time()
        pts, time_base = await self.next_timestamp()
        ret, cv_frame = cap.read()
        corrected = cv2.cvtColor(cv_frame, cv2.COLOR_RGB2BGR)
        frame = VideoFrame.from_ndarray(corrected)
        frame.pts = pts
        frame.time_base = time_base
        
        #print("Returned Frame", time.time() - startTime, cv_frame.shape)
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
    pcs.append(pc)
    

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            global status_messages
            print(message)
            command_pub.publish(message)
            for send_message in status_messages:
                print("Sending Message", send_message)
                channel.send(send_message)
            status_messages = []


    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            pcs.remove(pc)
        state = get_webrtc_state()
        print("Connection Status Changed")
        print(state)
        
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

def get_garage_state(tkn):
    try:
        headers= {'Authorization': f'Bearer {tkn}'}
        response = requests.get(f"https://{endpoint}/garages/{GARAGE_ID}",headers=headers)

        if response.status_code ==200:
            contents = json.loads(response.content.decode('utf-8'))
            state = garage_state()

            state.garage_id = contents.get("garage_id","unavailable")
            state.linked_rover_id = contents.get("linked_rover_id", "unavailable")
            state.state = contents.get("state","unavailable")
            state.lights_on = contents.get("lights_on", False)
            state.health = contents.get("health","unavailable")
            state.health_details = contents.get("health_description","unavailable")
            garage_pub.publish(state)
        elif response.status_code ==401:
            global token
            token = get_token()
        else:
            print(f"Received Status Code: {response.status_code}, Details: {response.text}")
    except Exception as e:
        print("Garage Update failed with error",e )
    

# Periodically tries to connect to the cloud if the system is not connected
async def update_connection():
    while True:
        print("Update Connection")
        if token == "":
            get_token()
        if not api_connected: 
            connect_to_api()
        await asyncio.sleep(5)


async def update_garage(token):
    print("Started Garage")
    while True:
        get_garage_state(token)
        print("test")
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
  

@sio.on('connect')
def connect():
    print('API connection established')
    global api_connected
    api_connected = True

@sio.event
def disconnect():
    rospy.logwarn("API connection lost")
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

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    for pc in pcs:
        pcs.remove(pc)
    await asyncio.gather(*coros)
    #pcs.clear()
    sio.disconnect()
    stop()
    exit()

def run_webserver():
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_post("/offer", offer)
    web.run_app(
        app, access_log=None, host="0.0.0.0", port=8080
    )


# Get Token from API
token = get_token()
connect_to_api()



loop = asyncio.get_event_loop()

garage_task = loop.create_task(update_garage(token))
connection_task = loop.create_task(update_connection())

loop.run_in_executor(None, garage_task)
loop.run_in_executor(None, connection_task)

run_webserver()

