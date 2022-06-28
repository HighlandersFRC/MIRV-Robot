import socketio
import json

import argparse
import asyncio
import logging
import os
import ssl
import uuid
import numpy as np
import math
from aiohttp import web
import cv2
from av import VideoFrame
import aiohttp_cors
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder, MediaRelay
from aiortc.contrib.signaling import BYE, add_signaling_arguments, create_signaling
import requests


CLOUD_HOST = "127.0.0.1"
CLOUD_PORT = 8000



# Setup webtrc connection components
logger = logging.getLogger("pc")
pcs = set()
relay = MediaRelay()

# Setup OpenCV Capture Components
# TODO replace with stream read from camera topic
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


# Setup Socket Connection with the cloud
sio = socketio.Client()




# Class Describing how to send VideoStreams to the Cloud
class FlagVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()  # don't forget this!
        
    async def recv(self):
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = VideoFrame.from_ndarray(frame)
        pts, time_base = await self.next_timestamp()
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
    pc.addTrack(FlagVideoStreamTrack())
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)
    

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            print("Received: ", message)

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


def on_shutdown():
    # close peer connections
    coros = [pc.close() for pc in pcs]
    asyncio.gather(*coros)
    pcs.clear()





# Send sample Rover Status to Cloud
print(f"http://{CLOUD_HOST}:{CLOUD_PORT}/ws")
sio.connect(f"http://{CLOUD_HOST}:{CLOUD_PORT}/ws", headers={"roverId": "rover_6"},
            auth={"password": None}, socketio_path="/ws/socket.io")
send("data", {
    "roverId": "rover_6",
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
