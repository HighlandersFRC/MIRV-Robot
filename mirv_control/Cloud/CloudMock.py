import socketio
import json
import asyncio
import logging
import uuid
from aiohttp import web
import cv2
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
import requests
import numpy
import math

CLOUD_HOST = "44.202.152.178"
CLOUD_PORT = 8000



# Setup webtrc connection components
logger = logging.getLogger("pc")
pcs = set()
channels = []
# Setup OpenCV Capture Components

# Setup Socket Connection with the cloud
sio = socketio.Client()




# Class Describing how to send VideoStreams to the Cloud
class FlagVideoStreamTrack(VideoStreamTrack):
    """
    A video track that returns an animated flag.
    """

    def __init__(self):
        super().__init__()  # don't forget this!
        self.counter = 0
        height, width = 480, 640

        # generate flag
        data_bgr = numpy.hstack(
            [
                self._create_rectangle(
                    width=213, height=480, color=(255, 0, 0)
                ),  # blue
                self._create_rectangle(
                    width=214, height=480, color=(255, 255, 255)
                ),  # white
                self._create_rectangle(width=213, height=480, color=(0, 0, 255)),  # red
            ]
        )

        # shrink and center it
        M = numpy.float32([[0.5, 0, width / 4], [0, 0.5, height / 4]])
        data_bgr = cv2.warpAffine(data_bgr, M, (width, height))

        # compute animation
        omega = 2 * math.pi / height
        id_x = numpy.tile(numpy.array(range(width), dtype=numpy.float32), (height, 1))
        id_y = numpy.tile(
            numpy.array(range(height), dtype=numpy.float32), (width, 1)
        ).transpose()

        self.frames = []
        for k in range(30):
            phase = 2 * k * math.pi / 30
            map_x = id_x + 10 * numpy.cos(omega * id_x + phase)
            map_y = id_y + 10 * numpy.sin(omega * id_x + phase)
            self.frames.append(
                VideoFrame.from_ndarray(
                    cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR), format="bgr24"
                )
            )

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        frame = self.frames[self.counter % 30]
        frame.pts = pts
        frame.time_base = time_base
        self.counter += 1
        #for channel in channels:
        #    channel.send("I Like Turtles!")

        return frame

    def _create_rectangle(self, width, height, color):
        data_bgr = numpy.zeros((height, width, 3), numpy.uint8)
        data_bgr[:, :] = color
        return data_bgr

# Website posts an offer to python server
async def offer(request):
    params = await request.json()
    print("Received Params")
    print(params)
    offer = RTCSessionDescription(sdp=params["offer"]["sdp"], type=params["offer"]["type"])

    pc = RTCPeerConnection()
    channel = pc.createDataChannel("chat")
    channels.append(channel)

    pc.addTrack(FlagVideoStreamTrack())
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)
    

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            print("Received: ", message)
            channel.send("I Like Turtles")

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
