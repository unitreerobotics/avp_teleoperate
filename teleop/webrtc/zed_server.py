import asyncio
import json
import logging
import os
import platform
import ssl

import aiohttp_cors
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender
from multiprocessing import Process, Array, Value, shared_memory

ROOT = os.path.dirname(__file__)

relay = None
webcam = None


from aiortc import MediaStreamTrack
from av import VideoFrame
import numpy as np
import time 

class ZedVideoTrack(MediaStreamTrack):
    kind = "video"
    def __init__(self, queue, toggle_streaming, fps):
        super().__init__()  # Initialize base class
        # self.img_shape = (2*img_shape[0], img_shape[1], 3)
        # self.img_height, self.img_width = img_shape[:2]
        # self.shm_name = shm_name
        # existing_shm = shared_memory.SharedMemory(name=shm_name)
        # self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf)
        self.img_queue = queue
        self.toggle_streaming = toggle_streaming
        self.streaming_started = False
        self.timescale = 1000  # Use a timescale of 1000 for milliseconds
        # self.frame_interval = 1 / fps
        self._last_frame_time = time.time()
        self.start_time = time.time()
    
    async def recv(self):
        """
        This method is called when a new frame is needed.
        """
        # now = time.time()
        # wait_time = self._last_frame_time + self.frame_interval - now
        # if wait_time > 0:
        #     await asyncio.sleep(wait_time)
        # self._last_frame_time = time.time()
        # start = time.time()
        if not self.streaming_started:
            self.toggle_streaming.set()
            self.streaming_started = True
        frame = self.img_queue.get()
        # self.sem.release()
        # print("Time to get frame: ", time.time() - start, self.img_queue.qsize())
        # frame = self.img_array.copy()  # Assuming this is an async function to fetch a frame
        # frame = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
        # print("recv")
        # start = time.time()
        av_frame = VideoFrame.from_ndarray(frame, format='rgb24')  # Convert numpy array to AVFrame
        timestamp = int((time.time() - self.start_time) * self.timescale)
        av_frame.pts = timestamp
        av_frame.time_base = self.timescale
        # print("Time to process frame: ", time.time() - start)
        return av_frame
        

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )


async def index(request):
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)


async def javascript(request):
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

class RTC():
    def __init__(self, img_shape, img_queue, toggle_streaming, fps) -> None:
        self.img_shape = img_shape
        self.img_queue = img_queue
        self.fps = fps
        self.toggle_streaming = toggle_streaming

    async def offer(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("Connection state is %s" % pc.connectionState)
            if pc.connectionState == "failed":
                await pc.close()
                pcs.discard(pc)

        # open media source
        zed_track = ZedVideoTrack(self.img_queue, self.toggle_streaming, self.fps)
        video_sender = pc.addTrack(zed_track)
        # if Args.video_codec:
        force_codec(pc, video_sender, "video/H264")
        # elif Args.play_without_decoding:
            # raise Exception("You must specify the video codec using --video-codec")

        await pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )


pcs = set()

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


from params_proto import ParamsProto, Proto, Flag


class Args(ParamsProto):
    description = "WebRTC webcam demo"
    cert_file = Proto(help="SSL certificate file (for HTTPS)")
    key_file = Proto(help="SSL key file (for HTTPS)")

    host = Proto(default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)")
    port = Proto(default=8080, dtype=int, help="Port for HTTP server (default: 8080)")

    play_from = Proto(help="Read the media from a file and send it.")
    play_without_decoding = Flag(
        "Read the media without decoding it (experimental). "
        "For now it only works with an MPEGTS container with only H.264 video."
    )

    audio_codec = Proto(help="Force a specific audio codec (e.g. audio/opus)")
    video_codec = Proto(help="Force a specific video codec (e.g. video/H264)")
    img_shape = Proto(help="")
    shm_name = Proto(help="")
    fps = Proto(help="")

    verbose = Flag()


if __name__ == '__main__':

    if Args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if Args.cert_file:
        print("Using SSL certificate file: %s" % Args.cert_file)
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(Args.cert_file, Args.key_file)
    else:
        ssl_context = None

    app = web.Application()
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods="*",
        )
    })
    rtc = RTC((960, 640), queue)
    app.on_shutdown.append(on_shutdown)
    cors.add(app.router.add_get("/", index))
    cors.add(app.router.add_get("/client.js", javascript))
    cors.add(app.router.add_post("/offer", rtc.offer))

    web.run_app(app, host=Args.host, port=Args.port, ssl_context=ssl_context)