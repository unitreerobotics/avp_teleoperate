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

    def __init__(self, img_shape, shm_name, fps):
        super().__init__()  # Initialize base class
        self.img_shape = (2*img_shape[0], img_shape[1], 3)
        self.img_height, self.img_width = img_shape[:2]
        existing_shm = shared_memory.SharedMemory(name=shm_name)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf)

        self.frame_interval = 1 / fps
        self._last_frame_time = time.time()
    
    async def recv(self):
        """
        This method is called when a new frame is needed.
        """
        now = time.time()
        wait_time = self._last_frame_time + self.frame_interval - now
        if wait_time > 0:
            await asyncio.sleep(wait_time)
        self._last_frame_time = time.time()

        frame = self.img_array  # Assuming this is an async function to fetch a frame
        av_frame = VideoFrame.from_ndarray(frame, format='bgr24')  # Convert numpy array to AVFrame
        av_frame.pts = None
        av_frame.time_base = None
        return av_frame
        
def create_local_tracks(play_from, decode):
    global relay, webcam
    
    if play_from:
        player = MediaPlayer(play_from, decode=decode)
        return player.audio, player.video
    else:
        options = {"framerate": "30", "video_size": "640x480"}
        if relay is None:
            if platform.system() == "Darwin":
                webcam = MediaPlayer(
                    "default:none", format="avfoundation", options=options
                )
            elif platform.system() == "Windows":
                webcam = MediaPlayer(
                    "video=Integrated Camera", format="dshow", options=options
                )
            else:
                webcam = MediaPlayer("/dev/video0", format=None, options=options)
            relay = MediaRelay()
        return None, relay.subscribe(webcam.video)


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


async def offer(request):
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
    audio, video = create_local_tracks(
        Args.play_from, decode=not Args.play_without_decoding
    )

    if audio:
        audio_sender = pc.addTrack(audio)
        if Args.audio_codec:
            force_codec(pc, audio_sender, Args.audio_codec)
        elif Args.play_without_decoding:
            raise Exception("You must specify the audio codec using --audio-codec")

    if video:
        video_sender = pc.addTrack(video)
        if Args.video_codec:
            force_codec(pc, video_sender, Args.video_codec)
        elif Args.play_without_decoding:
            raise Exception("You must specify the video codec using --video-codec")

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

    app.on_shutdown.append(on_shutdown)
    cors.add(app.router.add_get("/", index))
    cors.add(app.router.add_get("/client.js", javascript))
    cors.add(app.router.add_post("/offer", offer))

    web.run_app(app, host=Args.host, port=Args.port, ssl_context=ssl_context)