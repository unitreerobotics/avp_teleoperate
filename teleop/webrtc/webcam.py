import argparse
import asyncio
import json
import logging
import os
import ssl

import cv2
import numpy as np
import time
from pyzed import sl
from aiortc import MediaStreamTrack
import av

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.rtcrtpsender import RTCRtpSender

ROOT = os.path.dirname(__file__)


# relay = None
# webcam = None

left_mat = sl.Mat()
right_mat = sl.Mat()

class VideoTransformTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.runtime_parameters = sl.RuntimeParameters()
        self._direction = "sendonly"
        self.start_time = time.time()
        self.timescale = 1000  # Use a timescale of 1000 for milliseconds
        self.frame_index = 0

    async def recv(self):
        frame = await self.get_frame()
        if frame is not None:
            # Timestamp is calculated based on the frame index and the frame rate of the video
            timestamp = int((time.time() - self.start_time) * self.timescale)
            # Create an aiortc VideoFrame, setting timestamp and time_base accordingly
            video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
            video_frame.pts = timestamp
            video_frame.time_base = self.timescale
            return video_frame
        else:
            return None

    async def get_frame(self):
        if self.camera.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.camera.retrieve_image(left_mat, sl.VIEW.LEFT)
            self.camera.retrieve_image(right_mat, sl.VIEW.RIGHT)
            # Convert to BGR format as required by aiortc
            bgra = np.vstack((left_mat.numpy(), right_mat.numpy()))
            frame = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            return frame
        else:
            return None

    
def create_local_tracks(play_from, decode):
    camera = sl.Camera()
    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, camera_fps=30)
    if camera.open(init_params) == sl.ERROR_CODE.SUCCESS:
        print("ZED Camera successfully opened.")
        return None, VideoTransformTrack(camera)  # Assuming you only want video track
    else:
        print("Failed to open ZED camera")
        return None, None
    

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

    @pc.on('icecandidate')
    async def on_ice_candidate(event):
        if event.candidate:
            if event.candidate.type == 'host':
                # Handle or signal only the host candidate
                print(f"Host candidate: {event.candidate}")
            else:
                # Ignore non-host candidates
                pass

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # open media source
    audio, video = create_local_tracks(
        args.play_from, decode=not args.play_without_decoding
    )

    if audio:
        audio_sender = pc.addTrack(audio)
        if args.audio_codec:
            force_codec(pc, audio_sender, args.audio_codec)
        elif args.play_without_decoding:
            raise Exception("You must specify the audio codec using --audio-codec")

    if video:
        video_sender = pc.addTrack(video)
        if args.video_codec:
            force_codec(pc, video_sender, args.video_codec)
        elif args.play_without_decoding:
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC webcam demo")
    parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    parser.add_argument("--play-from", help="Read the media from a file and sent it.")
    parser.add_argument(
        "--play-without-decoding",
        help=(
            "Read the media without decoding it (experimental). "
            "For now it only works with an MPEGTS container with only H.264 video."
        ),
        action="store_true",
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Port for HTTP server (default: 8080)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    parser.add_argument(
        "--audio-codec", help="Force a specific audio codec (e.g. audio/opus)"
    )
    parser.add_argument(
        "--video-codec", help="Force a specific video codec (e.g. video/H264)"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)
    else:
        ssl_context = None

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)