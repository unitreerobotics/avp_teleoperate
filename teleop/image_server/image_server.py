import cv2
import zmq
import time
import struct
from collections import deque
import numpy as np
import pyrealsense2 as rs


class RealSenseCamera(object):
  def __init__(self, img_shape, fps, serial_number = None, enable_depth = False) -> None:
    self.img_shape = img_shape
    self.fps = fps
    self.serial_number = serial_number
    self.enable_depth = enable_depth

    align_to = rs.stream.color
    self.align = rs.align(align_to)
    self.init_realsense()

  def init_realsense(self):

    self.pipeline = rs.pipeline()
    config = rs.config()
    if self.serial_number is not None:
      config.enable_device(self.serial_number)

    config.enable_stream(rs.stream.color, self.img_shape[1] // 2, self.img_shape[0], rs.format.bgr8, self.fps)

    if self.enable_depth:
      config.enable_stream(rs.stream.depth, self.img_shape[1] // 2, self.img_shape[0], rs.format.z16, self.fps)

    profile = self.pipeline.start(config)
    self._device = profile.get_device()
    if self._device is None:
        print('pipe_profile.get_device() is None .')
    if self.enable_depth:
      assert self._device is not None
      depth_sensor = self._device.first_depth_sensor()
      self.g_depth_scale = depth_sensor.get_depth_scale() 

    self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()


  def get_frame(self):
    frames = self.pipeline.wait_for_frames()
    aligned_frames = self.align.process(frames)
    color_frame = aligned_frames.get_color_frame()

    if self.enable_depth:
      depth_frame = aligned_frames.get_depth_frame()

    if not color_frame:
      return None, None
    
    color_image = np.asanyarray(color_frame.get_data())
    # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    depth_image = np.asanyarray(depth_frame.get_data()) if self.enable_depth else None
    return color_image, depth_image


class ImageServer:
    def __init__(self, img_shape = (480, 640 * 2, 3), fps = 30, enable_wrist = False, port = 5555, Unit_Test = False):
        """
        img_shape: User's expected camera resolution shape (H, W, C). 
        
        p.s.1: 'W' of binocular camera according to binocular width (instead of monocular).

        p.s.2: User expectations are not necessarily the end result. The final img_shape value needs to be determined from \
               the terminal output (i.e. "Image Resolution: width is ···")
        
        fps: user's expected camera frames per second.
        
        port: The port number to bind to, where you can receive messages from subscribers.
        
        Unit_Test: When both server and client are True, it can be used to test the image transfer latency, \
                   network jitter, frame loss rate and other information.
        """
        self.img_shape = img_shape
        self.fps = fps
        self.enable_wrist = enable_wrist
        self.port = port
        self.enable_performance_eval = Unit_Test

        # initiate camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_shape[1])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_shape[0])
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if self.enable_wrist:
            # initiate realsense camera
            self.left_cam  = RealSenseCamera(img_shape = self.img_shape, fps = self.fps, serial_number = "218622271789")  # left wrist camera
            self.right_cam = RealSenseCamera(img_shape = self.img_shape, fps = self.fps, serial_number = "218622278527")  # right wrist camera

        # set ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")

        if self.enable_performance_eval:
            self._init_performance_metrics()

        print("Image server has started, waiting for client connections...")
        print(f"Image Resolution: width is {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, height is {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}\n")

    def _init_performance_metrics(self):
        self.frame_count = 0  # Total frames sent
        self.time_window = 1.0  # Time window for FPS calculation (in seconds)
        self.frame_times = deque()  # Timestamps of frames sent within the time window
        self.start_time = time.time()  # Start time of the streaming

    def _update_performance_metrics(self, current_time):
        # Add current time to frame times deque
        self.frame_times.append(current_time)
        # Remove timestamps outside the time window
        while self.frame_times and self.frame_times[0] < current_time - self.time_window:
            self.frame_times.popleft()
        # Increment frame count
        self.frame_count += 1

    def _print_performance_metrics(self, current_time):
        if self.frame_count % 30 == 0:
            elapsed_time = current_time - self.start_time
            real_time_fps = len(self.frame_times) / self.time_window
            print(f"[Image Server] Real-time FPS: {real_time_fps:.2f}, Total frames sent: {self.frame_count}, Elapsed time: {elapsed_time:.2f} sec")
        
    def _close(self):
        self.cap.release()
        self.socket.close()
        self.context.term()
        print("[Image Server] The server has been closed.")

    def send_process(self):
        try:
            while True:
                ret, head_color = self.cap.read()
                if not ret:
                    print("[Image Server] Frame read is error.")
                    break

                if self.enable_wrist:
                    left_wrist_color, left_wrist_depth   = self.left_cam.get_frame()
                    right_wrist_color, right_wrist_depth = self.right_cam.get_frame()
                    # Concatenate images along the width
                    head_color = cv2.hconcat([head_color, left_wrist_color, right_wrist_color])

                ret, buffer = cv2.imencode('.jpg', head_color)
                if not ret:
                    print("[Image Server] Frame imencode is failed.")
                    continue

                jpg_bytes = buffer.tobytes()

                if self.enable_performance_eval:
                    timestamp = time.time()
                    frame_id = self.frame_count
                    header = struct.pack('dI', timestamp, frame_id)  # 8-byte double, 4-byte unsigned int
                    message = header + jpg_bytes
                else:
                    message = jpg_bytes
                
                self.socket.send(message)

                if self.enable_performance_eval:
                    current_time = time.time()
                    self._update_performance_metrics(current_time)
                    self._print_performance_metrics(current_time)

        except KeyboardInterrupt:
            print("[Image Server] Interrupted by user.")
        finally:
            self._close()

if __name__ == "__main__":
    # server = ImageServer(img_shape = (720, 640 * 2, 3), Unit_Test = True)   # test
    server = ImageServer(img_shape = (640, 480 * 2, 3), enable_wrist = True, Unit_Test = False)  # deployment
    server.send_process()