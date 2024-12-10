#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.

import csv
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.video.renderer import PdrawRenderer


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")


class StreamingExample:
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        self.h264_frame_stats = []
        self.frame_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.renderer = None

    def start(self):
        # Connect to drone
        assert self.drone.connect(retry=3)

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.streaming.start()
        self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
        self.running = True
        self.processing_thread.start()

    def stop(self):
        self.running = False
        self.processing_thread.join()
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        assert self.drone.streaming.stop()
        assert self.drone.disconnect()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def yuv_frame_processing(self):
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            # You should process your frames here and release (unref) them when you're done.
            # Don't hold a reference on your frames for too long to avoid memory leaks and/or memory
            # pool exhaustion.
            yuv_frame.unref()

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass


    def fly(self):
        # Takeoff, fly, land, ...
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait"
                    )
                )
            )
        ).wait()
        maxtilt = self.drone.get_state(MaxTiltChanged)["max"]
        self.drone(MaxTilt(maxtilt)).wait()

        for i in range(4):
            print(f"Moving by ({i + 1}/4)...")
            self.drone(moveBy(10, 0, 0, math.pi, _timeout=20)).wait().success()

        print("Landing...")
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("Landed\n")

def test_streaming():
    streaming_example = StreamingExample()
    # Start the video stream
    streaming_example.start()
    # Perform some live video processing while the drone is flying
    streaming_example.fly()
    # Stop the video stream
    streaming_example.stop()
    # Recorded video stream postprocessing

if __name__ == "__main__":
    test_streaming()
