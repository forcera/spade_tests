import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, NavigateHome
import threading
import time
import queue
import cv2
import logging


class OlympeStreaming(threading.Thread):
    def __init__(self, drone):
        self.drone = drone
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.frame_num = 0
        self.renderer = None
        super().__init__()
        super().start()

    def start(self):
        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.streaming.start()
        # self.renderer = PdrawRenderer(pdraw=self.drone.streaming)

    def stop(self):
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        self.drone.streaming.stop()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        pass

    def display_frame(self, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()

        height, width = (  # noqa
            info["raw"]["frame"]["info"]["height"],
            info["raw"]["frame"]["info"]["width"],
        )

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)
        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[yuv_frame.format()]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        cv2.imshow("Frames via Olympe", cv2frame)
        cv2.waitKey(1)

    def run(self):
        while True:
            try:
                yuv_frame = self.frame_queue.get()
                print(yuv_frame)
                self.display_frame(yuv_frame)
                yuv_frame.unref()
            except queue.Empty:
                print('empty')
                continue


#logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # eventually IP will be specified depending on what drone is chosen
    olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})
    IP = "10.202.0.1"
    drone = olympe.Drone(IP)
    drone.connect()
    drone(olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="hovering", _policy="check")
                                    | olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="flying", _policy="check")
                                    | (olympe.messages.ardrone3.GPSSettingsState.GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                                       >> (olympe.messages.ardrone3.Piloting.TakeOff(_no_expect=True) & olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="hovering", _timeout=10,
                                                                                         _policy="check_wait")))).wait()
    print('[drone_utils.flight_control] Drone took off!')

    streamer = OlympeStreaming(drone)
    streamer.start()

    ### Flight commands here ###
    time.sleep(60)

    streamer.stop()

    drone(olympe.messages.ardrone3.Piloting.Landing() >> olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="landed")).wait().success()

    drone.disconnect()
