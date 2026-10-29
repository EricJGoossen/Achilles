"""
A stand-in for the sentry's RealSense: blank frames at the camera's rate.

The camera isn't modelled yet. Otto's main loop is clocked by camera frames (and autonav
updates once per frame), so this source keeps that loop running at the RealSense's 60 Hz with
the frames stamped and delayed as the real camera's are. The frames are uniform gray: ArUco
localization finds no tags and the detector no plates.
"""

import queue
import threading
import time
from typing import Any, Tuple

import numpy as np
import pyrealsense2 as rs  # type: ignore

from project_otto.camera.image import ImagesetSource
from project_otto.camera.imagesets import RealsenseImageset
from project_otto.camera.intrinsics import DistortionModel, Intrinsics
from project_otto.core.timestamps import JetsonTimestamp

# A RealSense D435's color camera at 960x540.
WIDTH, HEIGHT, FX, FY, PPX, PPY = 960, 540, 690.0, 690.0, 480.0, 270.0
FRAME_PERIOD_S = 1.0 / 60.0
# Between exposure and the frame reaching Otto ("typically 18-35ms", RealsenseImagesetSource).
DELIVERY_LATENCY_S = 0.025


def realsense_intrinsics() -> Tuple[Intrinsics, Any]:
    """The camera's intrinsics, as Otto's Intrinsics and as pyrealsense2's."""
    intrinsics = Intrinsics(WIDTH, HEIGHT, PPX, PPY, FX, FY, DistortionModel.BROWN_CONRADY, [0.0] * 5)
    rs_intrinsics = rs.intrinsics()
    rs_intrinsics.width, rs_intrinsics.height = WIDTH, HEIGHT
    rs_intrinsics.ppx, rs_intrinsics.ppy = PPX, PPY
    rs_intrinsics.fx, rs_intrinsics.fy = FX, FY
    rs_intrinsics.model = rs.distortion.brown_conrady
    rs_intrinsics.coeffs = [0.0] * 5
    return intrinsics, rs_intrinsics


class BlankRealsenseImagesetSource(ImagesetSource[JetsonTimestamp]):
    """A RealsenseImagesetSource stand-in delivering blank frames."""

    def __init__(self):
        intrinsics, self._rs_intrinsics = realsense_intrinsics()
        super().__init__(intrinsics)
        self._color = np.full((HEIGHT, WIDTH, 3), 110, dtype=np.uint8)
        self._depth = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
        self._queue: "queue.Queue[RealsenseImageset]" = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._poll_loop, name="blank_camera", daemon=True)
        self._thread.start()

    def _poll_loop(self):
        next_capture = time.time()
        while True:
            capture = next_capture
            next_capture += FRAME_PERIOD_S
            time.sleep(max(0.0, capture + DELIVERY_LATENCY_S - time.time()))
            imageset = RealsenseImageset(
                self._color, self._depth, JetsonTimestamp(int(capture * 1e6)), self.intrinsics, self._rs_intrinsics
            )
            if self._queue.full():
                try:
                    _ = self._queue.get_nowait()
                except queue.Empty:
                    pass
            self._queue.put(imageset)

    def wait_for_imageset(self) -> RealsenseImageset:
        """Blocks until the next frame, like RealsenseImagesetSource.wait_for_imageset."""
        return self._queue.get(block=True, timeout=2)

    def close(self):
        """Nothing to release."""
