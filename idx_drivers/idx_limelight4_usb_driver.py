#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import math
import time
import threading
import requests
import cv2

from nepi_sdk import nepi_utils


PKG_NAME = 'IDX_LIMELIGHT4_USB'
FILE_TYPE = 'DRIVER'


class Limelight4UsbCamDriver(object):

    MAX_CONSEC_FRAME_FAIL_COUNT = 3

    def __init__(self, host, stream_port=5800, api_port=5807):
        self.host = host
        self.stream_port = stream_port
        self.api_port = api_port
        self.stream_url = "http://" + host + ":" + str(stream_port)
        self.api_url = "http://" + host + ":" + str(api_port) + "/results"

        # Verify connectivity and store device info from REST API
        try:
            resp = requests.get(self.api_url, timeout=2)
            resp.raise_for_status()
            self._device_info_json = resp.json()
        except Exception as e:
            raise RuntimeError("Failed to connect to Limelight REST API at " + self.api_url + ": " + str(e))

        # Initialize acquisition state — mirrors GenericONVIF_NVT structure
        self._lock = threading.Lock()
        self._cap = None
        self._latest_frame = None
        self._latest_frame_timestamp = None
        self._latest_frame_success = False
        self._img_acq_thread = None
        self._consec_failed_frames = 0
        self._grab_timestamps = []

        # Prime and verify the MJPEG stream is reachable before returning
        ret, msg = self.start_image_acquisition()
        if ret is False:
            raise RuntimeError("Failed to prime image acquisition: " + msg)
        self.stop_image_acquisition()

    def get_device_info(self):
        """Return device identification dict parsed from the stored REST response."""
        j = self._device_info_json
        return {
            'device_name': 'Limelight 4',
            'serial_number': str(j.get('fidx', 'unknown')),
            'hw_version': str(j.get('hw', j.get('cid', 'unknown'))),
            'sw_version': 'unknown',
            'manufacturer': 'LimelightVision',
        }

    def start_image_acquisition(self):
        """Open the MJPEG stream via cv2.VideoCapture and start the background grab thread."""
        self._lock.acquire()
        if self._cap is not None:
            self._lock.release()
            return True, "Already capturing from " + self.stream_url

        self._cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
        if not self._cap.isOpened():
            self._cap.release()
            self._cap = None
            self._lock.release()
            return False, "Failed to open stream at " + self.stream_url

        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
        self._lock.release()

        self._latest_frame = None
        self._latest_frame_timestamp = None
        self._latest_frame_success = False
        self._grab_timestamps = []
        self._consec_failed_frames = 0

        self._img_acq_thread = threading.Thread(target=self.runImgAcqThread)
        self._img_acq_thread.daemon = True
        self._img_acq_thread.start()

        return True, "Success"

    def stop_image_acquisition(self):
        """Release the VideoCapture handle and stop the grab thread."""
        self._lock.acquire()
        if self._cap is None:
            self._lock.release()
            return True, "No current capture from " + self.stream_url
        self._cap.release()
        self._cap = None
        self._lock.release()
        return True, "Success"

    def image_acquisition_running(self):
        """Return True if the MJPEG stream capture handle is currently open."""
        return self._cap is not None

    def get_image(self):
        """Retrieve and decode the latest grabbed frame from the background thread.

        Returns (frame, timestamp, success, message). On MAX_CONSEC_FRAME_FAIL_COUNT
        consecutive failures the stream is automatically stopped and restarted.
        """
        if self._cap is None or not self._cap.isOpened():
            return None, None, False, "Capture not opened for " + self.stream_url

        self._lock.acquire()
        if self._latest_frame_success is True:
            ret, self._latest_frame = self._cap.retrieve()
        else:
            ret = False
            self._latest_frame = None
        self._lock.release()

        if not ret:
            self._consec_failed_frames += 1
            if self._consec_failed_frames < self.MAX_CONSEC_FRAME_FAIL_COUNT:
                return None, None, False, "Failed to read next frame from " + self.stream_url
            else:
                self.stop_image_acquisition()
                self.start_image_acquisition()
                return None, None, False, (
                    "Failed to read next frame " + str(self.MAX_CONSEC_FRAME_FAIL_COUNT) +
                    " times consecutively -- auto-restarting image acquisition"
                )

        return self._latest_frame, self._latest_frame_timestamp, True, "Success"

    def get_framerate(self):
        """Return rolling-average framerate computed from the last two grab timestamps.

        Returns (True, fps_float) when at least two timestamps are available;
        (False, 0.0) otherwise.
        """
        self._lock.acquire()
        timestamps = list(self._grab_timestamps)
        self._lock.release()
        if len(timestamps) < 2:
            return False, 0.0
        dt = timestamps[-1] - timestamps[-2]
        if dt <= 0:
            return False, 0.0
        return True, 1.0 / dt

    def get_available_resolutions(self):
        """Return fixed list of resolution dicts supported by the Limelight 4 MJPEG stream."""
        return [
            {'width': 320,  'height': 240},
            {'width': 640,  'height': 480},
            {'width': 960,  'height': 720},
            {'width': 1280, 'height': 960},
        ]

    def set_pipeline(self, index):
        """Switch the active Limelight pipeline by index via the REST API."""
        try:
            url = "http://" + self.host + ":" + str(self.api_port) + "/pipeline?index=" + str(index)
            resp = requests.get(url, timeout=2)
            resp.raise_for_status()
            return True, "Pipeline set to " + str(index)
        except Exception as e:
            return False, "Failed to set pipeline: " + str(e)

    def get_pipeline(self):
        """Return the current active pipeline index from the REST API as (bool, int)."""
        try:
            resp = requests.get(self.api_url, timeout=2)
            resp.raise_for_status()
            pipeline = int(resp.json().get('pID', 0))
            return True, pipeline
        except Exception as e:
            return False, "Failed to get pipeline: " + str(e)

    def get_imu_data(self):
        """Poll the Limelight REST endpoint and return a dict of IMU values.

        Reads the 'imu' object from the /results JSON response. The 'data' sub-array
        contains [robot_yaw, roll, pitch, raw_yaw, gyro_z, gyro_x, gyro_y, accel_z, accel_x, accel_y].
        Angles are in degrees; angular rates are in degrees/sec; accelerations are in g.

        Returns a dict with keys: robot_yaw_deg, roll_deg, pitch_deg, yaw_deg,
        gyro_z_dps, gyro_x_dps, gyro_y_dps, accel_z_g, accel_x_g, accel_y_g.
        Returns None if the REST call fails or the imu key is absent or incomplete.
        """
        try:
            resp = requests.get(self.api_url, timeout=2)
            resp.raise_for_status()
            imu = resp.json().get('imu')
            if imu is None:
                return None
            data = imu.get('data', [])
            if len(data) < 10:
                return None
            return {
                'robot_yaw_deg': data[0],
                'roll_deg':       data[1],
                'pitch_deg':      data[2],
                'yaw_deg':        data[3],
                'gyro_z_dps':     data[4],
                'gyro_x_dps':     data[5],
                'gyro_y_dps':     data[6],
                'accel_z_g':      data[7],
                'accel_x_g':      data[8],
                'accel_y_g':      data[9],
            }
        except Exception:
            return None

    def get_detection_data(self):
        """Poll the Limelight REST endpoint and return a list of NN detection dicts.

        Reads the 'Detector' array from the /results JSON response. The Limelight
        Neural Detector does not provide bounding box corners. Instead it provides
        target center in degrees (tx, ty from crosshair) and target area as a
        fraction of the image (ta). This method computes an approximate normalized
        bounding box (0.0-1.0) from those values:

          center_x_norm = 0.5 + tx / FOV_H_DEG
          center_y_norm = 0.5 - ty / FOV_V_DEG  (ty positive = up = smaller screen y)
          side_norm     = sqrt(ta)               (square approximation)

        FOV constants (90 x 70 degrees) match the default Limelight 4 configuration.
        The resulting xmin/ymin/xmax/ymax are normalized (0.0-1.0). Conversion to
        pixel coordinates is the caller's responsibility.

        Returns:
            list: Detection dicts with keys name, id, uid, prob, xmin, ymin, xmax,
                ymax, area_pixels, area_ratio. Empty list if no targets present.
            None: If the REST call fails.
        """
        FOV_H_DEG = 90.0
        FOV_V_DEG = 70.0
        try:
            resp = requests.get(self.api_url, timeout=2)
            resp.raise_for_status()
            detectors = resp.json().get('Detector', [])
            if not detectors:
                return []
            detect_dict_list = []
            for det in detectors:
                tx = float(det.get('tx', 0.0))
                ty = float(det.get('ty', 0.0))
                ta = float(det.get('ta', 0.0))
                center_x = 0.5 + tx / FOV_H_DEG
                center_y = 0.5 - ty / FOV_V_DEG
                half_side = math.sqrt(max(0.0, ta)) / 2.0
                xmin = max(0.0, center_x - half_side)
                ymin = max(0.0, center_y - half_side)
                xmax = min(1.0, center_x + half_side)
                ymax = min(1.0, center_y + half_side)
                area_ratio = max(0.0, (xmax - xmin) * (ymax - ymin))
                detect_dict_list.append({
                    'name': str(det.get('class', 'unknown')),
                    'id': int(det.get('classID', 0)),
                    'uid': '',
                    'prob': float(det.get('conf', 0.0)),
                    'xmin': xmin,
                    'ymin': ymin,
                    'xmax': xmax,
                    'ymax': ymax,
                    'area_pixels': 0,
                    'area_ratio': area_ratio,
                })
            return detect_dict_list
        except Exception:
            return None

    def runImgAcqThread(self):
        if self._cap is None or not self._cap.isOpened():
            return

        keep_going = True
        while keep_going:
            self._lock.acquire()
            if self._cap is None:
                keep_going = False
            else:
                self._latest_frame_success = self._cap.grab()
                ts = nepi_utils.get_time()
                self._latest_frame_timestamp = ts
                self._grab_timestamps.append(ts)
                if len(self._grab_timestamps) > 2:
                    self._grab_timestamps = self._grab_timestamps[-2:]
            self._lock.release()
            time.sleep(0.001)


if __name__ == '__main__':
    driver = Limelight4UsbCamDriver(host='172.29.0.1')
    print("Device info:", driver.get_device_info())
    print("Available resolutions:", driver.get_available_resolutions())
    print("Current pipeline:", driver.get_pipeline())
    print("Starting acquisition...")
    ret, msg = driver.start_image_acquisition()
    print(ret, msg)
    if ret:
        for _ in range(5):
            frame, ts, ok, msg = driver.get_image()
            print("Frame:", ok, msg)
            time.sleep(0.1)
        print("Framerate:", driver.get_framerate())
        driver.stop_image_acquisition()
