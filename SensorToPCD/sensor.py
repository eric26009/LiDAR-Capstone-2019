import cv2 as cv
import numpy as np
import sys
import math


class Sensor:
    def __init__(self):
        self.cameraWidth = 720
        self.cameraHeight = 360
        self.cameraFound, self.camera = self.detect_mv_camera()
        if self.cameraFound:
            self.camera.set(3, self.cameraWidth)
            self.camera.set(4, self.cameraHeight)

    def detect_mv_camera(self):
        camera_detected = False
        # Capture from camera at location 1. The number could change depending on how many cameras are connected to this computer.
        for i in range(0, 3):
            camera = cv.VideoCapture(i)
            if camera:
                width = camera.get(3)
                height = camera.get(4)
                if (width == 640) and (height == 180):  # for Windows
                    camera_detected = True
                if (width == 960) and (height == 360):  # for Linux
                    camera_detected = True
                if camera_detected:
                    break
                camera.release()
        return camera_detected, camera

    def getFrame(self, waveLength):
        if not self.cameraFound:
            return False,

        ret, img = self.camera.read()   # reading data from Lidar unit
        # image organized as depth/amp pairs, two bytes each ** reshape does (rows, cols)
        image = img.reshape(self.cameraHeight, self.cameraWidth*2, 2)
        image = cv.flip(image, 0)
        img16 = image.view('uint16')  # interpret data pairs as UINT16
        depth = img16[:, 0::2]
        amp = img16[:, 1::2]

        return True, depth, amp, depth[:, waveLength::3], amp[:, waveLength::3]
