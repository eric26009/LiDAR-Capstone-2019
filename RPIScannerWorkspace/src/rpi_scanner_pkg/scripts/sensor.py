import cv2 as cv
import numpy as np
import sys
import math
import threading
from wavelength import WaveLengthIDs
from pointcloud import PointCloud

HORIZONTAL_FOV_IN_DEG = 64
VERTICAL_FOV_IN_DEG = 36

class Sensor:
    def __init__(self):
        self.pointCloud = None
        self.frame = None
        self.depthValues = None
        self.cameraWidth = 720
        self.cameraHeight = 360
        self.frameRate = 20
        self.freshFrame = False
        # trim vars
        self.desiredhfov = 30.0

        while(True):
            self.cameraFound, self.camera = self.detect_mv_camera()
            if self.cameraFound:
                print("CAMERA FOUND")
                break
            else :
                print "CAMERA NOT FOUND"
        width_reformat = (self.cameraWidth*4)/3
        self.captureThread = threading.Thread(target=self.__captureLoop)
        self.lock = threading.Lock()
        self.camera.set(3, width_reformat)
        self.camera.set(4, self.cameraHeight)
        self.camera.set(5, self.frameRate)  # 20 Hz enables single channel mode
        self.captureThread.start()
        

    def cleanup(self):
        self.camera.release()

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
                camera.set(cv.CAP_PROP_BUFFERSIZE,1)
        return camera_detected, camera

    def __captureLoop(self) :
        while True :
            # self.__timeAveragedFrames()
            cameraConnected, allDepth, allAmp, depthValues, amplitudeValues = self.__getFrame(WaveLengthIDs['885'])
            self.__saveDepth(allDepth)
            
    def __timeAveragedFrames(self):
        depth_avg_captured = np.zeros(shape=(360,720,1))
        for i in range(30):
            cameraConnected, allDepth, allAmp, depthValues, amplitudeValues = self.__getFrame(WaveLengthIDs['885'])
            if (not cameraConnected):
                return
            depth_avg_captured = np.add(allDepth, depth_avg_captured)
        depth_avg_captured = depth_avg_captured / 30
        self.__saveDepth(depth_avg_captured)

    def __getFrame(self, waveLength):
        if not self.cameraFound:
            return False,
        ret, img = self.camera.read()   # reading data from Lidar unit
        # image organized as depth/amp pairs, two bytes each ** reshape does (rows, cols)
        image = img.reshape(self.cameraHeight, self.cameraWidth*2, 2)
        # image = cv.flip(image, 0)i
        img16 = image.view('uint16')  # interpret data pairs as UINT16
        depth = img16[:, 0::2]
        amp = img16[:, 1::2]

        return True, depth, amp, depth[:, waveLength::3], amp[:, waveLength::3]

    def __saveDepth(self, depth):
        self.lock.acquire()
        self.freshFrame = True
        self.depthValues = depth
        self.lock.release()

    def __getDepth(self):
        self.lock.acquire()
        depth = self.depthValues
        isFresh = self.freshFrame
        self.freshFrame = False # not fresh anymore
        self.lock.release()
        return depth, isFresh

    def snapPointCloud(self) :
        depthValues = self.__getDepth() # /TODO not considering freshness
        self.pointCloud = PointCloud(
            depthValues,
            HORIZONTAL_FOV_IN_DEG,
            32.0,
            VERTICAL_FOV_IN_DEG
        )
        return True

    def snapFrame(self) :
        if(not self.cameraFound):
            return False
        frame,isFresh = self.__getDepth()
        # trim frame
        ratio = self.desiredhfov / HORIZONTAL_FOV_IN_DEG
        newPixelWidth = ratio * self.cameraWidth
        pixelsToRemove = int((self.cameraWidth - newPixelWidth)/2) 
        print("leftmost edge: ", pixelsToRemove, "rightmost edge: ", self.cameraWidth - pixelsToRemove)
        self.frame = frame[:, pixelsToRemove : (self.cameraWidth - pixelsToRemove)]

        print("new frame shape:", "raw frame shape: ", self.frame.shape)
        
        return isFresh


    def getTimeAveragedFrameProperly(self,n):
        # get initial frame
        while(not self.snapFrame()):
            continue
        MIN_VALID_DEPTH = 1
        frameSum = np.zeros(self.frame.shape, dtype=np.uint32)
        frameCount = np.zeros_like(self.frame)
        # sum n frames
        for i in range(n) :
            # get new frame
            while(not self.snapFrame() ):
                continue
            # mask out the small values and accumulate the others
            depthMasked = np.ma.masked_less(self.frame, MIN_VALID_DEPTH, copy=True)
            frameCount[depthMasked.mask == False] += 1
            frameSum[depthMasked.mask == False] += self.frame[depthMasked.mask == False]
        # For elements with zero count, sum is also zero. Set count=1 to avoid div0
        frameCount[frameCount==0] = 1
        frameAve = frameSum / frameCount
        frameAve = frameAve + 25
        return frameAve
        
    
    def getTimeAveragedFrame(self, n):
        validPointCounts = {}
        # get initial frame
        while(not self.snapFrame()):
            continue
        averagedFrame = self.frame

        # sum n frames
        for i in range(n) :
            # get a fresh frame
            while(not self.snapFrame() ):
                continue
            # sum depth values
            for rowi,rowArr in enumerate(self.frame):
                for coli,depth in enumerate(rowArr):
                    # check that point is in counts dictionary
                    if((rowi,coli) not in validPointCounts):
                        validPointCounts[(rowi,coli)] = 0
                    # sum and keep track of valid frame count
                    validPointCounts[(rowi,coli)] += 1 if depth > 0 else 0
                    averagedFrame[rowi][coli] += depth
        
        # calculate average
        for rowi,rowArr in enumerate(averagedFrame):
            for coli,depth in enumerate(rowArr):
                nValidFrames = validPointCounts[(rowi,coli)]
                # protect againts division by 0
                nValidFrames = nValidFrames if nValidFrames > 0 else 1 
                averagedFrame[rowi][coli] = depth / nValidFrames
        
        return averagedFrame