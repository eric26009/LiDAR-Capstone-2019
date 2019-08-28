#!/usr/bin/env python
# -*- coding: utf-8 -*-
#__copyright__ = "©2019 MicroVision, Inc."
#__date__ = "March 12, 2019"
""" 
Quad Viewer for MicroVision LiDAR module

Displays four concurrent views of the data available from the module.
"""

import cv2 as cv
import numpy as np
import sys
import argparse
import math

font       = cv.FONT_HERSHEY_SIMPLEX
fontScale  = 0.6
fontColor  = (0,255,0)  # R,G,B
lineType   = 1

FOV_H_DEGREES = 64.0
FOV_V_DEGREES = 36.0
XZ_OUTPUT_ROWS = 360
YZ_OUTPUT_COLS = 360

Z_DEPTH_OFFSET = 0    # Depth in mm at which display starts
Y_TILT_VIRTUAL = -45  # Angle in degrees for virtual perspective in lower right quadrant

#===== Generate correction table for radial depth to Cartesian Z ======
def depthRadialCorrectionTable(inputShape, horizFOV, vertFOV, tiltY) :
    rows,cols = inputShape
    correctXZ = np.empty((rows,cols))
    ZcorrectH = np.empty(cols,dtype=float)	
    ZcorrectV = np.empty(rows,dtype=float)
    radiansPerCol = math.radians(horizFOV/cols)
    radiansPerRow = math.radians(vertFOV/rows)
    yTiltRadians = math.radians(tiltY)
    halfCols = cols*0.5
    halfRows = rows*0.5
    for h, hCorrection in enumerate(ZcorrectH):
        ZcorrectH[h] = math.cos((h-halfCols)*radiansPerCol)
    for v, vCorrection in enumerate(ZcorrectV):
        ZcorrectV[v] = math.cos((v-halfRows)*radiansPerRow-yTiltRadians)
    for h, hCorrection in enumerate(ZcorrectH):
        for v, vCorrection in enumerate(ZcorrectV):
            correctXZ[v,h] = hCorrection * vCorrection
    return correctXZ  # result is floatingPt matrix, same size as input matrix

#===== Map a UINT16 input XY depth frame to a UINT8 output XZ density frame where X is horizontal angle, Z is depth ======
def mapXZ(depthFrame, radialCorrection, outputRowsZ, zMin,zMax, zOffset, yMinPct,yMaxPct, withYZ) :
    inputRows, inputCols = depthFrame.shape
    frameXZ = np.empty((outputRowsZ,inputCols), dtype=np.uint16) # destination for XZ data
    frameYZ = np.empty((inputRows,outputRowsZ), dtype=np.uint16) # destination for YZ data
    depthScaleFactor = float(outputRowsZ)/float(zMax-zMin)
    depthCorrected = (radialCorrection*(depthFrame-zOffset)-zMin)*depthScaleFactor
    depthCorrectInt = depthCorrected.astype(int)
    depthCorrect = np.clip(depthCorrectInt,0,outputRowsZ-1)
    depthCorrectYZ = np.clip(depthCorrectInt,0,outputRowsZ-1)
    yWeight = np.ones(inputRows)
    yMin = (inputRows*yMinPct)/100
    yMax = (inputRows*yMaxPct)/100
    for i, w in enumerate(yWeight):
        if i < yMin or i > yMax: 
            yWeight[i] = 0.2
    for col in range(inputCols):
        frameXZ[:,col] = np.bincount(depthCorrect[:,col],yWeight,minlength=outputRowsZ)
    if (withYZ):
        for row in range(inputRows):
            frameYZ[row,:] = np.bincount(depthCorrectYZ[row,:],weights=None,minlength=outputRowsZ)
    finalScale = 1000/(yMaxPct-yMinPct)
    frameXZ *= finalScale
    return frameXZ, frameYZ


def start_streaming(cam, height ,width, mirror=False, physical_tilt=0.0):

    DISPLAY_WIDTH  = width
    DISPLAY_HEIGHT = height
    showLabels = True
    viewSide = False
    os_vflip = False
    if sys.platform=="win32":
        os_vflip = True
    XZ_zMin = 0
    XZ_zMax = 6000.0
    tilt_zMin = 0
    tilt_zMax = XZ_zMax * math.cos(math.radians(Y_TILT_VIRTUAL))

    GLABEL_LEFT_X   = 10
    GLABEL_RIGHT_X  = 10 + DISPLAY_WIDTH
    GLABEL_UPPER_Y  = 18
    GLABEL_LOWER_Y  = 18  + DISPLAY_HEIGHT
    GLABEL2_LOWER_Y = 38 + DISPLAY_HEIGHT
    radialCorrectionXZ = depthRadialCorrectionTable((height,width), FOV_H_DEGREES, FOV_V_DEGREES, physical_tilt)
    radialCorrectionTilt = depthRadialCorrectionTable((height,width), FOV_H_DEGREES, FOV_V_DEGREES, Y_TILT_VIRTUAL)
    
    while(1):
        ret, img = cam.read()
        if img is None:
            print "Error: No stream frames available"
            raw_input("Press Enter to exit...")
            break
        if not ret:
            continue
        # frame data reformat: from interleaved to separated UINT16 depth, intensity
        image = img.reshape(height, width*2, 2) #image organized as intensity/depth pairs, two bytes each
        if os_vflip:
            image = cv.flip(image,0) #vertical flip -- not needed for Linux
        img16 = image.view('uint16') #interpret byte pairs as UINT16
        depth = img16[:,0::2]
        amp   = img16[:,1::2]

        #horizontal flip (mirror)
        if mirror:
            depth = cv.flip(depth,1)
            amp   = cv.flip(amp,1)

        depth16 = depth.reshape(height, width)
        imgXZ, imgYZ = mapXZ(depth16, radialCorrectionXZ, XZ_OUTPUT_ROWS, XZ_zMin, XZ_zMax, Z_DEPTH_OFFSET, 0, 100,viewSide)
        depthColorScaleFactor = 210.0/XZ_zMax
        depth8 = cv.convertScaleAbs(depth, cv.CV_8UC1, depthColorScaleFactor, -0.02);
        amp8   = cv.convertScaleAbs(amp,   cv.CV_8UC1, 0.11, -0.06);
        dmapXZ  = cv.convertScaleAbs(imgXZ, cv.CV_8UC1, 6, 0)
        if viewSide:
            dmapYZ  = cv.convertScaleAbs(imgYZ, cv.CV_8UC1, 30, 0)
            dmapYZ_Colored = cv.applyColorMap(dmapYZ, cv.COLORMAP_HOT)
        else:
            imgTilt, imgYZ = mapXZ(depth16, radialCorrectionTilt, XZ_OUTPUT_ROWS, tilt_zMin, tilt_zMax, Z_DEPTH_OFFSET,0,100,False)
            dmapTilt  = cv.convertScaleAbs(imgTilt, cv.CV_8UC1, 10, 0)
            dmapTilt_Colored = cv.applyColorMap(dmapTilt, cv.COLORMAP_OCEAN)
            dmapYZ_Colored = cv.flip(dmapTilt_Colored,0)

        depthColored = cv.applyColorMap(depth8, cv.COLORMAP_JET)  #COLORMAP_HOT, COLORMAP_RAINBOW, COLORMAP_JET, HSV etc
        #depthColored = cv.medianBlur(depthColored,3)
        #depthColored = cv.blur(depthColored,(5,5), (-2,-2))
        #depthColored = cv.bilateralFilter(depthColored,7,21,7)
        ampColored     = cv.applyColorMap(amp8, cv.COLORMAP_HOT)
        dmapXZ_Colored = cv.applyColorMap(dmapXZ, cv.COLORMAP_OCEAN)
        if (width > 700) : # 3-channel mode
            depthColored = cv.GaussianBlur(depthColored,(15,1),0,0)
            ampColored = cv.GaussianBlur(ampColored,(15,1),0,0)
            dmapXZ_Colored = cv.GaussianBlur(dmapXZ_Colored,(15,1),0,0)
            if not viewSide:
                dmapYZ_Colored = cv.GaussianBlur(dmapYZ_Colored,(15,1),0,0)
        # Arrange depth, ampl side by side
        stack2D = np.hstack((depthColored, ampColored))
        # match height of XZ, YZ maps and arrange side by side
        dmapXZ_Scaled = cv.resize(dmapXZ_Colored, (np.size(depthColored,1), np.size(depthColored,0)), interpolation = cv.INTER_NEAREST)
        dmapYZ_Scaled = cv.resize(dmapYZ_Colored, (np.size(dmapXZ_Scaled,1), np.size(dmapXZ_Scaled,0)), interpolation = cv.INTER_NEAREST)
        stackProjected = np.hstack((dmapXZ_Scaled, dmapYZ_Scaled))
        grid = np.vstack((stack2D, stackProjected))
        cv.namedWindow("MicroVision LiDAR",cv.WINDOW_NORMAL);
        if showLabels:
            cv.putText(grid,'Depth Color Scale', (GLABEL_LEFT_X, GLABEL_UPPER_Y ),font,fontScale,fontColor,lineType)
            cv.putText(grid,'Reflected Intensity', (GLABEL_RIGHT_X, GLABEL_UPPER_Y),font,fontScale,fontColor,lineType)
            dmapText = 'Top View, Display Range='+str(XZ_zMax/1000.0)+'m'
            cv.putText(grid,dmapText, (GLABEL_LEFT_X,GLABEL_LOWER_Y),font,fontScale,fontColor,lineType)
            if viewSide:
                cv.putText(grid,'Side View', (GLABEL_RIGHT_X,GLABEL_LOWER_Y),font,fontScale,fontColor,lineType)
                cv.putText(grid,'Depth', (GLABEL_RIGHT_X,GLABEL2_LOWER_Y),font,fontScale,fontColor,lineType)
                cv.line(grid,(GLABEL_RIGHT_X+60,GLABEL_LOWER_Y+16),(GLABEL_RIGHT_X+90,GLABEL_LOWER_Y+16),fontColor,2) # side arrow using 3 lines
                cv.line(grid,(GLABEL_RIGHT_X+80,GLABEL_LOWER_Y+11),(GLABEL_RIGHT_X+90,GLABEL_LOWER_Y+16),fontColor,2)
                cv.line(grid,(GLABEL_RIGHT_X+80,GLABEL_LOWER_Y+21),(GLABEL_RIGHT_X+90,GLABEL_LOWER_Y+16),fontColor,2)
            else:
                cv.putText(grid,'Vertical Tilt View', (GLABEL_RIGHT_X,GLABEL_LOWER_Y),font,fontScale,fontColor,lineType)
            cv.putText(grid,'Depth', (GLABEL_LEFT_X,GLABEL2_LOWER_Y),font,fontScale,fontColor,lineType)
            cv.line(grid,(GLABEL_LEFT_X+7,GLABEL_LOWER_Y+50),(GLABEL_LEFT_X+2,GLABEL_LOWER_Y+44),fontColor,2) # down arrow using 3 lines
            cv.line(grid,(GLABEL_LEFT_X+7,GLABEL_LOWER_Y+28),(GLABEL_LEFT_X+7,GLABEL_LOWER_Y+50),fontColor,2)
            cv.line(grid,(GLABEL_LEFT_X+7,GLABEL_LOWER_Y+50),(GLABEL_LEFT_X+12,GLABEL_LOWER_Y+44),fontColor,2)
        cv.imshow("MicroVision LiDAR", grid)

        key = cv.waitKey(1)
        if key == ord('L') or key == ord('l'):   # 'L' to toggle labels on/off
            showLabels = not showLabels
        if key == 32:   # Spacebar
            cv.imwrite("DepthGrayscale_16bit.PNG", depth)
            cv.imwrite("DepthColorscaleRGB24.PNG", depthColored)
            cv.imwrite("ColorGrid4.PNG", grid)
            print "saved Depth image"
        if key == ord('-') or key == ord('_'): # Control the displayed depth range
            if XZ_zMax >= 2500:  
                XZ_zMax -= 1000
                tilt_zMax = XZ_zMax * math.cos(math.radians(Y_TILT_VIRTUAL))
        if key == ord('+') or key == ord('='):   # Control the displayed depth range
            if XZ_zMax <= 8000:  
                XZ_zMax += 2000
                tilt_zMax = XZ_zMax * math.cos(math.radians(Y_TILT_VIRTUAL))
        if key == ord('v'):   # Toggle the side/tilt view
            viewSide = not viewSide  
        if key == 27 or key==3 or key==17 or key==26: # Esc, Ctrl+C, Ctrl+Q, Ctrl+Z
            break

def detect_mv_camera():
    camera_detected = False
    # Capture from camera at location 1. The number could change depending on how many cameras are connected to this computer.
    for i in range (0, 3) :
        camera = cv.VideoCapture(i)

        if camera :
            width = camera.get(3)
            height = camera.get(4)
            if sys.platform=="win32":
                if (width == 640) and (height == 180): # for Windows
                    camera_detected = True
            else: #Linux
               if (width == 960) and (height == 360) : # for Linux 
                    camera_detected = True
            if camera_detected:
				break;
            camera.release()

    return camera_detected, camera

def execute():
    parser = argparse.ArgumentParser(description = "Visualize data from LiDAR sensor")
    parser.add_argument('--w', default = 720, metavar = "Width", type = int, help = 'Width of frame (default = 720)')
    parser.add_argument('--h', default = 360, metavar = "Height", type = int, help = 'Height of frame (default = 360)')
    parser.add_argument('--mirror', action='store_true', help="Mirror view: apply this option when display is facing the LiDAR scan field")
    parser.add_argument('--ptilt', default=0, metavar = "TiltAngle", help="Physical downward tilt angle (deg) of the LiDAR device as mounted. (default = 0)")
    args = parser.parse_args()
    in_height =  args.h
    in_width = args.w
    mirror = args.mirror
    ptilt = -float(args.ptilt)

    camera_width = 0
    camera_height = 0
    if in_width == 120 :
        camera_height = 720
    elif in_width == 480 :
        camera_height = 180
    elif in_width == 360 :
        camera_height = 720
    elif in_width == 720 :
        camera_height = 360
    else :
        print "Unsupported input resolution."
        
    camera_width = (in_width*4)/3   #input width is in RGB8 format. Camera accepts RGB24 format.

    if camera_height > 0 :
        
        foundLidar, cam = detect_mv_camera()

        if foundLidar :
            cam.set(3, camera_width)    # set the width cv.cv.CV_CAP_PROP_FRAME_WIDTH
            cam.set(4, camera_height)   # set the height cv.cv.CV_CAP_PROP_FRAME_HEIGHT 	
            print "LiDAR started. Hit <Esc> key to quit, <spacebar> to save depth image"
            start_streaming(cam, camera_height, in_width, mirror=mirror, physical_tilt=ptilt)
            cv.destroyAllWindows() 
            cam.release()
        else :
            print "Couldn't detect MicroVision LiDAR. Check power and connection."
            raw_input("Press Enter to exit...")

execute()
