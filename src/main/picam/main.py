#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys

import numpy as np

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink
from networktables import NetworkTablesInstance
import cv2

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // ~~optional~~ required
#               "height": <video mode height>            // ~~optional~~ required
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        NetworkTablesInstance.NotifyFlags.IMMEDIATE |
        NetworkTablesInstance.NotifyFlags.NEW |
        NetworkTablesInstance.NotifyFlags.UPDATE)

    return server

# 80 = more false positives like faces, 120 = doesn't recognize far away cargo
MIN_SAT = 80
# 70 or 100 
MIN_VAL = 100


def hue_range(hsvImg, minHue, maxHue):
    return cv2.inRange(hsvImg, (minHue, MIN_SAT, MIN_VAL), (maxHue, 255, 255))

def mask_yellow(hsvImg):
    return hue_range(hsvImg, 17, 35)

def mask_blue(hsvImg):
    return hue_range(hsvImg, 95, 120)

def mask_red(hsvImg):
    return cv2.bitwise_or(hue_range(hsvImg, 0, 10), hue_range(hsvImg, 165, 180))

def find_balls(masked):
    # open to remove noise (erode, then dilate)
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    # eroded = cv2.erode(masked, kernel, iterations=1)
    # opened = cv2.dilate(eroded, kernel, iterations=2)

    i, contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    balls = []
    for i, c in enumerate(contours):
        (x, y), radius = cv2.minEnclosingCircle(c)
        # filter out really small objects
        if radius < 10: # and CHECKS:
            continue
        # filter out non-round objects
        rx, ry, w, h = cv2.boundingRect(c)
        fullness = cv2.contourArea(c) / (w * h)
        # balls have fullness around .5,
        #  but far in the distance balls can be lower
        if fullness < 0.5:
            continue

        aspectRatio = w / h
        # balls usually have aspect ratio >1 (they're round)
        if aspectRatio < 0.75:
            continue
        balls.append((x, y, radius))
    return balls

def proc_img(frame):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    balls = find_balls(mask_yellow(hsv))
    if balls:
        return max(balls, key=lambda b: b[2])

def main(ntinst):
    cam = cameras[0]
    config = json.loads(cam.getConfigJson())
    width = config["width"]
    height = config["height"]

    sink = CameraServer.getInstance().getVideo()
    img = np.zeros((height, width, 3), dtype=np.uint8)
    halfwidth = width / 2
    halfheight = height / 2
    last = 0
    while True:
        ts_us, img = sink.grabFrame(img)
        if ts_us == 0:
            continue
        ts = ts_us / 1e6
        dt = ts - last
        last = ts
        fps = 1 / dt
        #sd.putNumber("FPS", fps)

        b = proc_img(img)
        if b:
            x, y, r = b
            # make values in range -1 to 1
            x = (x - halfwidth) / halfwidth
            y = (y - halfheight) / halfheight
        else:
            x = y = r = -99
        sd.putNumber("BallX", x)
        sd.putNumber("BallY", y)
        sd.putNumber("BallR", r)

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
        ntinst.startDSClient()

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    main(ntinst)
