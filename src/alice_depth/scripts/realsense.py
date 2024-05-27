#! /usr/bin/python

import pyrealsense2 as rs
import numpy as np
import cv2


# Create a context object
ctx = rs.context()

# Get the device
devices = ctx.query_devices()
if len(devices) == 0:
    print("No device connected")
    exit(1)

# pipe = rs.pipeline()

# #For the config
# cfg = rs.config()

# #Enable the streams for data
# cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# #To actually launch the streaming
# pipe.start(cfg)


# while True:
#     frame = pipe.wait_for_frames()
#     depth_frame = frame.get_depth_frame
#     color_frame = frame.get_color_frame

#     #To convert to numpy arrays so we can visualise them in OpenCV
#     depth_image = np.asanyarray(depth_frame.get_data())
#     color_image = np.asanyarray(color_frame.get_data())

#     #Show them in openCV
#     cv2.imshow('depth', depth_image)
#     cv2.imshow('rgb', color_image)

#     if cv2.waitKey(1) == 113:
#         break


# pipe.stop()
