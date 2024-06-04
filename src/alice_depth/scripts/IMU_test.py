import numpy as np             
import pyrealsense2 as rs 

pipeline = rs.pipeline()
config = rs.config()

# Configuring streams at different rates
# Accelerometer available FPS: {100, 200}Hz
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # acceleration
# Gyroscope available FPS: {200,400}Hz
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)  # gyroscope

pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Get the gyroscope frame
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if gyro_frame:
            # Get the gyroscope data
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()

            # Print the gyroscope data
            print("Gyroscope: x={:.2f}, y={:.2f}, z={:.2f}".format(gyro_data.x, gyro_data.y, gyro_data.z))

except KeyboardInterrupt:
    pass

finally:
    # Stop streaming
    pipeline.stop()

