import pyrealsense2 as rs

# Set up RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

# filter
hole_filling = rs.hole_filling_filter(2)
decimation = rs.decimation_filter()

# Start streaming
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        print("OK")
finally:
    pipeline.stop()
