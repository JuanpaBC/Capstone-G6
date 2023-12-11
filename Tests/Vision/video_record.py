from picamera2.encoders import H264Encoder
from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(sensor={"output_size": (2028, 1520), "bit_depth": 12})
picam2.configure(preview_config)
encoder = H264Encoder(bitrate=10000000)
output = "test.h264"
picam2.start_preview(Preview.QTGL)
picam2.start_recording(encoder, output)
time.sleep(10)
picam2.stop_recording()
picam2.stop_preview()
