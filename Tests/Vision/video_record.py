from picamera2.encoders import H264Encoder
from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2()
picam2.resolution = (320, 240)
picam2.framerate = 30
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)
encoder = H264Encoder(bitrate=10000000)
output = "test.h264"
picam2.start_preview(Preview.QTGL)
picam2.start_recording(encoder, output)
time.sleep(10)
picam2.stop_recording()
picam2.stop_preview()