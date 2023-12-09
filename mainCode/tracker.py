import numpy as np
import time
# import keyboard
import threading
from picamera2 import Picamera2, Preview
import cv2

class NutsTracker:
    def __init__(self, resolution=(320, 240), framerate=30,):
        self.record = True
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.resolution = resolution
        self.framerate = framerate
        self.frame = None
        self.stopped = False
        self.tracking = True
        self.show = True
        self.mostrar_contorno = True
        self.x = -1
        self.y = -1
        self.x_max = resolution[0]
        self.y_max = resolution[1]
        self.detect = 0
        self.obj = [0, 0]
        self.min_area = 5
        self.max_area = 20000000
        self.default_lower = np.array([3, 162, 53])
        self.default_upper = np.array([47, 255, 255])
        self.detect = 0
        self.objX = resolution[0]/2
        self.objY = resolution[1]

    
    def initiateVideo(self):
        print("Initiating video.")
        try:
            self.camera = Picamera2()  # Change this line
            self.camera.resolution = self.resolution
            self.camera.framerate = self.framerate
            if(self.show):
                self.camera.start_preview(Preview.QTGL)
            preview_config = self.camera.create_preview_configuration()
            self.camera.configure(preview_config)
        
            #self.camera.start_preview(Preview.QT)
            self.camera.start()
            if(self.record):
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # Adjust fps and frame size
            # self.rawCapture = Picamera2.capture_array("raw")
            # self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
            # Other camera setup code...
            # start the thread to read frames from the video stream
            self.x_max = self.camera.resolution[0]
            self.y_max = self.camera.resolution[1]
            time.sleep(1)

        except Exception as e:
            print(f"Error initializing camera: {e}")
        
    def track(self):
        # keep looping infinitely until the thread is stopped
        print("Tracking...")
        while not self.stopped:
            try:
                # Capture frame from the camera
                self.frame = self.camera.capture_array()
                frameHSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, self.default_lower, self.default_upper)
                contornos, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                a = 0
                detectIt = 0
                for c in contornos:
                    area = cv2.contourArea(c)
                    if (area >= self.min_area) and (self.max_area >= area):
                        detectIt = detectIt + 1
                        a = 1
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        self.x = int(M["m10"] / M["m00"])
                        self.y = int(M["m01"] / M["m00"])
                        cv2.circle(self.frame, (self.x, self.y), 7, (255, 0, 255), -1)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(self.frame, '{},{}'.format(
                            self.x, self.y), (self.x+10, self.y), font, 0.75, (255, 0, 255), 1, cv2.LINE_AA)
                        nuevoContorno = cv2.convexHull(c)
                        cv2.circle(self.frame, (self.x, self.y), max(
                            nuevoContorno[:, 0, 0].tolist()) - self.x, (0, 0, 255), 2)

                        if self.mostrar_contorno:
                            cv2.drawContours(
                                self.frame, [nuevoContorno], 0, (0, 255, 0), 3)
                        # print(f"Distancia con respecto al centro de la imagen: {x - frame.shape[1] * 0.5}")}
                print(f"detectIt: {detectIt}")
                self.detect = a
                if a == 0:
                    self.x  = self.objX
                    self.y = self.objY
                if self.show:
                    cv2.imshow('frame', self.frame)
                if self.record:
                    self.out.write(self.frame)
                    
                if cv2.waitKey(1) & 0xFF == ord('s'):
                    break 
            except Exception as e:
                print(f"Error processing frame: {e}")   
        if self.tracking:
            self.stop_tracking()
        self.finish()
        print("Tracking stopped.")    
    def stop_tracking(self):
        self.tracking = False

    def finish(self):
        print("Windows released.")
        self.stopped = True
        self.camera.close()
        if(self.record):
            self.out.release()

        cv2.destroyAllWindows()
