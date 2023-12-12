import numpy as np
import time
# import keyboard
import threading
from picamera2 import Picamera2, Preview
import cv2

def apply_median_filter(image, kernel_size=3):
    filtered_image = cv2.medianBlur(image, kernel_size)

    return filtered_image

class NutsTracker:
    def __init__(self, resolution=(2028, 1520), framerate=12,):
        self.record = True
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.resolution = resolution
        self.framerate = framerate
        self.frame = None
        self.stopped = False
        self.tracking = True
        self.show = False
        self.mostrar_contorno = True
        self.x = -1
        self.y = -1
        self.x_max = resolution[0]
        self.y_max = resolution[1]
        self.detect = 0
        self.obj = [0, 0]
        self.min_area = 600
        self.max_area = 14598
        self.default_lower = np.array([7,40,164])
        self.default_upper = np.array([110, 173, 255])
        self.detect = 0
        self.objX = resolution[0]/2
        self.objY = resolution[1]

    
    def initiateVideo(self):
        print("Initiating video.")
        if(True):
            self.camera = Picamera2()  # Change this line
            if(self.show):
                self.camera.start_preview(Preview.QTGL)
            preview_config = self.camera.create_preview_configuration(sensor={"output_size": self.resolution, "bit_depth": self.framerate})
            self.camera.configure(preview_config)
            #self.camera.start_preview(Preview.QT)
            self.camera.start()
            
            if(self.record):
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.outRaw = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # Adjust fps and frame siz
                fourcc2 = cv2.VideoWriter_fourcc(*'XVID')
                self.outMask = cv2.VideoWriter('output2.avi', fourcc2, 20.0, (640, 480))  # Adjust fps and frame sizee
            self.frame = self.camera.capture_array()
            self.x_max = self.frame.shape[0]
            self.y_max = self.frame.shape[1]
            time.sleep(1)
        
    def track(self):
        # keep looping infinitely until the thread is stopped
        print("Tracking...")
        while not self.stopped:
            try:
                # Capture frame from the camera
                self.frame = self.camera.capture_array()
                frameRGB = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                frameRGBMedian = apply_median_filter(frameRGB, kernel_size=3)
                frameHSV = cv2.cvtColor(frameRGBMedian, cv2.COLOR_RGB2HSV)
                mask = cv2.inRange(frameHSV, self.default_lower, self.default_upper)
                contornos, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                a = 0
                detectIt = 0
                min_dist = 10000000000000000000000000000
                for c in contornos:
                    area = cv2.contourArea(c)
                    if (area >= self.min_area) and (self.max_area >= area):
                        detectIt = detectIt + 1
                        a = 1
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        x = int(M["m10"] / M["m00"])
                        y = int(M["m01"] / M["m00"])
                        dist = (x - self.obj[0])**2 + (y - self.obj[1])**2
                        if(dist < min_dist):
                            min_dist = dist
                            self.x = x
                            self.y = y
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
                #if self.show:
                    #cv2.imshow('frame', self.frame)
                    #if cv2.waitKey(1) & 0xFF == ord('s'):
                    #    break 
                if self.record:
                    #cv2.line(self.frame, (self.objX - 50, 0), (self.objX - 50, self.objY), (0, 0, 255), 2)
                    #cv2.line(self.frame, (self.objX + 50, 0), (self.objX + 50, self.objY), (0, 0, 255), 2)
                    self.outRaw.write(frameRGBMedian)
                    self.outMask.write(self.frame)
            
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
        #if(self.show):
            #cv2.destroyAllWindows()
