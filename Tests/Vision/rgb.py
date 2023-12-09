

import cv2
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import os

class NutsTracker:
    def __init__(self):
        self.record = True
        self.frame = None
        self.stopped = False
        self.tracking = True
        self.show = False
        self.mostrar_contorno = True
        self.x = -1
        self.y = -1
        self.x_max = 0
        self.y_max = 0
        self.detect = False
        self.obj = [0, 0]
        self.min_area = 5
        self.max_area = 20000000
        self.default_lower = 220
        self.default_upper = 255

    def track(self, image_files):
        for image_file in image_files:
            
            self.frame = cv2.imread(image_file)
            frameRGB = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            green_channel = frameRGB[:, :, 1]
            print(green_channel.shape)                                                                
            mask = np.logical_and(
                self.default_lower <= green_channel, green_channel <= self.default_upper)
            plt.imshow(mask*255, cmap='gray')
            plt.show() # Wait for a key press before continuing
            kernel = np.ones((15, 15), np.uint8)
            kernel2 = np.ones((15, 15), np.uint8)
            erosion = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
            plt.imshow(erosion,cmap='gray')
            plt.title('erosion')
            plt.show()
            opening = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel2)


            plt.imshow(opening, cmap='gray')
            plt.title('open')
            plt.show()
           

tracker = NutsTracker()
# replace with your actual image files
local  = os.getcwd()

image_files = [os.path.join(local, "Tests", "Vision", "wa1.jpg")]
print(image_files)

tracker.track(image_files)
