import threading
import multiprocessing
import time
import cv2
from PiVideoStream import PiVideoStream
import numpy as np
import asyncio
import json
import serial
#-------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------------#
centerX = multiprocessing.Value('i', 0)
centerY = multiprocessing.Value('i', 0)
refX =  multiprocessing.Value('i', 0)
refY =  multiprocessing.Value('i', 0)
width = multiprocessing.Value('i', 256)
heigth = multiprocessing.Value('i', 192)
#-------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------------#
def video():
    global showFrame, bajoLaser, altoLaser, bajoRef, altoRef
    h = heigth.value #Altura eje Y
    w = width.value #Ancho eje X
    resolution = (w, h) # (X, Y)
    framerate = 120
    vs = PiVideoStream(resolution, framerate).start()
    time.sleep(2)
    cap = vs

    showFrame = np.zeros((h,w,3), dtype=np.uint8) # (Y, X)
    toInv = 255*np.ones((h,w,3), dtype=np.uint8) # (Y, X)
    #[color, luz, sombra]
    bajoLaser = np.array([0, 0, 0], np.uint8)
    altoLaser = np.array([179, 255, 255], np.uint8)
    bajoRef = np.array([0, 0, 0], np.uint8)
    altoRef = np.array([179, 255, 255], np.uint8)

    #variables, listas y muestras para media movil
    n_smpls = 3
    samplesMX = n_smpls*[0]
    samplesMY = n_smpls*[0]
    samplesRX = n_smpls*[0]
    samplesRY = n_smpls*[0]
    sumAcumRedX = 0
    sumAcumRedY = 0
    sumAcumRefX = 0
    sumAcumRefY = 0

    samplesRed = [samplesMX, samplesMY]
    samplesRef = [samplesRX, samplesRY]
    sumAcumRed = [sumAcumRedX, sumAcumRedY]
    sumAcumRef = [sumAcumRefX, sumAcumRefY]
    ##

    def smooth(new_sample, samples, sumAcum):

        sumAcum = sumAcum - samples[0] + new_sample

        for i in range(n_smpls-1):
            samples[i] = samples[i + 1]
        samples[n_smpls-1] = new_sample

        mean = round(sumAcum/n_smpls)
        return mean, samples, sumAcum
    
    def create_circular_kernel(radius):
        diameter = 2 * radius + 1
        kernel = np.zeros((diameter, diameter), dtype=np.uint8)
        y, x = np.ogrid[:diameter, :diameter]
        mask = (x - radius) ** 2 + (y - radius) ** 2 <= radius ** 2
        kernel[mask] = 1
        return kernel   
    
    def create_cruz_kernel(n):
        kernel = np.ones((n,n), np.uint8)
        kernel[0,0] = 0
        kernel[0,2] = 0
        kernel[2,0] = 0
        kernel[2,2] = 0
        return kernel

    def detect(frame, limBajo, limAlto, kernel, samples, sumAcum):
        global frameHSV
        posX = centerX.value
        posY = centerY.value

        # frame = cv2.medianBlur(frame, 17)
        frameInv = toInv - frame
        frameHSV = cv2.cvtColor(frameInv, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(frameHSV, limBajo, limAlto)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.erode(mask, kernel, iterations=1)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        if contornos:
            M = cv2.moments(np.array(contornos[0]))
            if M["m00"] != 0:
                posX, samples[0], sumAcum[0] = smooth(float(M["m10"] / M["m00"]), samples[0], sumAcum[0]) 
                posY, samples[1], sumAcum[1] = smooth(float(M["m01"] / M["m00"]), samples[1], sumAcum[1])
        return posX, posY, mask
    
    def tracking():
        global showFrame

        kernel_circular = create_circular_kernel(3)

        while True:
            frame = cap.read()
            # print(cap.data())
            centerX.value, centerY.value, maskRed = detect(frame, bajoLaser, altoLaser, kernel_circular, samplesRed, sumAcumRed)
            refX.value, refY.value, maskRef = detect(frame, bajoRef, altoRef, kernel_circular, samplesRef, sumAcumRef)
            showFrame = cv2.bitwise_and(frame, frame, mask=(maskRed+maskRef))
            # print("CM: ", str(centerX.value), " | ",  str(heigth.value - centerY.value), " ref: ", str(refX.value), " | ",  str(heigth.value - refY.value))
            # showFrame = frame
            # showFrame = maskRed + maskRef

    def click_event(event, x, y, flags, param):
        global bajoLaser, altoLaser, bajoRef, altoRef, frameHSV
        if event == cv2.EVENT_LBUTTONDOWN:
            hue = frameHSV[y, x, 0] # rows = y, columns = x, 3 channels; HSV
            bajoLaser = np.array([hue-20, 50, 30], np.uint8)
            altoLaser = np.array([hue+20, 255, 255], np.uint8) 

        if event == cv2.EVENT_RBUTTONDOWN:
            _hue = frameHSV[y, x, 0] # rows = y, columns = x, 3 channels; HSV
            bajoRef = np.array([_hue-20, 50, 30], np.uint8)
            altoRef = np.array([_hue+20, 255, 255], np.uint8) 

    def show():
        global showFrame
        while True:
            cv2.imshow('Track Laser', showFrame)
            cv2.setMouseCallback('Track Laser', click_event)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

    t21 = threading.Thread(target = tracking, args=())
    t22 = threading.Thread(target = show)

    t21.start()
    t22.start()

    t21.join()
    t22.join()

    
#-------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------------------#

def com2ino():
    arduino = serial.Serial('/dev/ttyUSB0', 115200)

    # def buff(value):
    #     str_value = str(value)
    #     l = len(str_value)
    #     str_out = "0"*(3 - l) + str_value
    #     return str_out
    
    # def msg_gen(CM, Ref):
    #     cmX = buff(CM[0])
    #     cmY = buff(CM[1])
    #     rfX = buff(Ref[0])
    #     rfY = buff(Ref[1])
        
    #     msg = cmX + cmY + rfX + rfY + "E" # E = End
    #     return msg

    def enviar():
        # print('working')
        while True:
            X = centerX.value
            Y = heigth.value - centerY.value

            # mensaje = msg_gen([X, Y], [xR, yR])
            arduino.write(bytes([X, Y]))
            arduino.write(b'E')

    def recibir():
        # print('working')
        while True:
            lectura = arduino.readline().decode().rstrip()
    
    t31 = threading.Thread(target = enviar, args=())
    t32 = threading.Thread(target = recibir)

    t31.start()
    t32.start()

    t31.join()
    t32.join()

if __name__ == '__main__':

    t1 = multiprocessing.Process(target = video, args=())
    t2 = multiprocessing.Process(target = com2ino, args=())

    t1.start()
    t2.start()

    t1.join()
    t2.join()