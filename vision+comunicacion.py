import cv2
import numpy as np

import serial,time

mostrar_contorno = False
distancia = 0

arduino = serial.Serial('COM3',9600, timeout=1)
time.sleep(0.1)
if arduino.isOpen():
    print("{} conectado!".format(arduino.port))
    time.sleep(1)

def comunicacion(distancia):
    if arduino.isOpen():
        try:
            arduino.write(distancia.encode())
            #time.sleep(0.1) #wait for arduino to answer
            while arduino.inWaiting() == 0: 
                pass
            if  arduino.inWaiting() > 0: 
                answer = arduino.readline()
                print(answer.decode())
                arduino.flushInput() #remove data after reading
cap = cv2.VideoCapture(0)

while True:
    lower = np.array([49, 45, 0], np.uint8)
    upper = np.array([96, 255, 255], np.uint8)

    ret, frame = cap.read()
    
    if ret == True:
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV, lower, upper)
        contornos,_ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        #cv2.drawContours(frame, contornos, -1, (255, 0, 0), 3)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 1000:
                
                M = cv2.moments(c)
                if (M["m00"] == 0):
                    M["m00"] = 1
                x = int(M["m10"]/M["m00"])
                y = int(M["m01"]/M["m00"])
                cv2.circle(frame, (x,y), 7, (255,0,255), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame, '{},{}'.format(x, y), (x+10, y), font, 0.75, (255,0,255), 1, cv2.LINE_AA)
                nuevoContorno = cv2.convexHull(c)
                cv2.circle(frame, (x,y), max(nuevoContorno[:, 0, 0].tolist()) - x, (0,0,255), 2)
                
                if mostrar_contorno:
                    cv2.drawContours(frame, [nuevoContorno], 0, (0, 255, 0), 3)
                distancia = f"{x - frame.shape[1]*0.5}"
                # print(f"Distancia con respecto al centro de la imagen: {distancia}")
                comunicacion(distancia)
        # cv2.imshow('maskAzul', mask)
        # cv2.imshow('maskVerde', mask)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break
cap.release()
cv2.destroyAllWindows()



#vision()
#comunicacion()