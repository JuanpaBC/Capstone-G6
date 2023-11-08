import cv2
import numpy as np
import serial
import time
import keyboard

mostrar_contorno = False
distancia = 0
manual_mode = False
starts = False
target_W = "COM5"
target_L = '/dev/ttyACM0'

arduino = serial.Serial(target_W, 9600, timeout=1)
time.sleep(0.1)
if arduino.isOpen():
    print("{} conectado!".format(arduino.port))
    time.sleep(1)

cap = cv2.VideoCapture(0)  # Use the default camera (change to another number if needed)

# Create a viewing window
cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)

def comunicacion(distancia):
    # Manda la distancia medida y espera respuesta del Arduino.
    if arduino.isOpen():
        arduino.write(distancia.encode())
        while arduino.inWaiting() == 0:
            continue
        if arduino.inWaiting() > 0:
            answer = arduino.readline()
            print(answer.decode())
            arduino.flushInput()  # Remove data after reading

def read_distancia(frame):
    # Define rango de colores inferior y superior.
    lower = np.array([49, 45, 0], np.uint8)
    upper = np.array([96, 255, 255], np.uint8)

    # Se aplica una máscara de colores y encuentra el contorno de la imagen
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frameHSV, lower, upper)
    contornos, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.imshow("Object Detection", frame)
    for c in contornos:
        area = cv2.contourArea(c)
        if area > 1000:
            # Se calcula el centroide del contorno
            M = cv2.moments(c)
            if (M["m00"] == 0):
                M["m00"] = 1
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            cv2.circle(frame, (x, y), 7, (255, 0, 255), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, '{},{}'.format(x, y), (x+10, y), font, 0.75, (255, 0, 255), 1, cv2.LINE_AA)
            nuevoContorno = cv2.convexHull(c)
            cv2.circle(frame, (x, y), max(nuevoContorno[:, 0, 0].tolist()) - x, (0, 0, 255), 2)

            if mostrar_contorno:
                cv2.drawContours(frame, [nuevoContorno], 0, (0, 255, 0), 3)
            # Calcula y retorna la distancia del objeto al eje central vertical de la imagen.
            distancia = x - frame.shape[1] * 0.5
            # Display the frame in a window
            
            return distancia
    return 0

while True:
    ret, frame = cap.read()
    if ret:
        if (manual_mode):
            if keyboard.is_pressed('a'):
                comunicacion('L')
            elif keyboard.is_pressed('d'):
                comunicacion('R')
            elif keyboard.is_pressed('w'):
                comunicacion('U')
            elif keyboard.is_pressed('s'):
                comunicacion('D')
            elif keyboard.is_pressed('q'):
                comunicacion('Start')
            elif keyboard.is_pressed('e'):
                comunicacion('Exit')
        else:
            if keyboard.is_pressed('q'):
                comunicacion('Start')
                starts = True
            if (starts):
                distancia = read_distancia(frame)
                comunicacion(str(distancia))
        
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
