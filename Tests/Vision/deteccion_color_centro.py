import cv2
import numpy as np
import os

mostrar_contorno = False

# Se define una matriz de numpy con valor de color inicial de 0 para la variable color1_hsv
color1_hsv = np.array([0,0,0])
# Se definen los rangos de colores permitidos para la detección de objetos
LowerColorError = np.array([-40,-45,-75]) 
UpperColorError = np.array([40,45,75])  

# Se define una función que maneja los eventos del mouse
def _mouseEvent(event, x, y, flags, param):
	# Se declaran las variables globales que se van a utilizar
    global color1_hsv
    # Si se presiona el botón izquierdo del mouse
    if event == cv2.EVENT_LBUTTONDOWN:
        # Se convierte el frame capturado a formato HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color1_hsv = hsv_frame[y,x]
        # print("Color clickeado: ",color1_hsv )
        cv2.setTrackbarPos('HMax', 'image', color1_hsv[0] + UpperColorError[0])
        cv2.setTrackbarPos('SMax', 'image', color1_hsv[1] + UpperColorError[1])
        cv2.setTrackbarPos('VMax', 'image', 255)
        cv2.setTrackbarPos('HMin', 'image', color1_hsv[0] + LowerColorError[0])
        cv2.setTrackbarPos('SMin', 'image', color1_hsv[1] + LowerColorError[1])
        cv2.setTrackbarPos('VMin', 'image', color1_hsv[2] + LowerColorError[2])
        

def nothing(x):
    pass

# Load image
image = cv2.imread('hsv_color_map.png')

# Create a window
cv2.namedWindow('image')
cv2.resizeWindow('image', 540, 480)	
cv2.moveWindow('image', 700, 100)

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 96)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)
cv2.setTrackbarPos('HMin', 'image', 49)
cv2.setTrackbarPos('SMin', 'image', 45)
cv2.setTrackbarPos('VMin', 'image', 0)

# Initialize HSV min/max values h49 s85
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

try:
    file_path = os.path.join('valores_lower_upper.txt')
    with open(file_path, 'r') as file:
        lines = file.readlines()
        lower_line = lines[0].strip().split(': ')[1].replace('[', '').replace(']', '')
        upper_line = lines[1].strip().split(': ')[1].replace('[', '').replace(']', '')

        # Convierte los valores de string a numpy arrays
        lower = np.array([int(x) for x in lower_line.split(',')])
        upper = np.array([int(x) for x in upper_line.split(',')])


except FileNotFoundError:
    # Si el archivo no se encuentra, utiliza valores predeterminados
    print('Archivo no encontrado, utilizando valores predeterminados')
    lower = np.array([49, 45, 0], np.uint8)
    upper = np.array([96, 255, 255], np.uint8)

cap = cv2.VideoCapture(1)

# azulBajo = np.array([100, 100, 20], np.uint8)
# azulAlto = np.array([125, 255, 255], np.uint8)

# verdeBajo = np.array([40, 120, 20], np.uint8)
# verdeAlto = np.array([120, 210, 255], np.uint8)

cv2.namedWindow('frame',  cv2.WINDOW_NORMAL )	
cv2.resizeWindow('frame', 640, 480)
cv2.moveWindow('frame', 30, 100)

# Se establece el método de captura de eventos del mouse
cv2.setMouseCallback('frame',_mouseEvent)

cv2.setTrackbarPos('HMax', 'image', upper[0])
cv2.setTrackbarPos('SMax', 'image', upper[1])
cv2.setTrackbarPos('VMax', 'image', upper[2])
cv2.setTrackbarPos('HMin', 'image', lower[0])
cv2.setTrackbarPos('SMin', 'image', lower[1])
cv2.setTrackbarPos('VMin', 'image', lower[2])

try:
    while True:

        # Get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')
        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Convert to HSV format and color threshold
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(image, image, mask=mask)

        
        cv2.imshow('image', result)

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
                    print(f"Distancia con respecto al centro de la imagen: {x - frame.shape[1]*0.5}")
            # cv2.imshow('maskAzul', mask)
            # cv2.imshow('maskVerde', mask)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('s'):
                break
except KeyboardInterrupt:
    # Guardar los valores de lower y upper en un archivo de texto en caso de KeyboardInterrupt
    with open('valores_lower_upper.txt', 'w') as file:
        file.write(f'Lower: [{lower[0]},{lower[1]},{lower[2]}]\n')
        file.write(f'Upper: [{upper[0]},{upper[1]},{upper[2]}]\n')

finally:
    cap.release()
    cv2.destroyAllWindows()