import cv2
import numpy as np
import os

def convert_hsv_to_rgb(image):
    # Convert the image from HSV to RGB
    rgb_image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
    return rgb_image

def convert_rgb_to_hsv(image):
    # Convert the image from RGB to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return hsv_image

def apply_median_filter(image, kernel_size=5):
    filtered_image = cv2.medianBlur(image, kernel_size)


    return filtered_image


def apply_gaussian_filter(image, kernel_size=(5, 5), sigma=0):
    filtered_image = cv2.GaussianBlur(
        image, kernel_size, sigma)

    return filtered_image

def erode_and_dilate(mask, kernel_size=(5, 5), iterations=1):
    # Create a kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)

    # Apply dilation
    dilated_mask = cv2.dilate(mask, kernel, iterations=iterations)

    # Apply erosion
    eroded_mask = cv2.erode(dilated_mask, kernel, iterations=iterations)

    return eroded_mask, dilated_mask  

local = os.getcwd()
target = [os.path.join(local, "Tests", "Vision")]
os.chdir(target[0])

image = cv2.imread('hsv_color_map.png')

mostrar_contorno = True

# Se define una matriz de numpy con valor de color inicial de 0 para la variable color1_hsv
colors_hsv = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
# Se define una matriz de numpy con valor de color inicial de 0 para la variable color1_hsv
color2_hsv = np.array([0, 0, 0])
# Se define una matriz de numpy con valor de color inicial de 0 para la variable color1_hsv
color3_hsv = np.array([0, 0, 0])
# Se definen los rangos de colores permitidos para la detección de objetos
LowerColorError = np.array([-40, -45, -75])
UpperColorError = np.array([40, 45, 75])
clicked = 0
# Se define una función que maneja los eventos del mouse
def _mouseEvent(event, x, y):
    # Se declaran las variables globales que se van a utilizar
    global color1_hsv, flag, clicked
    # Si se presiona el botón izquierdo del mouse
    if event == cv2.EVENT_LBUTTONDOWN:
        if(clicked > 2):
            clicked = 0
        # Se convierte el frame capturado a formato HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_values = hsv_frame[y, x]
        lower_ranges.append(hsv_values - LowerColorError)
        upper_ranges.append(hsv_values + UpperColorError)
        # print("Color clickeado: ",color1_hsv )
        cv2.setTrackbarPos('HMax', 'image', colors_hsv[clicked][0] + UpperColorError[0])
        cv2.setTrackbarPos('SMax', 'image', colors_hsv[clicked][1] + UpperColorError[1])
        cv2.setTrackbarPos('VMax', 'image', colors_hsv[clicked][2] + UpperColorError[1])
        cv2.setTrackbarPos('HMin', 'image', colors_hsv[clicked][0] - LowerColorError[0])
        cv2.setTrackbarPos('SMin', 'image', colors_hsv[clicked][1] - LowerColorError[1])
        cv2.setTrackbarPos('VMin', 'image', colors_hsv[clicked][2] - LowerColorError[2])
        flag = True
        clicked += 1


 #--------------------------------- Meterle valores 
def clicker(image):
    # Load image

    # Create a window
    cv2.namedWindow('image')
    cv2.resizeWindow('image', 540, 480)
    cv2.moveWindow('image', 700, 100)

    # Create trackbars for color change
    # Hue is from 0-179 for OpenCV
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

    cv2.setMouseCallback('frame', _mouseEvent)
    
    while True:
        if flag:
            break
    flag = False
    
    
    
# Initialize HSV min/max values h49 s85
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0


hMin1 = [49, 45, 0]
sMin1= []
vMin1 = []
hMax1 = [] 
sMax1 = []
vMax1 = []

lower_ranges = []
upper_ranges = []

try:
    file_path = os.path.join('valores_lower_upper.txt')
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for i, line in enumerate(lines):
            values_line = line.strip().split(
                ': ')[1].replace('[', '').replace(']', '')
            values = np.array([int(x) for x in values_line.split(',')])
            if i == 0:
                lower_ranges.append(values)
            elif i == 1:
                upper_ranges.append(values)

except FileNotFoundError:
    # Si el archivo no se encuentra, utiliza valores predeterminados
    print('Archivo no encontrado, utilizando valores predeterminados')
    lower_ranges.append(np.array([49, 45, 0], np.uint8))
    upper_ranges.append(np.array([96, 255, 255], np.uint8))

# Replace camera capture with video file capture
video_file_path = 'castan.h264'  # Change this to your video file path
cap = cv2.VideoCapture(video_file_path)

cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame', 640, 480)
cv2.moveWindow('frame', 30, 100)

# Se establece el método de captura de eventos del mouse
cv2.setMouseCallback('frame', _mouseEvent)

# Se establece el método de captura de eventos del mouse


try:
    while True:

        
        # Set minimum and maximum HSV values to display
        lower_ranges[-1] = np.array([hMin, sMin, vMin])

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        union_mask = np.zeros_like(hsv[:, :, 0])

        for lower, upper in zip(lower_ranges, upper_ranges):
            mask = cv2.inRange(hsv, lower, upper)
            union_mask = cv2.bitwise_or(union_mask, mask)

        result = cv2.bitwise_and(image, image, mask=union_mask)

        cv2.imshow('image', result)

        ret, frame = cap.read()

        if ret == True:
            frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frameHSV, lower, upper)
            mask2 = cv2.inRange(frameHSV, lower, upper)
            mask3 = cv2.inRange(frameHSV, lower, upper)
            contornos, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            # cv2.drawContours(frame, contornos, -1, (255, 0, 0), 3)
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
        else:
            cap = cv2.VideoCapture(video_file_path)

except KeyboardInterrupt:
    # Guardar los valores de lower y upper en un archivo de texto en caso de KeyboardInterrupt
    with open('valores_lower_upper.txt', 'w') as file:
        file.write(f'Lower: [{lower[0]},{lower[1]},{lower[2]}]\n')
        file.write(f'Upper: [{upper[0]},{upper[1]},{upper[2]}]\n')

finally:
    cap.release()
    cv2.destroyAllWindows()