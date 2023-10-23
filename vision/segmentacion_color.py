# Se importan las librerías necesarias
import numpy as np
import cv2

# Se define una variable global llamada nClick
global nClick

# Se establece el valor de la cámara, 0 es la cámara del notebook
nCam = 0

# Se crea un objeto VideoCapture para capturar video de la cámara especificada
cap = cv2.VideoCapture(nCam) 	

# Se define una matriz de numpy con valor de color inicial de 0 para la variable color1_hsv
color1_hsv = np.array([0,0,0])

# Se definen los rangos de colores permitidos para la detección de objetos
LowerColorError = np.array([-40,-45,-75]) 
UpperColorError = np.array([40,45,75])  

# Se inicializa la variable nClick en 1
nClick = 1


info_text = f"Seleciona el color con un click"
info_text2 = ''

# Se define una función que maneja los eventos del mouse
def _mouseEvent(event, x, y, flags, param):
	# Se declaran las variables globales que se van a utilizar
	global nClick
	global color1_hsv
	global info_text
	global info_text2

	# Si se presiona el botón izquierdo del mouse
	if event == cv2.EVENT_LBUTTONDOWN:
		# Se convierte el frame capturado a formato HSV
		hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		if(nClick == 1):
			# Si es el primer click se guarda el color seleccionado en la variable color1_hsv
			color1_hsv = hsv_frame[y,x]
			print("Color 1: ",color1_hsv )
			print("Eje x: ", x)
			print("Eje y: ",y)
			info_text = f"Color Seleccionado: {color1_hsv}"
			info_text2 = "[Has click para reiniciar]"
			nClick += 1
		else:
			# Si es el segundo click se reinicia la variable nClick a 1
			info_text = f"Seleciona el color con un click"
			info_text2 = ''
			color1_hsv = np.array([0,0,0])
			nClick = 1

# Se crean dos ventanas de visualización de la imagen original y la imagen segmentada
cv2.namedWindow('Imagen Original',  cv2.WINDOW_NORMAL )	
cv2.resizeWindow('Imagen Original', 640, 480)
cv2.moveWindow('Imagen Original', 30, 100)

cv2.namedWindow('Imagen Segmentada',  cv2.WINDOW_NORMAL)
cv2.resizeWindow('Imagen Segmentada', 640, 480)	
cv2.moveWindow('Imagen Segmentada', 700, 100)

# Se establece el método de captura de eventos del mouse
cv2.setMouseCallback('Imagen Original',_mouseEvent)
			
while(True):
	# Se lee un frame de la cámara
	ret, frame = cap.read()	

	# Se convierte el frame capturado a formato HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# Se definen los límites de color permitidos para la detección de objetos
	LowerColor1 = color1_hsv + LowerColorError
	UpperColor1 = color1_hsv + UpperColorError	

	# Se aplica una máscara de color para detectar objetos
	Color1Mask = cv2.inRange(hsv, LowerColor1, UpperColor1)
	Color1Res = cv2.bitwise_and(frame,frame, mask= Color1Mask)

	# Se crea un texto con la información de los colores seleccionados
	font = cv2.FONT_HERSHEY_SIMPLEX
	scale = 0.7
	thickness = 2
	color = (0, 0, 255)  # Color en formato BGR (Cyan)
	x_offset = 10
	y_offset = 30
	cv2.putText(frame, info_text, (x_offset, y_offset), font, scale, color, thickness, cv2.LINE_AA)
	cv2.putText(frame, info_text2, (x_offset, y_offset+50), font, scale, color, thickness, cv2.LINE_AA)

	# Se muestra la imagen original y la imagen segmentada en las ventanas correspondientes
	cv2.imshow('Imagen Original',frame)	
	cv2.imshow('Imagen Segmentada',Color1Res)

	# Si se presiona la tecla ESC, se cierran las ventanas y se libera la cámara
	if cv2.waitKey(1) & 0xFF == 27:
		break

# Se libera la cámara cuando se termina el programa
cap.release()
# Se cierran todas las ventanas
cv2.destroyAllWindows()