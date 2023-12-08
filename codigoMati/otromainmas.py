import numpy as np
import serial
import time

class Arduino:
    def __init__(self) -> None:
        self.mostrar_contorno = False
        self.manual_mode = False
        self.starts = False
        self.target_W = "COM7"
        self.target_L = '/dev/ttyACM0'
        self.baud = 9600
        self.data = ''

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))

    def comunicacion(self, a, b):
        # Manda la distancia medida y espera respuesta del Arduino.
        #print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen:
            a = self.arduino.write([a, b])
            print(a)

coms = Arduino()
coms.begin()
i = 20
last_time = time.time()
while True:
    current_time = time.time()
    if(current_time - last_time > 0.5):
        print(i)
        i += 1
        last_time = time.time()
        coms.comunicacion(i,i)