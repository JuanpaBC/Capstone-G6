import serial
import time

class Communication:
    def __init__(self) -> None:
        self.target_W = "COM7"
        self.target_L = '/dev/ttyACM0'
        self.baud = 9600
        self.data = ''

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)

    def read_and_print_messages(self):
        if self.arduino.isOpen():
            message = self.arduino.readline().decode('utf-8').rstrip()
            if message:
                self.data = message
                time.sleep(1)
                print(f'Recibiendo mensaje: {message}')
                # self.arduino.flush()
            else:
                print(message)
                print("No se recibio nada")

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        # print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            #self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)
