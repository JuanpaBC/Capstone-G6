import serial
import time
import threading


def task1():
    while True:
        print("Task 1 is running")
        time.sleep(5)  # Simulate time-consuming task


def task2():
    while True:
        print("Task 2 is running")
        time.sleep(5)  # Simulate time-consuming task


class Communication:
    def __init__(self) -> None:
        self.target_W = "COM7"
        self.target_L = '/dev/ttyACM0'
        self.baud = 9600
        self.data = ''
        self.messages = True

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)

    def read_and_print_messages(self):
        if self.messages:
            try:
                if self.arduino.isOpen():
                    message = self.arduino.readline().decode('utf-8').rstrip()
                    if message:
                        self.data = message
                        time.sleep(1)
                        print(f'Recibiendo mensaje: {message}')
                        # self.arduino.flush()
            except Exception as e:
                print(f"Error reading message: {e}")

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        # print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            #self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)

    def stop_messages(self):
        self.messages = False

if __name__ == '__main__':
    #ser = serial.Serial('COM7', 9600, timeout=1)
    #ser.reset_input_buffer()
    i = 0

    thread1 = threading.Thread(target=task1)
    thread2 = threading.Thread(target=task2)

    thread1.start()
    thread2.start()
    
    com = Communication()
    com.begin()
    while True:
        msg = f"Hello from Raspberry Pi! {i}\n"
        print("Mandando mensaje\n")
        com.comunicacion(msg)
        print("Mensaje enviado\n")
        com.read_and_print_messages()
        print("Mensaje recibido\n")
        #ser.write(msg.encode('utf-8'))
        #line = ser.readline().decode('utf-8').rstrip()
        #print(line)
        #time.sleep(1)
        i += 1
