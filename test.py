import math
import numpy as np
import serial
import time
# import keyboard
import threading
import csv
from math import ceil


def distribute_tuples(tuples, Num):
    lista = []
    n = Num//len(tuples)

    for i in range(len(tuples)):

        lista.append(n*tuples[i])

    with open(f'output.txt', 'a') as file:
        for i in range(len(lista)):
            for j in range(len(lista[i])):
                file.write(f"{lista[i][j][0]},{lista[i][j][1]}\n")


class Communication:
    def __init__(self) -> None:
        self.mostrar_contorno = False
        self.manual_mode = False
        self.starts = False
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
        while self.messages:
            try:
                if self.arduino.isOpen():
                    message = self.arduino.readline().decode('utf-8').strip()
                    if message:
                        self.data = message

                        # print(f'Recibiendo mensaje: {message}')
                        # self.arduino.flush()
            except Exception as e:
                print(f"Error reading message: {e}")

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        # print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)

    def stop_messages(self):
        self.messages = False


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, kpt=0.0, kit=0.0, kdt=0.0, x_target=0.0, y_target=0.0):
        self.clas = "PID"
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kp_t = kpt
        self.ki_t = kit
        self.kd_t = kdt
        self.previous_error = 0.0
        self.previous_errorA = 0.0
        self.previous_errorB = 0.0
        self.integral_angularA = 0.0
        self.integral_angularB = 0.0
        self.integral_lineal = 0.0
        self.x_target = x_target
        self.y_target = y_target
        self.tolangle = 7.5
        self.tolpixels = 70
        self.errA = 0
        self.errB = 0

    def theta_error(self, x, y):
        # Returns the error in the angle theta
        angle = math.atan((x - self.x_target)/(self.y_target - y))
        self.theta_err = angle

    def lineal_error(self, x, y):
        # Returns the error in the lineal distance
        self.lineal_err = ((self.x_target - x) + (self.y_target - y)
                           )**2/(self.x_target * self.y_target)

    def update(self, delta_time, x, y):
        self.theta_error(x, y)
        self.lineal_error(x, y)

        # PID para lineal y angular separados con distintos kp, ki y kd

        # Error lineal
        if self.theta_err == 0 or (abs(x-self.x_target) < self.tolpixels):
            self.integral_angularA = 0
            self.integral_angularB = 0
            self.errA = self.lineal_err/2
            self.integral_lineal += self.errA * delta_time
            derivativeA = (self.errA - self.previous_error) / delta_time
            outputA = self.kp * self.errA + self.ki * \
                self.integral_lineal + self.kd * derivativeA
            outputB = self.kp * self.errA + self.ki * \
                self.integral_lineal + self.kd * derivativeA  # Copiamos A = B
            self.previous_error = self.errA
            # Limitar la salida
            outputA = max(outputA, 73)
            outputB = max(outputB, 73)

        # Error angular
        else:
            self.integral_lineal = 0
            # Si el error es positivo, el motor A gira más rápido que el B
            self.errA = self.theta_err
            self.errB = -self.theta_err
            print(f"error angular: {self.theta_err}")
            # PID para el error angular A
            self.integral_angularA += self.errA * delta_time
            derivativeA = (self.errA - self.previous_errorA) / delta_time
            outputA = self.kp_t * self.errA + self.ki_t * \
                self.integral_angularA + self.kd_t * derivativeA
            self.previous_errorA = self.errA

            # PID para el error angular B
            self.integral_angularB += self.errB * delta_time
            derivativeB = (self.errB - self.previous_errorB) / delta_time
            outputB = self.kp_t * self.errB + self.ki_t * \
                self.integral_angularB + self.kd_t * derivativeB
            self.previous_errorB = self.errB

            if self.theta_err > 0:
                outputA = 1.5 * outputA
                outputB = outputB * 0.9
            else:
                outputA = outputA * 0.8
                outputB = outputB * 1
            # Limitar la salida
            # if outputA > 0:
            #    outputA = min(outputA, 255)
            #    outputA = max(outputA, 190)
            # elif outputA < 0:
            #    outputA = max(outputA, -255)
            #    outputA = min(outputA, -120)
            # if outputB > 0:
            #    outputB = min(outputB, 255)
            #    outputB = max(outputB, 190)
            # elif outputB < 0:
            #    outputB = max(outputB, -255)
            #    outputB = min(outputB, -120)
            if outputA > 0:
                outputA = min(outputA, 255)
                outputA = max(outputA, 150)
            if outputB > 0:
                outputB = min(outputB, 255)
                outputB = max(outputB, 150)
            if outputA < 0:
                outputA = max(outputA, -255)
            if outputB < 0:
                outputB = max(outputB, -255)
        return outputA, outputB


class tracker:
    def __init__(self, filename):
        self.file = open(filename, 'r')
        self.reader = csv.reader(self.file)
        self.x_max = 300
        self.y_max = 300
        self.x = 0
        self.y = 0

    def track(self):
        try:
            next_line = next(self.reader)
            self.x, self.y = map(int, next_line)
            
        except StopIteration:
            return None

    def close(self):
        self.file.close()


class Brain:

    def __init__(self, tracker, coms) -> None:

        self.kp = 6
        self.ki = 0.03
        self.kd = 0.1
        self.kp_t = 295
        self.ki_t = 5
        self.kd_t = 5

        self.tracker = tracker
        self.coms = coms

        self.tracking_thread = threading.Thread(
            target=self.track_wrapper, args=())
        self.tracking_thread.daemon = True

        self.read_messages_thread = threading.Thread(
            target=self.coms.read_and_print_messages)
        self.read_messages_thread.daemon = True

        self.going_back = False
        self.history = []

        self.distance = 1
        self.startTurnAround = 0
        self.instructions = {
            "forward": "1,200,200\n",
            "backward": "1,-200,-200\n",
            "turnAround": "1,250,-250\n",
            "right": "1,200,-200\n",
            "left": "1,-200,200\n",
            "shovel": "2\n",
            "stop": "1,0,0\n",
            "slow": "1,73,73\n"
        }
        self.scoop_in_progress = False
        self.scooping = 0
        self.last_time = 0.0
        self.begin()
        self.do()

    def track_wrapper(self):
        # This function is used to run the track() method in a separate thread.
        self.tracker.track()

    def begin(self):
        print("Starting...")

        self.coms.begin()
        print("Coms Started...")
        self.tracking_thread.start()
        print("Track thread Started...")
        self.read_messages_thread.start()
        print("read message thread Started...")
        self.control = PID(self.kp, self.ki, self.kd, self.kp_t, self.ki_t, self.kd_t, round(
            self.tracker.x_max/2), round(self.tracker.y_max))
        # self.control = PID(0.35, 0.001, 0.008, round(
        #    self.tracker.x_max/2), round(self.tracker.y_max))

    def do(self):
        running = True
        # RPMA_values = []
        # RPMB_values = []
        # RPMref_values = []
        last_data = ''

        try:

            self.last_time = time.time()
            start_time = time.time()
            while running:
                if (self.coms.manual_mode):
                    print("Manual")
                    command = input()
                    if command == 'a':
                        self.coms.comunicacion(self.instructions["left"])
                    elif command == 'd':
                        # coms.comunicacion('R\n')
                        self.coms.comunicacion(self.instructions["right"])
                    elif command == 'w':
                        # coms.comunicacion('U\n')
                        self.coms.comunicacion(self.instructions["forward"])
                    elif command == 's':
                        # coms.comunicacion('D\n')
                        self.coms.comunicacion(self.instructions["backward"])
                    elif command == 'p':
                        self.coms.comunicacion(self.instructions["shovel"])
                    elif command == 'q':
                        self.coms.comunicacion(self.instructions["stop"])
                else:

                    if (((time.time() - start_time) > 10)):
                        self.automatic()
                        # if(i>5):
                        #     self.coms.comunicacion(self.instructions["stop"])
                if (len(self.coms.data.split(',')) >= 4 and self.coms.data != last_data):
                    # Extract RPMA, RPMB, RPMref from the updated 'data'
                    timestamp, aData, bData, pala = self.coms.data.split(',')
                    self.scooping = int(pala.split(':')[1])

                    last_data = self.coms.data

        except KeyboardInterrupt:
            print("Data collection interrupted.")

        finally:
            self.finish()

    def automatic(self):
        print("automatic")

        if (self.scoop_in_progress):
            self.scoop_in_progress = False
            self.going_back = True
        else:
            if True:
                self.going_back = True
                actual_time = time.time()
                dt = actual_time - self.last_time
                outputA, outputB = self.control.update(
                    dt, self.tracker.x, self.tracker.y)
                self.history.append([-1*outputA, -1*outputB])
                self.last_time = actual_time
                print(f"OutputA: {outputA}, OutputB: {outputB}")
                self.coms.comunicacion(f"1,{outputA},{outputB}")
            else:
                self.control.integral = 0
                print(self.instructions["stop"])
                self.coms.comunicacion(self.instructions["stop"])

    def finish(self):
        # Close the serial port
        # Release the video writer after the main loop
       
        self.coms.arduino.close()
        self.coms.stop_messages()
        self.read_messages_thread.join()
        
        self.tracking_thread.join()


Num = 300
lista = [[(Num//2, Num//2)], [(Num//4, Num//4)], [(3*(Num//4), 3*(Num//4))]]
# distribute_tuples(lista, Num)
tracker = tracker('output.txt')

brain = Brain(tracker, Communication())
