import cv2
import numpy as np
import serial
import time
#import keyboard
import threading


class Communication:
    def __init__(self) -> None:
        self.mostrar_contorno = False
        self.manual_mode = False
        self.starts = False
        self.target_W = "COM4"
        self.target_L = '/dev/ttyACM0'
        self.baud = 9600

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)
    
    def read_and_print_messages(self):
        while True:
            if self.arduino.isOpen():
                message = self.arduino.readline().decode('utf-8').strip()
                if message:
                    print(f'Received message from Arduino: {message}')

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)
        if False:  # self.manual_mode:
            # Wait for an acknowledgment response from Arduino
            time.sleep(5)
            response = self.arduino.readline().decode('utf-8').strip()
            print(response)
            if response == "ACK":
                print("Arduino received the message")
            else:
                print("Arduino did not acknowledge the message")

    def switch_mode(self):
        if self.manual_mode:
            self.comunicacion('N')
            print("Changing into auto mode")
        else:
            self.comunicacion('M')
            print("Changing into manual mode")
        self.manual_mode = not self.manual_mode


def track_wrapper(tracker):
    # This function is used to run the track() method in a separate thread.
    tracker.track()


class Pid:
    def __init__(self):
        self.Ts = 0.2  # time sample
        
        self.init_pid_vars()
        self.init_pid_params()
        self.max_vel = 250
        self.min_vel = 100
        self.motor_R = 0
        self.motor_L = 0

    def init_pid_params(self):
        self.ref_ang = 0
        self.Kp_ang = 0.0001
        self.Ki_ang = 0.001
        self.Kd_ang = 0.00001
        self.tol_ang = 50  # tolerancia a error
        self.ref_lin = 0
        self.Kp_lin = 0.5
        self.Ki_lin = 0.01
        self.Kd_lin = 0.001
        self.tol_size = 50

    def init_pid_vars(self):
        self.E_ang = 0
        self.E_ang_ = 0
        self.E_ang__ = 0
        self.C_ang = 0
        self.C_ang_ = 0
        self.E_lin = 0
        self.E_lin_ = 0
        self.E_lin__ = 0
        self.C_lin = 0
        self.C_lin_ = 0

    def update_ang(self, error):
        self.E_ang__ = self.E_ang_
        self.E_ang_ = self.E_ang
        self.E_ang = error
        self.C_ang_ = self.C_ang
        self.C_ang = (self.C_ang_
                      + (self.Kp_ang + self.Ts*self.Ki_ang + self.Kd_ang/self.Ts)*self.E_ang
                      + (-self.Kp_ang - 2*self.Kd_ang/self.Ts)*self.E_ang_
                      + (self.Kd_ang/self.Ts)*self.E_ang__)
        
    def update_lin(self, size):
        self.E_ling__ = self.E_lin_
        self.E_ling_ = self.E_lin
        self.E_ling = size
        self.C_lin_ = self.C_lin
        self.C_lin = (self.C_lin_
                      + (self.Kp_lin+ self.Ts * self.Ki_lin + self.Kd_lin / self.Ts) * self.E_lin
                      + (-self.Kp_lin - 2 * self.Kd_lin/self.Ts) * self.E_lin_
                      + (self.Kd_lin / self.Ts) * self.E_lin__)

    def update(self, error, size):
        self.update_ang(error)
        self.update_lin(size)

    def make_control(self, distancia, size):
        if distancia == "0": # there is not objective
            self.motor_L = 0
            self.motor_R = 0
        else:
            distancia = int(distancia)
            size = int(size)
            ang_error = 0 if abs(distancia - self.ref_ang) < self.tol_ang else distancia
            lin_error = 0 if abs(size - self.ref_lin) < self.tol_size else size
            self.update(ang_error, lin_error)
            self.C_lin = 0
            self.motor_L = self.check_limit(self.C_lin + self.C_ang)
            self.motor_R = self.check_limit(self.C_lin - self.C_ang)
        print("vel_lin", self.C_lin, "  vel_ang", self.C_ang, "  mR", self.motor_R, "  mL", self.motor_L)

    def check_limit(self, vel):
        if abs(vel) < self.min_vel:
            return np.sign(vel) * self.min_vel
        if abs(vel) > self.max_vel:
            return np.sign(vel) * self.max_vel
        return vel

    def get_control(self):
        right, left = [self.motor_R, self.motor_L]
        return str(int(min(abs(right), 250))) + "," + str(int(np.sign(right))) + "," + str(int(min(abs(left), 250))) + "," + str(int(np.sign(left)))


coms = Communication()
coms.begin()

control = Pid()
# Start a separate thread to read and print messages from Arduino
read_messages_thread = threading.Thread(target=coms.read_and_print_messages)
read_messages_thread.daemon = True
read_messages_thread.start()

running = True
sendIt = True
while running:
    if sendIt:
        coms.comunicacion('5,1,5,-1')
        time.sleep(2)
        coms.comunicacion('5,-1,5,1')
        time.sleep(2)
    else:
        # Add any additional logic here if needed
        pass