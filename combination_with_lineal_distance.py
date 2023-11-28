import cv2
import numpy as np
import serial
import time
import keyboard
import threading


class ColorTracker:
    def __init__(self):
        self.setear_colores = False
        self.mostrar_contorno = False
        self.cap = None
        self.image = None
        self.distancia = "0"
        self.area = "0"
        self.tracking = True
        self.show = True

    def nothing(self, x):
        pass

    def colorSetup(self):
        if self.setear_colores:
            # Load image
            self.image = cv2.imread('hsv_color_map.png')

            # Create a windows
            cv2.namedWindow('image')

            # Create trackbars for color change
            # Hue is from 0-179 for OpenCV
            cv2.createTrackbar('HMin', 'image', 0, 179, self.nothing)
            cv2.createTrackbar('SMin', 'image', 0, 255, self.nothing)
            cv2.createTrackbar('VMin', 'image', 0, 255, self.nothing)
            cv2.createTrackbar('HMax', 'image', 0, 179, self.nothing)
            cv2.createTrackbar('SMax', 'image', 0, 255, self.nothing)
            cv2.createTrackbar('VMax', 'image', 0, 255, self.nothing)

            # Set default value for Max HSV trackbars
            cv2.setTrackbarPos('HMax', 'image', 96)
            cv2.setTrackbarPos('SMax', 'image', 255)
            cv2.setTrackbarPos('VMax', 'image', 255)
            cv2.setTrackbarPos('HMin', 'image', 49)
            cv2.setTrackbarPos('SMin', 'image', 45)
            cv2.setTrackbarPos('VMin', 'image', 0)

            # Initialize HSV min/max values h49 s85
            self.hMin = self.sMin = self.vMin = self.hMax = self.sMax = self.vMax = 0
            self.phMin = self.psMin = self.pvMin = self.phMax = self.psMax = self.pvMax = 0

    def initiateVideo(self):
        self.cap = cv2.VideoCapture(0)

    def track(self):
        while self.tracking:
            if self.setear_colores:
                # Get current positions of all trackbars
                self.hMin = cv2.getTrackbarPos('HMin', 'image')
                self.sMin = cv2.getTrackbarPos('SMin', 'image')
                self.vMin = cv2.getTrackbarPos('VMin', 'image')
                self.hMax = cv2.getTrackbarPos('HMax', 'image')
                self.sMax = cv2.getTrackbarPos('SMax', 'image')
                self.vMax = cv2.getTrackbarPos('VMax', 'image')

                # Set minimum and maximum HSV values to display
                lower = np.array([self.hMin, self.sMin, self.vMin])
                upper = np.array([self.hMax, self.sMax, self.vMax])

                # Convert to HSV format and color threshold
                hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(self.image, self.image, mask=mask)

                if self.show:
                    cv2.imshow('image', result)
            else:
                lower = np.array([49, 45, 0], np.uint8)
                upper = np.array([96, 255, 255], np.uint8)

            ret, frame = self.cap.read()

            if ret:
                frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, lower, upper)
                contornos, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                for c in contornos:
                    area = cv2.contourArea(c)
                    if area > 300:
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        x = int(M["m10"] / M["m00"])
                        y = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (x, y), 7, (255, 0, 255), -1)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(frame, '{},{}'.format(
                            x, y), (x+10, y), font, 0.75, (255, 0, 255), 1, cv2.LINE_AA)
                        nuevoContorno = cv2.convexHull(c)
                        cv2.circle(frame, (x, y), max(
                            nuevoContorno[:, 0, 0].tolist()) - x, (0, 0, 255), 2)

                        if self.mostrar_contorno:
                            cv2.drawContours(
                                frame, [nuevoContorno], 0, (0, 255, 0), 3)
                        self.distancia = str(int(x - frame.shape[1] * 0.5))
                        self.area = area
                        # print(f"Distancia con respecto al centro de la imagen: {x - frame.shape[1] * 0.5}")}
                if len(contornos) == 0:
                    self.distancia = -1
                if self.show:
                    cv2.imshow('frame', frame)
                #if cv2.waitKey(1) & 0xFF == ord('s'):
                #    break

    def stop_tracking(self):
        self.tracking = False

    def finish(self):
        self.cap.release()
        cv2.destroyAllWindows()


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


tracker = ColorTracker()
tracker.colorSetup()
tracker.initiateVideo()

coms = Communication()
coms.begin()

control = Pid()


tracking_thread = threading.Thread(target=track_wrapper, args=(tracker,))
tracking_thread.daemon = True
tracking_thread.start()

running = True
sendIt = True
while running:
    if (coms.manual_mode):
        command = input()
        if command == 'a':
            coms.comunicacion('200,1,200,-1\n')
        elif command == 'd':
            # coms.comunicacion('R\n')
            coms.comunicacion('200,-1,200,1\n')
        elif command == 'w':
            # coms.comunicacion('U\n')
            coms.comunicacion('200,1,200,1\n')
        elif command == 's':
            # coms.comunicacion('D\n')
            coms.comunicacion('200,-1,200,-1\n')
        elif command == 'p':
            coms.comunicacion('S\n')
        elif command == 'q':
            coms.comunicacion('0,1,0,1\n')
        """if keyboard.is_pressed('a'):
            # coms.comunicacion('L\n')
            coms.comunicacion('200,1,200,-1\n')
        elif keyboard.is_pressed('d'):
            # coms.comunicacion('R\n')
            coms.comunicacion('200,-1,200,1\n')
        elif keyboard.is_pressed('w'):
            # coms.comunicacion('U\n')
            coms.comunicacion('200,1,200,1\n')
        elif keyboard.is_pressed('s'):
            # coms.comunicacion('D\n')
            coms.comunicacion('200,-1,200,-1\n')
        elif keyboard.is_pressed('p'):
            coms.comunicacion('S\n')
        elif keyboard.is_pressed('q'):
            coms.comunicacion('0,1,0,1\n')
        elif keyboard.is_pressed('m'):
            coms.switch_mode()
            tracker.show = True
            """
    else:
        ret, frame = tracker.cap.read()
        if ret:
            distancia = tracker.distancia
            if(distancia == -1):
                if not (sendIt):
                    coms.comunicacion('-1,1,-1,1\n')
                    sendIt = True
            else:    
                sendIt = False
                area = tracker.area
                print(distancia,area)
                control.make_control(distancia, area)
                control_signal = control.get_control()
                coms.comunicacion(control_signal)
        else:
            print("wtf")
    if keyboard.is_pressed('x'):
        running = False
        print("Stopped")
        coms.comunicacion('0,1,0,1\n')

tracker.stop_tracking()
tracker.finish()
