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

            # Create a window
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
                    self.distancia = "0"
                if self.show:
                    cv2.imshow('frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('s'):
                    break

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

    def begin(self):
        self.arduino = serial.Serial(self.target_L, 115200, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
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
        self.Kp = 0.0001
        self.Kd = 0.00001
        self.Ki = 0.000000001
        self.tol = 50  # tolerancia a error
        self.min_C = 180
        self.max_C = 255
        self.C_lin = 200

        self.ref = 0
        self.E = 0
        self.E_ = 0
        self.E__ = 0
        self.C = 0
        self.C_ = 0
        self.motor_R = 0
        self.motor_L = 0


        self.Kp_size = 0.5
        self.Kd_size = 0.01
        self.Ki_size = 0.001
        self.tol_size = 50
        self.min_velocity = 0
        self.max_velocity = 150

        self.E_size = 0
        self.E_size_ = 0
        self.E_size__ = 0
        self.velocity = 0


    def update(self, error, size):
        self.E__ = self.E_
        self.E_ = self.E
        self.E = error
        self.C_ = self.C
        self.C = self.C_ + (self.Kp + self.Ts*self.Ki + self.Kd/self.Ts)*self.E + \
            (-self.Kp - 2*self.Kd/self.Ts)*self.E_ + (self.Kd/self.Ts)*self.E__
        if abs(self.C) > self.max_C:
            self.C = np.sign(self.C) * self.max_C
        if abs(self.C) < self.min_C:
            self.C = np.sign(self.C) * self.min_C
        self.C = np.sign(self.C) * int(self.C)

        self.E_size__ = self.E_size_
        self.E_size_ = self.E_size
        self.E_size = size
        self.velocity = (
            self.velocity
            + (self.Kp_size + self.Ts * self.Ki_size + self.Kd_size / self.Ts) * self.E_size
            + (-self.Kp_size - 2 * self.Kd_size / self.Ts) * self.E_size_
            + (self.Kd_size / self.Ts) * self.E_size__
        )
        if abs(self.velocity) > self.max_velocity:
            self.velocity = np.sign(self.velocity) * self.max_velocity
        if abs(self.velocity) < self.min_velocity:
            self.velocity = np.sign(self.velocity) * self.min_velocity

    def make_control(self, distancia, size):
        print("dist", distancia, "size", size)
        angular_error = 0
        size_error = 0
        if abs(int(size)) > self.tol_size:
            size_error = int(size)
        if abs(int(distancia)) > self.tol:
            angular_error = int(distancia)
        self.update(angular_error, size_error)
        if abs(angular_error) > self.tol:
            if angular_error > 0:
                print(self.C)
                self.motor_L = self.C
                self.motor_R = - self.C
            else:
                self.motor_L = - self.C
                self.motor_R = self.C
        if (distancia == "0"):
            # there is not objective
            self.motor_L = 0
            self.motor_R = 0

    def get_control(self):
        right, left = self.get_control_value()
        return str(int(min(abs(right), 250))) + "," + str(int(np.sign(right))) + "," + str(int(min(abs(left), 250))) + "," + str(int(np.sign(left)))

    def get_control_value(self):
        return [self.velocity + self.motor_R, self.velocity + self.motor_L]

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

while running:
    if (coms.manual_mode):
        if keyboard.is_pressed('a'):
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
    else:
        ret, frame = tracker.cap.read()
        if ret:
            distancia = tracker.distancia
            area = tracker.area
            control.make_control(distancia, area)
            control_signal = control.get_control()
            coms.comunicacion(control_signal)
        else:
            print("wtf")
    if keyboard.is_pressed('x'):
        running = False
        print("Stopped")

tracker.stop_tracking()
tracker.finish()
