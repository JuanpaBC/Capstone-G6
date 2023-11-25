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
        self.on_view = True

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
                    self.on_view = False
                    self.distancia = -1
                else:
                    self.on_view = True
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


class Pid:
    def __init__(self):
        self.Ts = 0.2  # time sample
        self.ref_ang = 0
        self.Kp_ang = 0.0001
        self.Kd_ang = 0.00001
        self.Ki_ang = 0.000000001
        self.tol_ang = 50  # tolerancia a error
        
        self.ref_lin = 0
        self.Kp_lin = 0.5
        self.Kd_lin = 0.01
        self.Ki_lin = 0.001
        self.tol_size = 50
        
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
        
        self.min_C = 180
        self.max_C = 255
        self.motor_R = 0
        self.motor_L = 0

    def update_ang(self, error):
        self.E_ang__ = self.E_ang_
        self.E_ang_ = self.E_ang
        self.E_ang = error
        self.C_ang_ = self.C
        self.C_ang = (self.C_ang_
                      + (self.Kp_ang + self.Ts*self.Ki_ang + self.Kd_ang/self.Ts)*self.E_ang
                      + (-self.Kp_ang - 2*self.Kd_ang/self.Ts)*self.E_ang_
                      + (self.Kd_ang/self.Ts)*self.E_ang__)
        
    def update_lin(self, size):
        self.E_ling__ = self.E_lin_
        self.E_ling_ = self.E_lin
        self.E_ling = size
        self.C_lin = (self.C_lin_
                      + (self.Kp_lin+ self.Ts * self.Ki_lin + self.Kd__lin / self.Ts) * self.E_lin
                      + (-self.Kp_lin - 2 * self.Kd_lin/self.Ts) * self.E_lin_
                      + (self.Kd_lin / self.Ts) * self.E_lin__)
        if abs(self.velocity) > self.max_velocity:#CHECK
            self.velocity = np.sign(self.velocity) * self.max_velocity
        if abs(self.velocity) < self.min_velocity:
            self.velocity = np.sign(self.velocity) * self.min_velocity

    def update(self, error, size):
        self.update_ang(error)
        self.update_lin(size)

    def make_control(self, distancia, size):
        if distancia == "0": # there is not objective
            self.motor_L = 0
            self.motor_R = 0
        else
            distancia = int(distancia)
            size = int(size)
            ang_error = 0 if abs(distancia - self.ref_ang) < self.tol_ang else distancia
            lin_error = 0 if abs(size - self.ref_lin) < self.tol_lin else size
            self.update(ang_error, lin_error)
            self.motor_L = self.check_limit(self.C_lin + self.C_ang)
            self.motor_R = self.check_limit(self.C_lin - self.C_ang)
        print("vel_lin", self.C_lin, "  vel_ang", self.C_ang, "  mR", self.motor_R, "  mL", self.motor_L)

    def check_limit(self, vel):
        if vel < self.min_C:
            return self.min_C
        if vel > self.max_C:
            return self.max_C
        return vel

    def get_control(self):
        right, left = [self.motor_R, self.motor_L]
        return str(int(min(abs(right), 250))) + "," + str(int(np.sign(right))) + "," + str(int(min(abs(left), 250))) + "," + str(int(np.sign(left)))


class Controller:

    def __init__(self, tracker, comunication, pid) -> None:
        self.tracker = tracker
        self.tracking_thread = threading.Thread(target=self.track_wrapper, args=(self))
        self.tracking_thread.daemon = True
        self.coms = comunication
        self.pid = pid
        self.x = 0
        self.y = 0
        self.obj_x = 3000
        self.obj_y = 3000
        self.running = True
        self.sendIt = True
        self.going = True
        self.turning = 0
        self.pid_steps = []
        self.directions = {
            'forward': '200,1,200,1\n',
            'backward': '200,-1,200,-1\n',
            'left': '200,1,200,-1\n',
            'right': '200,-1,200,1\n',
            'manual stop': '0,1,0,1\n',
            'pid stop': '-1,1,-1,1\n'
        }
    
    def track_wrapper(self):
        # This function is used to run the track() method in a separate thread.
        self.tracker.track()

    def begin(self):
        self.tracker.colorSetup()
        self.tracker.initiateVideo()
        self.tracking_thread.start()
        self.run()

    def undo_pid(self):
        step_data = self.pid_steps.pop(-1).strip().split(',')
        step_data[1] = step_data[1]*-1
        step_data[3] = step_data[3]*-1
        step_data = ','.join(step_data) + '\n'
        self.coms.comunicacion(step_data)


    def follow_path(self):
        if not self.tracker.on_view and len(self.pid_steps) == 0:
            if self.x >= self.obj_x and self.y >= self.obj_y:
                self.coms.comunicacion(self.directions['manual stop'])
            elif self.turning and self.going:
                self.y += 1
                self.turning +=1
                self.coms.comunicacion(self.directions['right'])
                if self.turning >= 100:
                    self.turning = 0
                    self.going = False
            elif self.turning and not self.going:
                self.y += 1
                self.turning +=1
                self.coms.comunicacion(self.directions['left'])
                if self.turning >= 100: #placeholder, hay que ver cuanto cuenta en realidad al doblar
                    self.turning = 0
                    self.going = True
            elif self.going and self.x < self.obj_x:
                self.x += 1
                self.coms.comunicacion(self.directions['forward'])
            elif self.going and self.x >= self.obj_x:
                self.turning += 1
            elif not self.going and self.x > 0:
                self.x -= 1
                self.coms.comunicacion(self.directions['forward'])
            elif not self.going and self.x <= 0:
                self.turning += 1
        elif len(self.pid_steps) != 0:
            self.undo_pid()
        else:
            ret, frame = self.tracker.cap.read()
            if ret:
                distancia = self.tracker.distancia
                if(distancia == -1):
                    if not (self.sendIt):
                        self.coms.comunicacion(self.directions['pid stop'])
                        self.sendIt = True
                else:    
                    self.sendIt = False
                    area = self.tracker.area
                    print(distancia,area)
                    self.control.make_control(distancia, area)
                    control_signal = self.control.get_control()
                    self.pid_steps.append(control_signal)
                    self.coms.comunicacion(control_signal)
            else:
                print("wtf")
    
    def run(self):
        while self.running:
            if (self.coms.manual_mode):
                command = input()
                if command == 'a':
                    self.coms.comunicacion(self.directions['left'])
                elif command == 'd':
                    # coms.comunicacion('R\n')
                    self.coms.comunicacion(self.directions['right'])
                elif command == 'w':
                    # coms.comunicacion('U\n')
                    self.coms.comunicacion(self.directions['forward'])
                elif command == 's':
                    # coms.comunicacion('D\n')
                    self.coms.comunicacion(self.directions['backward'])
                elif command == 'p':
                    self.coms.comunicacion('S\n')
                elif command == 'q':
                    self.coms.comunicacion(self.directions['manual stop'])
            else:
                self.follow_path()
            if keyboard.is_pressed('x'):
                self.running = False
                print("Stopped")
                self.coms.comunicacion('0,1,0,1\n')
    
    def stop(self):
        self.tracker.stop_tracking()
        self.tracker.finish()

controller = Controller(ColorTracker(), Communication(), Pid())
controller.begin()
