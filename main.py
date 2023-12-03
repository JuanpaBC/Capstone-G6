import cv2
import numpy as np
import serial
import time
# import keyboard
import threading
import torch
from torchvision import transforms
from ultralytics import YOLO
import csv

class NutsTracker:
    def __init__(self):
        self.setear_colores = False
        self.mostrar_contorno = False
        self.cap = None
        self.image = None
        self.distancia = "0"
        self.area = "0"
        self.tracking = True
        self.show = True
        self.model = YOLO("best.pt")
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])

    def nothing(self, x):
        pass
    
    def initiateVideo(self):
        self.cap = cv2.VideoCapture(0)

    def track(self):
        while self.tracking:
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # Make predictions
            with torch.no_grad():
                predictions = self.model(frame)
                for result in predictions:
                    result_bboxes = result.boxes
                    #print(result)
                    for result_bbox in result_bboxes:
                        b_coordinates = result_bbox.xyxy[0]
                        too_much_overlap = False
                        b_center = result_bbox.xywh[0]  # get box coordinates in (top, left, bottom, right) format
                        box = b_coordinates[:4].int().tolist()
                        for other in result_bboxes:
                            o_coordinates = other.xyxy[0]
                            if (b_coordinates.cpu().numpy() == o_coordinates.cpu().numpy()).all():
                                continue
                            # Calculate the intersection coordinates
                            x1 = max(b_coordinates[0].cpu().numpy(), o_coordinates[0].cpu().numpy())
                            y1 = max(b_coordinates[1].cpu().numpy(), o_coordinates[1].cpu().numpy())
                            x2 = min(b_coordinates[2].cpu().numpy(), o_coordinates[2].cpu().numpy())
                            y2 = min(b_coordinates[3].cpu().numpy(), o_coordinates[3].cpu().numpy())

                            # Calculate area of intersection
                            intersection_area = max(0, x2 - x1) * max(0, y2 - y1)
                            b_area = b_center[2].cpu().numpy() * b_center[3].cpu().numpy()
                            if intersection_area / b_area > 0.5:
                                too_much_overlap = True
                                break
                        if not too_much_overlap:
                            confidence = result_bbox.conf
                            if confidence[0].cpu().numpy() > 0.5:
                                self.distancia = str(int(b_center[0].cpu().numpy()))
                                self.area = str(int(b_center[2].cpu().numpy() * b_center[3].cpu().numpy()))
                                if self.show:
                                    frame = cv2.circle(frame, (int(b_center[0].cpu().numpy()), int(b_center[1].cpu().numpy())), 5, (0, 0, 255), -1)
                                    frame = cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
                                    label = "castana"
                                    frame = cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                    # Display the resulting frame with predictions
            if self.show:
                cv2.imshow('Object Detection', frame)
                cv2.waitKey(1)
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
        self.target_W = "COM7"
        self.target_L = '/dev/ttyACM0'
        self.baud = 9600
        self.data = ''


    def begin(self):
        self.arduino = serial.Serial(self.target_W, self.baud, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)

    def read_and_print_messages(self):
        while True:
            try:
                if self.arduino.isOpen():
                    message = self.arduino.readline().decode('utf-8').strip()
                    if message:
                        self.data = message
                        #print(f'Recibiendo mensaje: {message}')
                        self.arduino.flush()
            except Exception as e:
                print(f"Error reading message: {e}")

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        #print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)


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
                      + (self.Kp_ang + self.Ts*self.Ki_ang +
                         self.Kd_ang/self.Ts)*self.E_ang
                      + (-self.Kp_ang - 2*self.Kd_ang/self.Ts)*self.E_ang_
                      + (self.Kd_ang/self.Ts)*self.E_ang__)

    def update_lin(self, size):
        self.E_ling__ = self.E_lin_
        self.E_ling_ = self.E_lin
        self.E_ling = size
        self.C_lin_ = self.C_lin
        self.C_lin = (self.C_lin_
                      + (self.Kp_lin + self.Ts * self.Ki_lin +
                         self.Kd_lin / self.Ts) * self.E_lin
                      + (-self.Kp_lin - 2 * self.Kd_lin/self.Ts) * self.E_lin_
                      + (self.Kd_lin / self.Ts) * self.E_lin__)

    def update(self, error, size):
        self.update_ang(error)
        self.update_lin(size)

    def make_control(self, distancia, size):
        if distancia == "0":  # there is not objective
            self.motor_L = 0
            self.motor_R = 0
        else:
            distancia = int(distancia)
            size = int(size)
            ang_error = 0 if abs(
                distancia - self.ref_ang) < self.tol_ang else distancia
            lin_error = 0 if abs(size - self.ref_lin) < self.tol_size else size
            self.update(ang_error, lin_error)
            self.C_lin = 0
            self.motor_L = self.check_limit(self.C_lin + self.C_ang)
            self.motor_R = self.check_limit(self.C_lin - self.C_ang)
        #print("vel_lin", self.C_lin, "  vel_ang", self.C_ang,
              #"  mR", self.motor_R, "  mL", self.motor_L)

    def check_limit(self, vel):
        if abs(vel) < self.min_vel:
            return np.sign(vel) * self.min_vel
        if abs(vel) > self.max_vel:
            return np.sign(vel) * self.max_vel
        return vel

    def get_control(self):
        right, left = [self.motor_R, self.motor_L]
        return "0, " + str(int(right)) +","+ str(int(left)) + "\n"


class Brain:

    def __init__(self, tracker, coms, pid) -> None:
        self.tracker = tracker
        self.coms = coms
        self.pid = pid
        self.tracking_thread = threading.Thread(target=self.track_wrapper, args=(self,))
        self.tracking_thread.daemon = True
        self.read_messages_thread = threading.Thread(target=self.coms.read_and_print_messages)
        self.read_messages_thread.daemon = True
        self.begin()

    def begin(self):
        self.tracker.initiateVideo()
        self.coms.begin()
        self.tracking_thread.start()
        self.read_messages_thread.start()
    
    def track_wrapper(self):
        # This function is used to run the track() method in a separate thread.
        self.tracker.track()

    def do(self):
        running = True
        sendIt = True

        RPMA_values = []
        RPMB_values = []
        RPMref_values = []


        csv_file_path = 'serial_data.csv'
        csv_header = ['Time', 'RPMA', 'RPMB', 'ARPMref', 'BRPMref']
        last_data = ""
        try:
            with open(csv_file_path, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_header)
                while running:
                    if (self.coms.manual_mode):
                        command = input()
                        if command == 'a':
                            self.coms.comunicacion('1,-200,200\n')
                        elif command == 'd':
                            # coms.comunicacion('R\n')
                            self.coms.comunicacion('1,200,-200\n')
                        elif command == 'w':
                            # coms.comunicacion('U\n')
                            self.coms.comunicacion('1,200,200\n')
                        elif command == 's':
                            # coms.comunicacion('D\n')
                            self.coms.comunicacion('1,-200,-200\n')
                        elif command == 'p':
                            self.coms.comunicacion('2\n')
                        elif command == 'q':
                            self.coms.comunicacion('1,0,0\n')
                    else:
                        if self.tracker.distancia != "0":
                            self.control.make_control(self.tracker.distancia, self.tracker.area)
                            self.coms.comunicacion(self.control.get_control())
                        if(len(self.coms.data.split(','))>=3 and self.coms.data != last_data):
                            # Extract RPMA, RPMB, RPMref from the updated 'data'
                            timestamp, aData, bData = self.coms.data.split(',')
                            aParts = aData.split('|')
                            BParts = bData.split('|')
                            ARef = float(aParts[0].split(':')[1])
                            RPMA = float(aParts[1].split(':')[1])
                            BRef = float(BParts[0].split(':')[1])
                            RPMB = float(BParts[1].split(':')[1])
                            # Save data to listsc
                            RPMA_values.append(RPMA)
                            RPMB_values.append(RPMB)
                            RPMref_values.append(ARef)
                            # Save data to CSV
                            csv_writer.writerow([timestamp, RPMA, RPMB, ARef,BRef])
                            last_data = self.coms.data

                            time.sleep(0.1)
                    # if keyboard.is_pressed('x'):
                    #    running = False
                    #    print("Stopped")
                    #    coms.comunicacion('0,1,0,1\n')

        except KeyboardInterrupt:
            print("Data collection interrupted.")

        finally:
            self.finish()

    def finish(self):
        # Close the serial port
        self.coms.arduino.close()
        self.tracker.stop_tracking()
        self.tracker.finish()


brain = Brain(NutsTracker(), Communication(), Pid())
