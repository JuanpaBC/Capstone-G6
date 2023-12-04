import math
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
    def __init__(self, model):
        self.cap = None
        self.image = None
        self.distancia = "0"
        self.area = "0"
        self.tracking = True
        self.show = True
        self.x = -1
        self.y = -1
        self.x_max = 0
        self.y_max = 0
        self.model = model
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])

    def initiateVideo(self):
        self.cap = cv2.VideoCapture(0)
        ret, frame = self.cap.read()
        self.y_max, self.x_max, _ = frame.shape

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
                                self.x, self.y, w, h = b_center.cpu().numpy()
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


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, x_target=0.0, y_target=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.x_target = x_target
        self.y_target = y_target
        
    def theta_error(self, x, y):
        # Returns the error in the angle theta
        angle = math.atan2(self.y_target - y, self.x_target - x)*180/math.pi
        if -7.5 < angle < 7.5:
            self.theta_err = 0
        else:
            self.theta_err = angle/abs(angle)

    def lineal_error(self, x, y):
        # Returns the error in the lineal distance
        self.lineal_err = math.sqrt((self.x_target - x)**2 + (self.y_target - y)**2)

    def err_reference(self, x, y):
        # Returns the reference for the PID
        self.theta_error(self,x, y)
        self.lineal_error(self,x, y)
        if self.theta_err == 0:
            self.errA = self.lineal_err
            self.errB = self.lineal_err
        else:
            self.errA = self.theta_err * self.lineal_err
            self.errB = -self.theta_err * self.lineal_err

    def update(self, delta_time, x, y):
        self.err_reference(self,x, y)
        self.integral += self.errA * delta_time
        derivativeA = (self.errA - self.previous_error) / delta_time
        outputA = self.kp * self.errA + self.ki * self.integral + self.kd * derivativeA
        self.previous_error = self.errA

        self.integral += self.errB * delta_time
        derivativeB = (self.errB - self.previous_error) / delta_time
        outputB = self.kp * self.errB + self.ki * self.integral + self.kd * derivativeB
        self.previous_error = self.errB

        if outputA > 0:
            outputA = min(outputA, 255)
        elif outputA < 0:
            outputA = max(outputA, -255)
        if outputB > 0:
            outputB = min(outputB, 255)
        elif outputB < 0:
            outputB = max(outputB, -255)

        return outputA, outputB


class Brain:

    def __init__(self, tracker, coms, pid) -> None:

        self.tracker = tracker
        self.coms = coms
        self.control = PID

        self.tracking_thread = threading.Thread(target=self.track_wrapper, args=())
        self.tracking_thread.daemon = True

        self.read_messages_thread = threading.Thread(target=self.coms.read_and_print_messages)
        self.read_messages_thread.daemon = True

        
        self.turning = False
        self.instructions = {
            "forward" : "1,200,200\n",
            "backward" : "1,-200,-200\n",
            "right" : "1,200,-200\n",
            "left" : "1,-200,200\n",
            "shovel" : "2\n",
            "stop" : "1,0,0\n"
        }
        self.last_time = 0.0
        self.begin()
        self.do()


    def track_wrapper(self):
        # This function is used to run the track() method in a separate thread.
        print("Starting tracking")
        self.tracker.track()
        print("Ending tracking")
    def begin(self):
        self.tracker.initiateVideo()
        self.coms.begin()
        self.tracking_thread.start()
        self.read_messages_thread.start()
        self.pid = PID(0.01, 0.0, 0.0, round(
            self.tracker.x_max/2), round(self.tracker.y_max/2))
        

    def do(self):
        running = True
        sendIt = True

        RPMA_values = []
        RPMB_values = []
        RPMref_values = []


        #csv_file_path = 'serial_data.csv'
        #csv_header = ['Time', 'RPMA', 'RPMB', 'ARPMref', 'BRPMref']
        last_data = ""
        try:
            #with open(csv_file_path, 'w', newline='') as csvfile:
                #csv_writer = csv.writer(csvfile)
                #csv_writer.writerow(csv_header)
            self.last_time = time.time()
            while running:
                if (self.coms.manual_mode):
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
                    # Movimiento automático
                    if self.tracker.distancia != "0":
                        actual_time = time.time()
                        self.control.update(
                            actual_time - self.last_time, self.tracker.x, self.tracker.y)
                        self.last_time = actual_time
                        self.coms.comunicacion(f"{self.pid.outputA},{self.pid.outputB}")
                    # if(len(self.coms.data.split(','))>=3 and self.coms.data != last_data):
                        # Extract RPMA, RPMB, RPMref from the updated 'data'
                        # timestamp, aData, bData = self.coms.data.split(',')
                        # aParts = aData.split('|')
                        # BParts = bData.split('|')
                        # RPMA = float(aParts[0].split(':')[1])
                        # RPMB = float(BParts[0].split(':')[1])
                        # Save data to listsc
                        # RPMA_values.append(RPMA)
                        # RPMB_values.append(RPMB)
                        # Save data to CSV
                        #csv_writer.writerow([timestamp, RPMA, RPMB, ARef,BRef])
                        # last_data = self.coms.data

                        # time.sleep(0.1)
                    # if keyboard.is_pressed('x'):
                    #    running = False
                    #    print("Stopped")
                    #    coms.comunicacion('0,1,0,1\n')

        except KeyboardInterrupt:
            print("Data collection interrupted.")

        finally:
            self.finish()

    def automatic(self):
        return


    def finish(self):
        # Close the serial port
        self.coms.arduino.close()
        self.tracker.stop_tracking()
        self.tracker.finish()


model = YOLO("best.pt")
brain = Brain(NutsTracker(model), Communication(), PID())
