import math
import cv2
import numpy as np
import serial
import time
import threading
import matplotlib.pyplot as plt
from torchvision import transforms
from ultralytics import YOLO
import csv
last_data = ''


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
                    # print(result)
                    for result_bbox in result_bboxes:
                        b_coordinates = result_bbox.xyxy[0]
                        too_much_overlap = False
                        # get box coordinates in (top, left, bottom, right) format
                        b_center = result_bbox.xywh[0]
                        box = b_coordinates[:4].int().tolist()
                        for other in result_bboxes:
                            o_coordinates = other.xyxy[0]
                            if (b_coordinates.cpu().numpy() == o_coordinates.cpu().numpy()).all():
                                continue
                            # Calculate the intersection coordinates
                            x1 = max(b_coordinates[0].cpu().numpy(
                            ), o_coordinates[0].cpu().numpy())
                            y1 = max(b_coordinates[1].cpu().numpy(
                            ), o_coordinates[1].cpu().numpy())
                            x2 = min(b_coordinates[2].cpu().numpy(
                            ), o_coordinates[2].cpu().numpy())
                            y2 = min(b_coordinates[3].cpu().numpy(
                            ), o_coordinates[3].cpu().numpy())

                            # Calculate area of intersection
                            intersection_area = max(
                                0, x2 - x1) * max(0, y2 - y1)
                            b_area = b_center[2].cpu().numpy(
                            ) * b_center[3].cpu().numpy()
                            if intersection_area / b_area > 0.5:
                                too_much_overlap = True
                                break
                        if not too_much_overlap:
                            confidence = result_bbox.conf
                            if confidence[0].cpu().numpy() > 0.5:
                                self.distancia = str(
                                    int(b_center[0].cpu().numpy()))
                                self.area = str(
                                    int(b_center[2].cpu().numpy() * b_center[3].cpu().numpy()))
                                if self.show:
                                    frame = cv2.circle(frame, (int(b_center[0].cpu().numpy()), int(
                                        b_center[1].cpu().numpy())), 5, (0, 0, 255), -1)
                                    frame = cv2.rectangle(
                                        frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
                                    label = "castana"
                                    frame = cv2.putText(
                                        frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
        self.target_L = 'COM7'  #'COM4' # Change this to your actual target
        self.baud = 9600
        self.data = ''

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud, timeout=1)
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
                        print(f'Recibiendo mensaje: {message}')
                        self.arduino.flush()
            except Exception as e:
                print(f"Error reading message: {e}")

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)

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


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def theta_error(self, x, y, x_target, y_target):
        # Returns the error in the angle theta
        angle = math.atan2(y_target - y, x_target - x)*180/math.pi
        if -7.5 < angle < 7.5:
            self.theta_err = 0
        else:
            self.theta_err = angle/abs(angle)

    def lineal_error(self, x, y, x_target, y_target):
        # Returns the error in the lineal distance
        self.lineal_err = math.sqrt((x_target - x)**2 + (y_target - y)**2)

    def err_reference(self, x, y, x_target, y_target):
        # Returns the reference for the PID
        self.theta_error(x, y, x_target, y_target)
        self.lineal_error(x, y, x_target, y_target)
        if self.theta_err == 0:
            self.errA = self.lineal_err
            self.errB = self.lineal_err
        else:
            self.errA = self.theta_err * self.lineal_err
            self.errB = -self.theta_err * self.lineal_err

    def update(self, delta_time, x, y, x_target, y_target):
        self.err_reference(x, y, x_target, y_target)
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


coms = Communication()
coms.begin()
tracker = NutsTracker()

tracker.tracking_thread = threading.Thread(target=tracker.track_wrapper, args=(tracker,))
tracker.tracking_thread.daemon = True

tracker.read_messages_thread = threading.Thread(target=tracker.coms.read_and_print_messages)
tracker.read_messages_thread.daemon = True
tracker.initiateVideo()

# Start a separate thread to read and print messages from Arduino
read_messages_thread = threading.Thread(target=coms.read_and_print_messages)
read_messages_thread.daemon = True
read_messages_thread.start()

running = True
sendIt = True
duration = 2  # Duration in seconds for each set of messages


# CSV file configuration
#csv_file_path = 'serial_data.csv'
#csv_header = ['Time', 'RPMA', 'RPMB', 'ARPMref', 'BRPMref','EncoderA','EncoderB']

#RPMref_values = [0, 50,100,150,200]
# External variables
RPMA_values = []
RPMB_values = []
EncoderA_values = []  # New list for EncoderA values
EncoderB_values = []  # New list for EncoderB values
sendIt = True
v_ref = str(250)
i = 0

try:
    #with open(csv_file_path, 'w', newline='') as csvfile:
     #   csv_writer = csv.writer(csvfile)
     #   csv_writer.writerow(csv_header)

        start_time = time.time()
        while running:
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time >= duration:
                start_time = current_time  # Reset the start time
                if(i == 0):
                    coms.comunicacion(f"{1},150,-150")
                if(i == 1):
                    coms.comunicacion(f"{1},0,0")
                if(i!=4):
                    i= i+1
                
                
            if(len(coms.data.split(','))>=3 and coms.data != last_data):
                # Extract RPMA, RPMB, RPMref, EncoderA, EncoderB from the updated 'data'
                timestamp, aData, bData = coms.data.split(',')
                aParts = aData.split('|')
                BParts = bData.split('|')
                ARef = float(aParts[0].split(':')[1])
                EncoderA= float(aParts[1].split(':')[1])
                RPMA = float(aParts[2].split(':')[1])  # Assuming EncoderA is the third part of aData
                BRef = float(BParts[0].split(':')[1])
                EncoderB = float(BParts[1].split(':')[1])
                RPMB = float(BParts[2].split(':')[1])  # Assuming EncoderB is the third part of bData
                # Save data to lists
                RPMA_values.append(RPMA)
                RPMB_values.append(RPMB)
                EncoderA_values.append(EncoderA)  # Save EncoderA value
                EncoderB_values.append(EncoderB)  # Save EncoderB value
                RPMref_values.append(ARef)
                # Save data to CSV
                csv_writer.writerow([timestamp, RPMA, RPMB, ARef, BRef, EncoderA, EncoderB])
                last_data = coms.data
                time.sleep(0.1)  # Adjust sleep time based on your application

except KeyboardInterrupt:
    print("Data collection interrupted.")

finally:
    # Close the serial port
    coms.arduino.close()

# Plot the results
plt.plot(RPMA_values, label='RPMA')
plt.plot(RPMB_values, label='RPMB')
plt.plot(RPMref_values, label='RPMref')
plt.xlabel('Time (s)')
plt.ylabel('RPM')
plt.legend()
plt.show()

plt.plot(EncoderA_values, label='EncoderA')  # Plot EncoderA values
plt.plot(EncoderB_values, label='EncoderB')  # Plot EncoderB values

plt.xlabel('Time (s)')
plt.ylabel('Encoders')
plt.legend()
plt.show()