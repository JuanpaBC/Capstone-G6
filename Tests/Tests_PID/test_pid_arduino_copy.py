import cv2
import numpy as np
import serial
import time
import threading
import matplotlib.pyplot as plt
import csv
last_data = ''
class Communication:
    def __init__(self) -> None:
        self.mostrar_contorno = False
        self.manual_mode = False
        self.starts = False
        self.target_L = '/dev/ttyACM0'  #'COM4' # Change this to your actual target
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

coms = Communication()
coms.begin()

# Start a separate thread to read and print messages from Arduino
read_messages_thread = threading.Thread(target=coms.read_and_print_messages)
read_messages_thread.daemon = True
read_messages_thread.start()

running = True
sendIt = True
duration = 1  # Duration in seconds for each set of messages

start_time = time.time()




# CSV file configuration
csv_file_path = 'serial_data.csv'
csv_header = ['Time', 'RPMA', 'RPMB', 'ARPMref', 'BRPMref','EncoderA','EncoderB']

RPMref_values = [200,0,0,0]
# External variables
RPMA_values = []
RPMB_values = []
EncoderA_values = []  # New list for EncoderA values
EncoderB_values = []  # New list for EncoderB values

v_ref = str(250)
i = 0
try:
    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(csv_header)

        while running:
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time >= duration:
                start_time = current_time  # Reset the start time

                if sendIt:
                    if(i==0):
                        coms.comunicacion(f"{RPMref_values[0]} ,{RPMref_values[1]}")
                    if(i!=3):
                        i= i+1
                # Toggle the flag for the next iteration
                sendIt = not sendIt
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
# plt.plot(RPMA_values, label='RPMA')
# plt.plot(RPMB_values, label='RPMB')
# plt.plot(RPMref_values, label='RPMref')
# plt.xlabel('Time (s)')
# plt.ylabel('RPM')
# plt.legend()
# plt.show()

# plt.plot(EncoderA_values, label='EncoderA')  # Plot EncoderA values
# plt.plot(EncoderB_values, label='EncoderB')  # Plot EncoderB values

# plt.xlabel('Time (s)')
# plt.ylabel('Encoders')
# plt.legend()
# plt.show()