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
        self.target_L = '/dev/ttyACM0'  # '/dev/ttyACM0'  # Change this to your actual target
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
duration = 0.01  # Duration in seconds for each set of messages

start_time = time.time()

# CSV file configuration
csv_file_path = 'serial_data.csv'
csv_header = ['Time', 'KpB', 'RPMB', 'BRPMref']

# External variables
KpB_valuesref = np.linspace(0.0, 0.1, 100).tolist()  # List of 100 values from 0.0 to 0.1
KpB_values = []
RPMB_values = []
RPMref_values = []
v_ref = str(250)
i  = 0
try:
    while i <= len(KpB_valuesref ):
        with open(csv_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header)

            for KpB in KpB_valuesref:
                current_time = time.time()
                start_time = current_time  # Reset the start time
                elapsed_time = current_time - start_time
                while(elapsed_time < duration):
                    current_time = time.time()
                    elapsed_time = current_time - start_time
                    if (len(coms.data.split(',')) >= 2 and coms.data != last_data):
                        # Extract KpB, RPMB, RPMref from the updated 'data'
                        Time, Bparts = coms.data.split(',')
                        refB, RPMB, kpbrec = Bparts.split('|')
                          # Assuming KpB is the third part of bData
                        # Save data to lists
                        RPMB_values.append(RPMB.split(':')[1].strip())
                        RPMref_values.append(refB.split(':')[1].strip())
                        KpB_values.append(kpbrec.split(':')[1].strip())  # Update KpB_values in the same loop
                        # Save data to CSV
                        csv_writer.writerow([Time, kpbrec.split(':')[1].strip(), RPMB.split(':')[1].strip(), refB.split(':')[1].strip()])
                        last_data = coms.data
                        i = i+1
                coms.comunicacion(f"{str(round(KpB, 3))} ,{v_ref}")

except KeyboardInterrupt:
    print("Data collection interrupted.")

finally:
    # Close the serial port
    coms.arduino.close()

# Plot the results
plt.plot(KpB_values, RPMB_values, label='RPMB vs KpB')
plt.xlabel('KpB')
plt.ylabel('RPMB')
plt.legend()
plt.show()