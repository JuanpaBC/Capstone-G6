import numpy as np
import serial
import time
import threading
import matplotlib.pyplot as plt
import csv
last_data = ''

class Communication:
    def __init__(self) -> None:
        self.target_W = "COM7"
        self.target_L = '/dev/ttyACM0'
        self.baud = 9600
        self.data = ''

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)

    def read_and_print_messages(self):
        if self.arduino.isOpen():
            message = self.arduino.readline().decode('utf-8').rstrip()
            if message:
                self.data = message
                time.sleep(1)
                print(f'Recibiendo mensaje: {message}')
                # self.arduino.flush()
            else:
                print(message)
                print("No se recibio nada")

    def comunicacion(self, mensaje):
        # Manda la distancia medida y espera respuesta del Arduino.
        # print(f'Enviando mensaje {mensaje}')
        if self.arduino.isOpen():
            # self.arduino.flush()
            self.arduino.write(mensaje.encode('utf-8'))
            time.sleep(0.1)


coms = Communication()
coms.begin()

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
KpB_valuesref = np.linspace(0.0, 0.3, 230).tolist()  # List of 100 values from 0.0 to 0.1
KpB_values = []
RPMB_values = []
RPMref_values = []
v_ref = str(250)
i  = 0
try:
    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(csv_header)

        for KpB in KpB_valuesref:
            start_time = time.time()
            while True:
                current_time = time.time()
                elapsed_time = current_time - start_time
                if elapsed_time >= duration:
                    break
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
                coms.comunicacion(
                    f"{str(round(KpB, 3))},{v_ref},{str(round(KpB, 3))},{v_ref}")

except KeyboardInterrupt:
    print("Data collection interrupted.")

finally:
    coms.arduino.close()

plt.plot(KpB_values, RPMB_values, label='RPMB vs KpB')
plt.xlabel('KpB')
plt.ylabel('RPMB')
plt.legend()
plt.save("zn.jpg")
