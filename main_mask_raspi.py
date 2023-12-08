import math
import cv2
import numpy as np
import serial
import time
# import keyboard
import threading
from picamera2 import Picamera2, Preview
import multiprocessing
centerX = multiprocessing.Value('i', 0)
centerY = multiprocessing.Value('i', 0)
RPMAG =  multiprocessing.Value('i', 0)
RPMBG =  multiprocessing.Value('i', 0)
detect = multiprocessing.Value('i', 0)
class NutsTracker:
    def __init__(self, resolution=(320, 240), framerate=30):
        self.record = True
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.resolution = resolution
        self.framerate = framerate
        self.frame = None
        self.stopped = False
        self.tracking = True
        self.show = False
        self.mostrar_contorno = True
        self.x = -1
        self.y = -1
        self.x_max = 0
        self.y_max = 0
        self.detect = False
        self.obj = [0, 0]
        self.min_area = 5
        self.max_area = 20000000
        self.default_lower = np.array([3, 162, 53])
        self.default_upper = np.array([47, 255, 255])

    
    def initiateVideo(self):
        print("Initiating video.")
        try:
            self.camera = Picamera2()  # Change this line
            self.camera.resolution = self.resolution
            self.camera.framerate = self.framerate

            #self.camera.start_preview(Preview.QTGL)

            preview_config = self.camera.create_preview_configuration()
            self.camera.configure(preview_config)
            
            #self.camera.start_preview(Preview.QT)
            self.camera.start()
            if(self.record):

                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # Adjust fps and frame size
            # self.rawCapture = Picamera2.capture_array("raw")
            # self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
            # Other camera setup code...
            # start the thread to read frames from the video stream
            self.x_max = self.camera.resolution[0]
            self.y_max = self.camera.resolution[1]
            time.sleep(2)
            threading.Thread(target=self.track, args=()).start()
            return self
        except Exception as e:
            print(f"Error initializing camera: {e}")
        
    def track(self):
        # keep looping infinitely until the thread is stopped
        while not self.stopped:
            try:
                # Capture frame from the camera
                self.frame = self.camera.capture_array()
                frameHSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, self.default_lower, self.default_upper)
                contornos, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                a = 0
                for c in contornos:
                    area = cv2.contourArea(c)
                    if (area >= self.min_area) and (self.max_area >= area):
                        a = 1
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        self.x = int(M["m10"] / M["m00"])
                        self.y = int(M["m01"] / M["m00"])
                        centerX.value = self.x
                        centerY.value = self.y
                        cv2.circle(self.frame, (self.x, self.y), 7, (255, 0, 255), -1)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(self.frame, '{},{}'.format(
                            self.x, self.y), (self.x+10, self.y), font, 0.75, (255, 0, 255), 1, cv2.LINE_AA)
                        nuevoContorno = cv2.convexHull(c)
                        cv2.circle(self.frame, (self.x, self.y), max(
                            nuevoContorno[:, 0, 0].tolist()) - self.x, (0, 0, 255), 2)

                        if self.mostrar_contorno:
                            cv2.drawContours(
                                self.frame, [nuevoContorno], 0, (0, 255, 0), 3)
                        # print(f"Distancia con respecto al centro de la imagen: {x - frame.shape[1] * 0.5}")}
                detect.value = a
                if len(contornos) == 0:
                    self.distancia = "0"
                if self.show:
                    cv2.imshow('frame', self.frame)
                if self.record:
                    self.out.write(self.frame)
                    
                if cv2.waitKey(1) & 0xFF == ord('s'):
                    break 
            except Exception as e:
                print(f"Error processing frame: {e}")   
        if self.tracking:
            self.stop_tracking()
        self.finish()
        print("Tracking stopped.")    
    def stop_tracking(self):
        self.tracking = False

    def finish(self):
        print("Windows released.")
        self.stopped = True
        self.camera.close()
        if(self.record):
            self.out.release()

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
        self.messages = True

    def begin(self):
        self.arduino = serial.Serial(self.target_L, self.baud, timeout=1)
        time.sleep(0.1)
        if self.arduino.isOpen():
            print("{} conectado!".format(self.arduino.port))
            time.sleep(1)
            self.read_messages_thread = threading.Thread(target=self.recibir, args=())
            self.read_messages_thread.start()
            self.send_messages_thread = threading.Thread(target=self.enviar, args=())
            self.send_messages_thread.start()

            self.send_messages_thread.join()
            self.read_messages_thread.join()
        else:
            print("Error al conectar el puerto {}".format(self.arduino.port))
    def recibir(self):
        # print('working')
        while True:
            message = self.arduino.readline().decode('utf-8').strip()
            print(message)
    def enviar(self, RPMA, RPMB):
        while True:
            RPMASend = RPMA.value
            RPMBSend = RPMB.value
            # mensaje = msg_gen([X, Y], [xR, yR])
            self.arduino.write(bytes([RPMASend, RPMBSend]))
            self.arduino.write(b'E')
    
    def stop_messages(self):
        self.comunicacion(bytes([0, 0]))
        self.messages = False


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, kpt=0.0, kit=0.0, kdt=0.0, x_target=0.0, y_target=0.0):
        self.clas = "PID"
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kp_t = kpt
        self.ki_t = kit
        self.kd_t = kdt
        self.previous_error = 0.0
        self.previous_errorA= 0.0
        self.previous_errorB= 0.0
        self.integral_lineal= 0.0
        self.integral_angularA = 0.0
        self.integral_angularB = 0.0
        self.x_target = x_target
        self.y_target = y_target
        self.tolangle = 7.5
        self.tolpixels = 70
        self.errA = 0
        self.errB = 0
        
    def theta_error(self, x, y):
        # Returns the error in the angle theta
        angle = math.atan((x - self.x_target)/(self.y_target - y))
        self.theta_err = angle

    def lineal_error(self, x, y):
        # Returns the error in the lineal distance
        self.lineal_err = ((self.x_target - x) + (self.y_target - y))**2/(self.x_target * self.y_target)
        
   
    def update(self, delta_time, x, y):
        self.theta_error(x, y)
        self.lineal_error(x, y)
        
        #PID para lineal y angular separados con distintos kp, ki y kd
        
        # Error lineal
        if self.theta_err == 0 or (abs(x-self.x_target) < self.tolpixels):
            
            self.errA = self.lineal_err/2
            self.integral_lineal += self.errA * delta_time
            derivativeA = (self.errA - self.previous_error) / delta_time
            outputA = self.kp * self.errA + self.ki * self.integral_lineal + self.kd * derivativeA
            outputB = self.kp * self.errA + self.ki * self.integral_lineal + self.kd * derivativeA #Copiamos A = B
            self.previous_error = self.errA
            # Limitar la salida
            outputA = max(outputA, 65)
            outputB = max(outputB, 65)

        # Error angular
        else:
            self.integral_lineal = 0
            # Si el error es positivo, el motor A gira más rápido que el B
            self.errA = self.theta_err
            self.errB = -self.theta_err
        
            # PID para el error angular A
            self.integral_angularA += self.errA * delta_time
            derivativeA = (self.errA - self.previous_errorA) / delta_time
            outputA = self.kp_t * self.errA + self.ki_t * self.integral_angularA + self.kd_t * derivativeA
            self.previous_errorA = self.errA

            # PID para el error angular B
            self.integral_angularB += self.errB * delta_time
            derivativeB = (self.errB - self.previous_errorB) / delta_time
            outputB = self.kp_t * self.errB + self.ki_t * self.integral_angularB+ self.kd_t * derivativeB
            self.previous_errorB = self.errB

            if self.theta_err > 0:
                outputA = 1.5 *outputA
                outputB = outputB *0.9
            else:
                outputA = outputA * 0.8
                outputB = outputB * 1.5
            # Limitar la salida
            #if outputA > 0:
            #    outputA = min(outputA, 255)
            #    outputA = max(outputA, 190)
            #elif outputA < 0:
            #    outputA = max(outputA, -255)
            #    outputA = min(outputA, -120)
            #if outputB > 0:
            #    outputB = min(outputB, 255)
            #    outputB = max(outputB, 190)
            #elif outputB < 0:
            #    outputB = max(outputB, -255)
            #    outputB = min(outputB, -120)
            if outputA > 0:
                outputA = min(outputA, 230)
            if outputB > 0:
                outputB = min(outputB, 230)
            if outputA < 0:
                outputA = max(outputA, -230)
            if outputB < 0:
                outputB = max(outputB, -230)
        return outputA, outputB


class Brain:

    def __init__(self, tracker, coms) -> None:

        self.kp = 6
        self.ki = 0.03
        self.kd = 0.15
        self.kp_t = 300
        self.ki_t = 5
        self.kd_t = 5

        self.tracker = tracker
        self.coms = coms

        self.going_back = False
        self.history = []

        self.distance = 1
        self.turning = False
        self.state = 0
        self.startTurnAround = 0
        self.instructions = {
            "forward" : "1,200,200\n",
            "backward" : "1,-200,-200\n",
            "turnAround" : "1,250,-250\n",
            "right" : "1,200,-200\n",
            "left" : "1,-200,200\n",
            "shovel" : "2\n",
            "stop" : "1,0,0\n",
            "slow" : "1,73,73\n"
        }
        self.scoop_in_progress = False
        self.scooping = 0
        self.last_time = 0.0
        self.begin()

    
    def check_timeout(self):
        # Check if the timeout duration has passed
        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.timeout_duration:
            print("Timeout reached. Stopping the tracking and finishing.")
            self.finish()

    def begin(self):
        print("Initiating coms.")
        self.coms.begin()
        print("Coms instantiated.")

        print("Initiating tracker.")
        self.tracker.initiateVideo()
        print("tracker instantiated.")


        print("Initiating Control.")
        self.control = PID(self.kp, self.ki, self.kd, self.kp_t, self.ki_t, self.kd_t,round(
            self.tracker.x_max/2), round(self.tracker.y_max))
        print("Control instantiated.")

        #t1 = multiprocessing.Process(target = self.tracker.track, args=())
        t2= multiprocessing.Process(target = self.coms.read_and_print_messages, args=())
        t3 = multiprocessing.Process(target = self.do, args=())

        #print("Initiating tracker.")
        #t1.start()
        print("Initiating read msg.")
        t2.start()
        print("Initiating Control.")
        t3.start()

        #print("Waiting for threads to finish.")
        #t1.join()
        print("tracker finish.")
        t2.join()
        print("read messages finish.")
        t3.join()
        print("control finish.")
        #self.control = PID(0.35, 0.001, 0.008, round(
        #    self.tracker.x_max/2), round(self.tracker.y_max))

    def do(self):
        running = True
        #RPMA_values = []
        #RPMB_values = []
        #RPMref_values = []
        last_data = ''

        #csv_file_path = 'serial_data.csv'
        #csv_header = ['Time', 'RPMA', 'RPMB', 'ARPMref', 'BRPMref']
        #last_data = ""
        try:
            #with open(csv_file_path, 'w', newline='') as csvfile:
                #csv_writer = csv.writer(csvfile)
                #csv_writer.writerow(csv_header)
            self.last_time = time.time()
            start_time = time.time()
            while time.time() - start_time < 80:
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
                    
                    if(((time.time() - start_time) > 10)):
                        self.automatic()
                        # if(i>5):
                        #     self.coms.comunicacion(self.instructions["stop"])
                if(len(self.coms.data.split(','))==4 and self.coms.data != last_data):
                        # Extract RPMA, RPMB, RPMref from the updated 'data'
                    splitData = self.coms.data.split(',')
                    timestamp = splitData[0]
                    aData = splitData[1]
                    bData = splitData[2]
                    pala = splitData[3]
                    self.scooping = int(pala.split(':')[1])
                        # aParts = aData.split('|')
                        # BParts = bData.split('|')
                        # RPMA = float(aParts[0].split(':')[1])
                        # RPMB = float(BParts[0].split(':')[1])
                        # Save data to listsc
                        # RPMA_values.append(RPMA)
                        # RPMB_values.append(RPMB)
                        # Save data to CSV
                        #csv_writer.writerow([timestamp, RPMA, RPMB, ARef,BRef])
                    last_data = self.coms.data

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
        # si scoopea, detente
        if detect.value == 1:
            #self.going_back = True
            actual_time = time.time()
            dt = actual_time - self.last_time
            outputA, outputB = self.control.update(dt, centerX.value, centerY.value)
            self.history.append([-1*outputA, -1*outputB])
            self.last_time = actual_time
            print(f"OutputA: {outputA}, OutputB: {outputB}")
            RPMAG.value = int(outputA)
            RPMBG.value = int(outputB)
        else:
            self.control.integral = 0
            if(self.state == 0):
                print("OutputA: 68, OutputB: 68")
                self.coms.comunicacion(self.instructions["slow"])
            elif(self.state == 1):
                if(self.startTurnAround == 0):
                    self.startTurnAround = time.time()
                if((time.time() - self.startTurnAround) < 7):
                    print("OutputA: 220, OutputB: -150")
                    RPMAG.value = 0
                    RPMBG.value = 0
                else:
                    self.startTurnAround = 0
                    self.state = 3
            elif(self.state == 2):
                print("OutputA: 68, OutputB: 68")
                
                RPMAG.value = 68
                RPMBG.value = 68
                self.coms.comunicacion(self.instructions["slow"])
            elif(self.state == 3):
                print("OutputA: 0, OutputB: 0.000001")
                
                RPMAG.value = 0
                RPMBG.value = 0


    def finish(self):
        # Close the serial port
        # Release the video writer after the main loop
        print("Finishing program.")
        self.coms.arduino.close()
        print("Arduino closed.")
        self.coms.stop_messages()
        self.read_messages_thread.join()
        self.tracker.stop_tracking()
        self.tracker.finish()
        print("Program finished.")

brain = Brain(NutsTracker(), Communication())
