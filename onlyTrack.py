import cv2
import numpy as np

class NutsTracker:
    def __init__(self):
        self.record = True
        self.cap = None
        self.image = None
        self.distancia = "0"
        self.area = "0"
        self.tracking = True
        self.show = False
        self.mostrar_contorno = True
        self.x = -1
        self.y = -1
        self.x_max = 0
        self.y_max = 0
        self.detect = False
        self.obj = [0, 0]
        self.min_area = 1000
        self.max_area = 10000
        self.camera_num = 0
        self.default_lower = [0, 85, 56]
        self.default_upper = [42,185,255]
    def initiateVideo(self):
        self.cap = cv2.VideoCapture(self.camera_num)
        ret, frame = self.cap.read()
        while (not ret):
            print(ret,frame)
            ret, frame = self.cap.read()
        self.y_max, self.x_max, _ = frame.shape
        self.obj = [int(self.x_max / 2), int(self.y_max)]

    def track(self):
        while self.tracking:
            #try:
                #file_path = os.path.join('vision', 'valores_lower_upper.txt')
                #with open(file_path, 'r') as file:
                 #   lines = file.readlines()
                  #  lower_line = lines[0].strip().split(': ')[1].replace('[', '').replace(']', '')
                   # upper_line = lines[1].strip().split(': ')[1].replace('[', '').replace(']', '')

                    # Convierte los valores de string a numpy arrays
                    #lower = np.array([int(x) for x in lower_line.split(',')])
                   # upper = np.array([int(x) for x in upper_line.split(',')])

            #except FileNotFoundError:
            #    # Si el archivo no se encuentra, utiliza valores predeterminados
            lower = np.array(self.default_lower, np.uint8)
            upper = np.array(self.default_upper, np.uint8)

            ret, frame = self.cap.read()

            if ret:
                frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, lower, upper)
                contornos, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                a = False
                for c in contornos:
                    area = cv2.contourArea(c)
                    if (area >= self.min_area) and (self.max_area >= area):
                        a = True
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        self.x = int(M["m10"] / M["m00"])
                        self.y = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (self.x, self.y), 7, (255, 0, 255), -1)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(frame, '{},{}'.format(
                            self.x, self.y), (self.x+10, self.y), font, 0.75, (255, 0, 255), 1, cv2.LINE_AA)
                        nuevoContorno = cv2.convexHull(c)
                        cv2.circle(frame, (self.x, self.y), max(
                            nuevoContorno[:, 0, 0].tolist()) - self.x, (0, 0, 255), 2)

                        if self.mostrar_contorno:
                            cv2.drawContours(
                                frame, [nuevoContorno], 0, (0, 255, 0), 3)
                        self.distancia = str(self.x - frame.shape[1] * 0.5)
                        # print(f"Distancia con respecto al centro de la imagen: {x - frame.shape[1] * 0.5}")}
                self.detect = a
                if len(contornos) == 0:
                    self.distancia = "0"
                if self.show:
                    cv2.imshow('frame', frame)
                if self.record:
                    out.write(frame)
                    
                if cv2.waitKey(1) & 0xFF == ord('s'):
                    break        
    def stop_tracking(self):
        self.tracking = False

    def finish(self):
        self.cap.release()
        cv2.destroyAllWindows()