import math

class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, kpt=0.0, kit=0.0, kdt=0.0, x_target=0.0, y_target=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kp_t = kpt
        self.ki_t = kit
        self.kd_t = kdt
        self.previous_error = 0.0
        self.previous_errorA= 0.0
        self.previous_errorB= 0.0
        self.integral_angularA = 0.0
        self.integral_angularB = 0.0
        self.integral_lineal = 0.0
        self.x_target = x_target
        self.y_target = y_target
        self.tolangle = 7.5
        self.tolpixels = 50
        self.errA = 0
        self.errB = 0

    def theta_error(self, x, y):
        # Returns the error in the angle theta
        if(self.y_target - y) == 0:
            if(x - self.x_target == 0):
                angle = -math.pi/2
            else:
                angle = (x - self.x_target)/abs(x - self.x_target)  * math.pi/2
        else:
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
            self.integral_angularA = 0
            self.integral_angularB = 0
            self.errA = self.lineal_err/2
            self.integral_lineal += self.errA * delta_time
            derivativeA = (self.errA - self.previous_error) / delta_time
            outputA = self.kp * self.errA + self.ki * self.integral_lineal + self.kd * derivativeA
            outputB = self.kp * self.errA + self.ki * self.integral_lineal + self.kd * derivativeA #Copiamos A = B
            self.previous_error = self.errA
            # Limitar la salida
            outputA = max(outputA, 73)
            outputB = max(outputB, 73)

        # Error angular
        else:
            self.integral_lineal = 0
            # Si el error es positivo, el motor A gira más rápido que el B
            self.errA = self.theta_err
            self.errB = -self.theta_err
            print(f"error angular: {self.theta_err}")
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
                outputA = 4/3 *outputA
                outputB = outputB
            else:
                outputA = outputA
                outputB = outputB * 4/3
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
            #if outputA > 0:
            #    outputA = min(outputA, 255)
            #    outputA = max(outputA, 150)
            #if outputB > 0:
            #    outputB = min(outputB, 255)
            #    outputB = max(outputB, 150)
            #if outputA < 0:
            #    outputA = max(outputA, -255)
            #if outputB < 0:
            #    outputB = max(outputB, -255)
        return int(round(outputA)), int(round(outputB))



if __name__ == '__main__':
    print("Starting...")
    control = PID(6, 0.03, 0.1, 150, 0.03, 0.5, 2028/2, 1520)

    while True:
        outputA, outputB = control.update(0.1, 1800,100)
        print(control.theta_err)
        print(control.lineal_err)
        print(f"OutputA: {outputA} OutputB: {outputB}")
