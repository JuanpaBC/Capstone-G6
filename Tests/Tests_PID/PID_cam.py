import math


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
