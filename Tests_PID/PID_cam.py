
def theta_error(x, y, x_target, y_target):
    # Returns the error in the angle theta
    angle = math.atan2(y_target - y, x_target - x)*180/math.pi
    if-7.5 < angle < 7.5:
        return 0
    else:
        return angle/abs(angle)
    

def lineal_error(x, y, x_target, y_target):
    # Returns the error in the lineal distance
    return math.sqrt((x_target - x)**2 + (y_target - y)**2)

def err_reference(x, y, x_target, y_target):
    # Returns the reference for the PID
    e_t = theta_error(x, y, x_target, y_target)
    e_l = lineal_error(x, y, x_target, y_target)
    if e_t == 0:
        return e_l, e_l
    else:
        return e_t*e_l, -e_t*e_l


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, delta_time, x, y, x_target, y_target, err_reference=err_reference):
        errA, errB = err_reference(x, y, x_target, y_target)
        self.integralA += errA * delta_time
        derivativeA = (errA - self.previous_errA) / delta_time
        outputA = self.kp * errA + self.ki * self.integralA + self.kd * derivativeA
        self.previous_errA = errA
        
    
        self.integralA += errB * delta_time
        derivativeB = (errB - self.previous_errB) / delta_time
        outputB = self.kp * errB + self.ki * self.integralB + self.kd * derivativeB
        self.previous_errB = errB
        
        return outputA, outputB
