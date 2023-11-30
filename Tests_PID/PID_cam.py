class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def theta_error(x, y, x_target, y_target):
    # Returns the error in the angle theta
    return math.atan2(y_target - y, x_target - x)

def lineal_error(x, y, x_target, y_target):
    # Returns the error in the lineal distance
    return math.sqrt((x_target - x)**2 + (y_target - y)**2)