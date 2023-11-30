import numpy as np 

class PID:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.kd
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0