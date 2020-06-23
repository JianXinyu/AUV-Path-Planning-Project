from math import pi


class PID(object):
    def __init__(self, kp, ki, kd, minout, maxout, sampleTime=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sampleTime = sampleTime
        self.lastInput = 0
        self.lastSetp = 0
        self.integral = 0
        self.max = maxout
        self.min = minout

    def compute(self, input, setpoint):
        error = setpoint - input

        # error = error + 2*pi if error<-pi else error
        # error = error - 2*pi if error>pi else error

        dinput = (input - self.lastInput) / self.sampleTime
        dsetp = (setpoint - self.lastSetp) / self.sampleTime

        self.integral += self.ki * error * self.sampleTime

        self.integral = max(min(self.max, self.integral), self.min)

        output = self.kp * error + self.integral + self.kd * (dsetp - dinput)
        output = max(min(output, self.max), self.min)

        self.lastInput = input
        self.lastSetp = setpoint
        return output

    def clear(self):
        self.integral = 0


class PID_angle(PID):
    def compute(self, input, setpoint):
        error = setpoint - input

        error = error + 2*pi if error < -pi else error
        error = error - 2*pi if error > pi else error

        dinput = (input - self.lastInput) / self.sampleTime
        dsetp = (setpoint - self.lastSetp) / self.sampleTime

        self.integral += self.ki * error * self.sampleTime

        self.integral = max(min(self.max, self.integral), self.min)

        output = self.kp * error + self.integral + self.kd * (dsetp - dinput)
        output = max(min(output, self.max), self.min)

        self.lastInput = input
        self.lastSetp = setpoint
        return output
