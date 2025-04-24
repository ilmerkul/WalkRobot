import time


class PIDController:
    def __init__(
        self, Kp: float = 5.0, Ki: float = 3.0, Kd: float = 3.0, current_time=None
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.sample_time = 0
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.SetPoint = 0.0

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_error = 0.0

        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value: float, current_time=None):
        self.current_time = time.time() if current_time is None else current_time
        error = self.SetPoint - feedback_value

        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time > self.sample_time:

            self.p_term = self.Kp * error
            self.i_term += error * delta_time

            if self.i_term < -self.windup_guard:
                self.i_term = -self.windup_guard
            elif self.i_term > self.windup_guard:
                self.i_term = self.windup_guard

            self.d_term = 0.0
            if delta_time > 0:
                self.d_term = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = (
                self.p_term + (self.Ki * self.i_term) + (self.Kd * self.d_term)
            )

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
