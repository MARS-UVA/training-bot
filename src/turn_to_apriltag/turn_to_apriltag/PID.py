#Copied from https://xiaoxiae.github.io/Robotics-Simplified-Website/motor-controllers/pid/ and edited

class PID:
    """A class implementing a PID controller."""

    def __init__(self, p_constant, i_constant, d_constant, current_time, error_value):
        """Initialises PID controller object from P, I, D constants, a function
        that returns current time and the feedback function."""
        # p, i, and d constants
        self.p_constant, self.i_constant, self.d_constant = p_constant, i_constant, d_constant

        # saves the functions that return the time and the feedback
        self.current_time = current_time
        self.error_value = error_value

    def reset(self):
        """Resets/creates variables for calculating the PID values."""
        # reset PID values
        self.proportional, self.integral, self.derivative = 0, 0, 0

        # reset previous time and error variables
        self.previous_time, self.previous_error = 0, 0

    def get_value(self):
        """Calculates and returns the PID value."""

        # get current time
        time = self.current_time()

        # time and error differences to the previous get_value call
        delta_time = time - self.previous_time
        delta_error = self.error_value - self.previous_error

        # calculate proportional (just error times the p constant)
        self.proportional = self.p * self.error_value

        # calculate integral (error accumulated over time times the constant)
        self.integral += self.error_value * delta_time * self.i

        # calculate derivative (rate of change of the error)
        # for the rate of change, delta_time can't be 0 (divison by zero...)
        self.derivative = 0
        if delta_time > 0:
            self.derivative = delta_error / delta_time * self.d

        # update previous error and previous time values to the current values
        self.previous_time, self.previous_error = time, self.error_value

        # add P, I and D
        pid = self.proportional + self.integral + self.derivative

        # return pid adjusted to values from -1 to +1
        return 1 if pid > 1 else -1 if pid < -1 else pid

    def set_goal(self, goal):
        """Sets the goal and resets the controller variables."""
        self.goal = goal
        self.reset()
