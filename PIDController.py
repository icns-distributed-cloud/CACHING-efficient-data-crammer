# This file is part of Qualified Caching-as-a-Service.
# Copyright 2019 Intelligent-distributed Cloud and Security Laboratory (ICNS Lab.)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial
# portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# title           : PIDController.py
# description     : python pid controller
# author          : Yunkon(Alvin) Kim
# date            : 20190129
# version         : 0.1
# python_version  : 3.6
# notes           : This PID controller is an implementation of a Proportional-Integral-Derivative (PID) Controller
#                   in the Python Programming Language.
# ==============================================================================
import time


class PIDController:

    def __init__(self, k_p=0.2, k_i=0.0, k_d=0.0):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.k_p = k_p
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.k_i = k_i
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.k_d = k_d

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.setpoint = 0.0

        self.int_error = 0.0
        self.excess_overshooting_threshold = 20.0

        self.__term_p = 0.0
        self.__term_i = 0.0
        self.__term_d = 0.0
        self.last_error = 0.0

        self.output = 0.0

    def compute(self, feedback_value):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time  # time or number of iteration

        if delta_time >= self.sample_time:
            # compute term_p
            error = self.setpoint - feedback_value
            self.__term_p = error

            # compute term_i
            # it includes avoiding excess_overshooting_threshold on the setter method
            self.__term_i += error * delta_time  # * delta_time or number of iteration(?)

            # compute term_d
            delta_error = error - self.last_error
            self.__term_d = delta_error / delta_time  # / delta_time or number of iteration(?)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.__term_p + (self.k_i * self.__term_i) + (self.k_d * self.__term_d)

    @property
    def term_i(self):
        return self.__term_i

    @term_i.setter
    def term_i(self, val):
        """One common problem resulting from the ideal PID implementations is integral windup.
        Following a large change in setpoint the integral term can accumulate an error larger than the maximal value for
        the regulation variable (windup), thus the system overshoots and continues to increase until this accumulated
        error is unwound.
        """
        if val < -self.excess_overshooting_threshold:
            self.__term_i = -self.excess_overshooting_threshold
        elif val > self.excess_overshooting_threshold:
            self.__term_i = self.excess_overshooting_threshold
