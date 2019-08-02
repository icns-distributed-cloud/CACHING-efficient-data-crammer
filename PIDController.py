# This file is part of Qualified Caching-as-a-Service.
# BSD 3-Clause License
#
# Copyright (c) 2019, Intelligent-distributed Cloud and Security Laboratory (ICNS Lab.)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
import numpy as np
import logging


class PIDController:

    def __init__(self, k_p=0.2, k_i=0.0, k_d=0.0, setpoint=0.0, remaining_ratio_upto_the_max=0.1):
        self._mylogger = logging.getLogger("PIDController")
        self._mylogger.setLevel(logging.DEBUG)
        # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        # self._mylogger.setFormatter(formatter)

        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.k_p = k_p
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.k_i = k_i
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.k_d = k_d

        self.sample_time = 0.025
        self.current_time = time.time()
        self.last_time = self.current_time
        self.delta_time = 0
        self.last_counter = 0

        self.setpoint = setpoint

        self.int_error = 0.0
        self._overshooting_guard = setpoint * 0.1
        self._output_max = (5 << 20)
        self._output_min = 0
        self._is_reached_to_limit = False

        self._term_p = 0.0
        self._term_i = 0.0
        self._term_d = 0.0
        self._last_error = 0.0

        self.output = 0.0
        self._last_output = 0.0
        self._sign_of_last_output = None
        # will be used for alpha
        self.alpha = remaining_ratio_upto_the_max * remaining_ratio_upto_the_max

    def initialize(self, setpoint=0.0, remaining_ratio_upto_the_max=0.1):
        self.sample_time = 0.025
        self.current_time = time.time()
        self.last_time = self.current_time
        self.delta_time = 0
        self.last_counter = 0

        self.setpoint = setpoint

        self.int_error = 0.0
        self._overshooting_guard = setpoint * 0.01
        self._output_max = (5 << 20)
        self._output_min = 0
        self._is_reached_to_limit = False

        self._term_p = 0.0
        self._term_i = 0.0
        self._term_d = 0.0
        self._last_error = 0.0

        self.output = 0.0
        self._last_output = 0.0
        self._sign_of_last_output = None
        # will be used for alpha
        self.alpha = remaining_ratio_upto_the_max * remaining_ratio_upto_the_max

    def compute(self, feedback_value):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time  # time or number of iteration

        if delta_time >= self.sample_time:
            # compute term_p
            error = self.setpoint - feedback_value
            # error = feedback_value
            self._term_p = error

            # compute term_i
            # it includes avoiding excess_overshooting_threshold on the setter method
            self._term_i += error * delta_time  # * delta_time or number of iteration(?)

            # compute term_d
            delta_error = error - self._last_error
            self._term_d = delta_error / delta_time  # / delta_time or number of iteration(?)

            self.output = self.k_p * self._term_p + (self.k_i * self._term_i) + (self.k_d * self._term_d)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self._last_error = error

    def reno_compute(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """
        self.current_time = time.time()
        self.delta_time = self.current_time - self.last_time  # time or number of iteration

        self._last_output = self.output

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = self.setpoint - feedback_value * self.delta_time
        # error = feedback_value
        self._term_p = error

        # clamping term_i by the signs of current error and previous output, and output limit(over and under saturation)
        if self._sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
            print("Clampping term_i")
            if self._term_i < -self._overshooting_guard:
                self._term_i = -self._overshooting_guard
            if self._term_i > self._overshooting_guard:
                self._term_i = self._overshooting_guard
        # compute term_i
        else:
            print("Compute term_i")
            self._term_i += error * self.delta_time  # * delta_time or number of iteration(?)
            # print("error * delta_time: %s" % (error * self.delta_time))

        # compute term_d
        delta_error = error - self._last_error
        self._term_d = delta_error / self.delta_time  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self._term_p, self._term_i, self._term_d))
        self.output = self.k_p * self._term_p + (self.k_i * self._term_i) + (self.k_d * self._term_d)
        print("self.output: %s" % self.output)

        # Little change(Stable): abs(self.output - self.last_output) > self.__term_i * error
        # Much change(Unstable): abs(self.output - self.last_output) <= self.__term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_time: %s" % self.delta_time)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.output - self._last_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # skip at the first round
        if counter != 1:
            # significant change in a system
            if abs(self.output - self._last_output) > self.k_i * error:
                print("SIGNIFICANT CHANGE")
                self._last_error = error
                self.last_time = self.current_time
            # if abs(self.output - self.last_output) > self.setpoint * self.k_d / 100:
            #     self.last_error = error
            #     self.last_counter = counter

            # Not much change change in a system
            else:
                print("NOT MUCH CHANGE")
                # keep last state (last_error, last_time, output)
                self.output = self._last_output
        else:
            self._last_error = error
            self.last_time = self.current_time

        # self.last_error = error
        #
        # # Remember last time and last error for next calculation
        # self.last_time = self.current_time
        # self.last_error = error
        self._sign_of_last_output = np.sign(self.output)

        self.clamp_output_limit()

        return self.output

    def cyclic_compute(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration

        # compute term_p
        error = self.setpoint - feedback_value
        # error = feedback_value
        self._term_p = error

        # compute term_i
        # it includes avoiding excess_overshooting_threshold on the setter method
        self._term_i += error * delta_counter  # * delta_time or number of iteration(?)

        # compute term_d
        delta_error = error - self._last_error
        self._term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        self.output = self.k_p * self._term_p + (self.k_i * self._term_i) + (self.k_d * self._term_d)

        # Remember last time and last error for next calculation
        self.last_counter = counter
        self._last_error = error

    def cyclic_compute_basic(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration
        self._last_output = self.output

        # self.current_time = time.time()
        # self.delta_time = self.current_time - self.last_time  # time or number of iteration

        self._last_output = self.output

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = float(self.setpoint) - feedback_value  # / delta_counter  # * delta_time
        # error = feedback_value
        self._term_p = error

        # clamping term_i by the signs of current error and previous output, and output limit(over and under saturation)
        if self._sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
            print("Clampping term_i")
            if self._term_i < -self._overshooting_guard:
                self._term_i = -self._overshooting_guard
            if self._term_i > self._overshooting_guard:
                self._term_i = self._overshooting_guard
        # compute term_i
        else:
            print("Compute term_i")
            self._term_i += error  # / delta_counter * delta_time or number of iteration(?)
            # print("error * delta_time: %s" % (error * self.delta_time))

        # compute term_d
        delta_error = error - self._last_error
        self._term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self._term_p, self._term_i, self._term_d))
        self.output = self.k_p * self._term_p + (self.k_i * self._term_i) + (self.k_d * self._term_d)
        print("self.output: %s" % self.output)

        # Little change(Stable): abs(self.output - self.last_output) > self.__term_i * error
        # Much change(Unstable): abs(self.output - self.last_output) <= self.__term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_counter: %s" % delta_counter)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.output - self._last_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # Remember last time and last error for next calculation
        self.last_counter = counter
        self._last_error = error

        self._sign_of_last_output = np.sign(self.output)

        self.clamp_output_limit()

        return self.output

    def cyclic_compute_with_rate_of_change(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration
        self._last_output = self.output

        # self.current_time = time.time()
        # self.delta_time = self.current_time - self.last_time  # time or number of iteration

        self._last_output = self.output

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = float(self.setpoint) - feedback_value  # / delta_counter  # * delta_time
        # error = feedback_value
        self._term_p = error

        # clamping term_i by the signs of current error and previous output, and output limit(over and under saturation)
        if self._sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
            print("Clampping term_i")
            if self._term_i < -self._overshooting_guard:
                self._term_i = -self._overshooting_guard
            if self._term_i > self._overshooting_guard:
                self._term_i = self._overshooting_guard
        # compute term_i
        else:
            print("Compute term_i")
            self._term_i += error  # / delta_counter * delta_time or number of iteration(?)
            # print("error * delta_time: %s" % (error * self.delta_time))

        # compute term_d
        delta_error = error - self._last_error
        self._term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self._term_p, self._term_i, self._term_d))
        self.output = self.k_p * self._term_p + (self.k_i * self._term_i) + (self.k_d * self._term_d)
        print("self.output: %s" % self.output)

        # Little change(Stable): abs(self.output - self.last_output) > self.__term_i * error
        # Much change(Unstable): abs(self.output - self.last_output) <= self.__term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_counter: %s" % delta_counter)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.output - self._last_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # skip at the first round
        if counter != 1:
            # significant change in a system
            if abs(self.output - self._last_output) > self.k_i * error:
                print("SIGNIFICANT CHANGE")
                self._last_error = error
                self.last_counter = counter
            # if abs(self.output - self._last_output) > self.setpoint * self.k_d / 100:
            #     print("SIGNIFICANT CHANGE")
            #     self._last_error = error
            #     self.last_counter = counter

            # Not much change change in a system
            else:
                print("NOT MUCH CHANGE")
                # keep last state (last_error, last_time, output)
                self.output = self._last_output
        else:
            self._last_error = error
            self.last_counter = counter

        # self.last_error = error
        #
        # # Remember last time and last error for next calculation
        # self.last_counter = self.last_counter
        # self.last_error = error
        self._sign_of_last_output = np.sign(self.output)

        self.clamp_output_limit()

        return self.output

    def cyclic_compute_with_rate_of_change2(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration
        self._last_output = self.output

        # self.current_time = time.time()
        # self.delta_time = self.current_time - self.last_time  # time or number of iteration

        self._last_output = self.output

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = float(self.setpoint) - feedback_value  # / delta_counter  # * delta_time
        # error = feedback_value
        self._term_p = error

        # clamping term_i by the signs of current error and previous output, and output limit(over and under saturation)
        if self._sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
            print("Clampping term_i")
            if self._term_i < -self._overshooting_guard:
                self._term_i = -self._overshooting_guard
            if self._term_i > self._overshooting_guard:
                self._term_i = self._overshooting_guard
        # compute term_i
        else:
            print("Compute term_i")
            self._term_i += error  # / delta_counter * delta_time or number of iteration(?)
            # print("error * delta_time: %s" % (error * self.delta_time))

        # compute term_d
        delta_error = error - self._last_error
        self._term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self._term_p, self._term_i, self._term_d))
        self.output = self.k_p * self._term_p + (self.k_i * self._term_i) + (self.k_d * self._term_d)
        print("self.output: %s" % self.output)

        # Little change(Stable): abs(self.output - self.last_output) > self.__term_i * error
        # Much change(Unstable): abs(self.output - self.last_output) <= self.__term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_counter: %s" % delta_counter)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.output - self._last_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # skip at the first round
        if counter != 1:
            # significant change in a system
            # if abs(self.output - self._last_output) > self.k_i * error:
            #     print("SIGNIFICANT CHANGE")
            #     self._last_error = error
            #     self.last_counter = counter
            if abs(self.output - self._last_output) > self.setpoint * self.k_d / 100:
                print("SIGNIFICANT CHANGE")
                self._last_error = error
                self.last_counter = counter

            # Not much change change in a system
            else:
                print("NOT MUCH CHANGE")
                # keep last state (last_error, last_time, output)
                self.output = self._last_output
        else:
            self._last_error = error
            self.last_counter = counter

        # self.last_error = error
        #
        # # Remember last time and last error for next calculation
        # self.last_counter = self.last_counter
        # self.last_error = error
        self._sign_of_last_output = np.sign(self.output)

        self.clamp_output_limit()

        return self.output

    def clamp_output_limit(self):

        if self.output < self._output_min:
            self.output = self._output_min
        elif self.output > self._output_max:
            self.output = self._output_max

        if self.output == self._output_min or self.output == self._output_max:
            self._is_reached_to_limit = True
        else:
            self._is_reached_to_limit = False
