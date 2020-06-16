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

    def __init__(self, k_p=1.0, k_i=0.0, k_d=0.0, setpoint=0.0, pid_output_max=None, pid_output_min=None):
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

        self._setpoint = setpoint

        self.int_error = 0.0
        self._overshooting_guard = setpoint * 0.1
        self._pid_output_max = pid_output_max
        self._pid_output_min = pid_output_min
        self._is_reached_to_limit = False

        self._term_p = 0.0
        self._term_i = 0.0
        self._term_d = 0.0
        self._last_error = 0.0

        self._control_interval = 0.01

        self._pid_output = 0.0
        self._last_pid_output = 0.0
        self._sign_of_last_output = None

    def initialize(self, setpoint=0.0, pid_output_max=None, pid_output_min=None):
        self.sample_time = 0.025
        self.current_time = time.time()
        self.last_time = self.current_time
        self.delta_time = 0
        self.last_counter = 0

        self._setpoint = setpoint

        self.int_error = 0.0
        self._overshooting_guard = setpoint * 0.01
        self._pid_output_max = pid_output_max
        self._pid_output_min = pid_output_min
        self._is_reached_to_limit = False

        self._term_p = 0.0
        self._term_i = 0.0
        self._term_d = 0.0
        self._last_error = 0.0

        self._control_interval = 0.01

        self._pid_output = 0.0
        self._last_pid_output = 0.0
        self._sign_of_last_output = None

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
            self.term_p = error

            if self.k_i != 0:
                # compute term_i
                # it includes avoiding excess_overshooting_threshold on the setter method
                self.term_i += error * delta_time  # * delta_time or number of iteration(?)

            if self.k_d != 0:
                # compute term_d
                delta_error = error - self.last_error
                self.term_d = delta_error / delta_time  # / delta_time or number of iteration(?)

            self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

    def reno_compute(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """
        self.current_time = time.time()
        self.delta_time = self.current_time - self.last_time  # time or number of iteration

        self.last_pid_output = self.pid_output

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = self.setpoint - feedback_value * self.delta_time
        # error = feedback_value
        self.term_p = error

        if self.k_i != 0:
            # clamping term_i by the signs of current error and previous output, and output limit(over and
            # under saturation)
            if self.sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
                print("Clampping term_i")
                if self.term_i < -self._overshooting_guard:
                    self.term_i = -self._overshooting_guard
                if self.term_i > self._overshooting_guard:
                    self.term_i = self._overshooting_guard
            # compute term_i
            else:
                print("Compute term_i")
                self.term_i += error * self.delta_time  # * delta_time or number of iteration(?)
                # print("error * delta_time: %s" % (error * self.delta_time))

        if self.k_d != 0:
            # compute term_d
            delta_error = error - self.last_error
            self.term_d = delta_error / self.delta_time  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self.term_p, self.term_i, self.term_d))
        self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)
        print("self.pid_output: %s" % self.pid_output)

        # Little change(Stable): abs(self.pid_output - self.last_pid_output) > self.term_i * error
        # Much change(Unstable): abs(self.pid_output - self.last_pid_output) <= self.term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_time: %s" % self.delta_time)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.pid_output - self.last_pid_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # skip at the first round
        if counter != 1:
            # significant change in a system
            if abs(self.pid_output - self.last_pid_output) > self.k_i * error:
                print("SIGNIFICANT CHANGE")
                self.last_error = error
                self.last_time = self.current_time
            # if abs(self.pid_output - self.last_pid_output) > self.setpoint * self.k_d / 100:
            #     self.last_error = error
            #     self.last_counter = counter

            # Not much change change in a system
            else:
                print("NOT MUCH CHANGE")
                # keep last state (last_error, last_time, pid_output)
                self.pid_output = self.last_pid_output
        else:
            self.last_error = error
            self.last_time = self.current_time

        # self.last_error = error
        #
        # # Remember last time and last error for next calculation
        # self.last_time = self.current_time
        # self.last_error = error
        self.sign_of_last_output = np.sign(self.pid_output)

        self.clamp_output_limit()

        return self.pid_output

    def cyclic_compute(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration

        # compute term_p
        error = self.setpoint - feedback_value
        # error = feedback_value
        self.term_p = error

        if self.k_i != 0:
            # compute term_i
            # it includes avoiding excess_overshooting_threshold on the setter method
            self.term_i += error * delta_counter  # * delta_time or number of iteration(?)

        if self.k_d != 0:
            # compute term_d
            delta_error = error - self.last_error
            self.term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)

        # Remember last time and last error for next calculation
        self.last_counter = counter
        self.last_error = error

    def cyclic_compute_basic(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration
        self.last_pid_output = self.pid_output

        # self.current_time = time.time()
        # self.delta_time = self.current_time - self.last_time  # time or number of iteration

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = float(self.setpoint) - feedback_value  # / delta_counter  # * delta_time
        # error = feedback_value
        self.term_p = error

        if self.k_i != 0:
            # clamping term_i by the signs of current error and previous output, and output limit(over and
            # under saturation)
            if self.sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
                print("Clampping term_i")
                if self.term_i < -self._overshooting_guard:
                    self.term_i = -self._overshooting_guard
                if self.term_i > self._overshooting_guard:
                    self.term_i = self._overshooting_guard
            # compute term_i
            else:
                print("Compute term_i")
                self.term_i += error  # / delta_counter * delta_time or number of iteration(?)
                # print("error * delta_time: %s" % (error * self.delta_time))

        if self.k_i != 0:
            # compute term_d
            delta_error = error - self.last_error
            self.term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self.term_p, self.term_i, self.term_d))
        self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)
        print("self.pid_output: %s" % self.pid_output)

        # Little change(Stable): abs(self.pid_output - self.last_pid_output) > self.term_i * error
        # Much change(Unstable): abs(self.pid_output - self.last_pid_output) <= self.term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_counter: %s" % delta_counter)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.pid_output - self.last_pid_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # Remember last time and last error for next calculation
        self.last_counter = counter
        self.last_error = error

        self.sign_of_last_output = np.sign(self.pid_output)

        self.clamp_output_limit()

        return self.pid_output

    def cyclic_compute_with_rate_of_change(self, error, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration

        self.term_p = error

        if self.k_i != 0:
            # clamping term_i by the signs of current error and previous output, and output limit(over and
            # under saturation)
            self.term_i += error  # / delta_counter * delta_time or number of iteration(?)

        if self.k_d != 0:
            # compute term_d
            delta_error = error - self.last_error
            self.term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)
        # print("Kp * term_p: %s * %s (%s)    Ki * term_i: %s * %s (%s)    Kd * term_d: %s * %s (%s)" %
        #       (format(self.k_p, ","), format(self.term_p, ","), format(self.k_p * self.term_p, ","),
        #        format(self.k_i, ","), format(self.term_i, ","), format(self.k_i * self.term_i, ","),
        #        format(self.k_d, ","), format(self.term_d, ","), format(self.k_d * self.term_d, ",")))

        if counter > 1:
            # Significant change in a system
            if abs(self.pid_output - self.last_pid_output) > self.k_i * error:
                print("Unsteady state: pid_output ( %s ) - last_pid_output ( %s ) > k_i * error ( %s )" %
                      (format(self.pid_output, ","), format(self.last_pid_output, ","), format(self.k_i * error, ",")))
                # NEED TO CHANGE CACHING INTERVAL (MAKE IT SHORTER) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                self._control_interval = 0.01

            # Not much change change in a system
            else:
                print("Steady state: pid_output ( %s ) - last_pid_output ( %s ) <= k_i * error ( %s )" %
                      (format(self.pid_output, ","), format(self.last_pid_output, ","), format(self.k_i * error, ",")))
                # NEED TO CHANGE CACHING INTERVAL (MAKE IT LONGER) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                self._control_interval += 0.01

        self.last_error = error
        self.last_counter = counter
        self.last_pid_output = self.pid_output

        # self.sign_of_last_output = np.sign(self.pid_output)
        #
        # self.clamp_output_limit()

        # print("Done to compute")

        return self.pid_output, self._control_interval

    def cyclic_compute_with_rate_of_change2(self, feedback_value, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration
        self.last_pid_output = self.pid_output

        # self.current_time = time.time()
        # self.delta_time = self.current_time - self.last_time  # time or number of iteration

        # self.last_pid_output = self.pid_output

        # if self.delta_time >= self.sample_time:
        # compute term_p
        error = float(self.setpoint) - feedback_value  # / delta_counter  # * delta_time
        # error = feedback_value
        self.term_p = error

        if self.k_i != 0:
            # clamping term_i by the signs of current error and previous output, and output limit(over and
            # under saturation)
            if self.sign_of_last_output == np.sign(error) and self._is_reached_to_limit:
                print("Clampping term_i")
                if self.term_i < -self._overshooting_guard:
                    self.term_i = -self._overshooting_guard
                if self.term_i > self._overshooting_guard:
                    self.term_i = self._overshooting_guard
            # compute term_i
            else:
                print("Compute term_i")
                self.term_i += error  # / delta_counter * delta_time or number of iteration(?)
                # print("error * delta_time: %s" % (error * self.delta_time))

        if self.k_d != 0:
            # compute term_d
            delta_error = error - self.last_error
            self.term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        print("term_p: %s /// term_i: %s /// term_d: %s" % (self.term_p, self.term_i, self.term_d))
        self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)
        print("self.pid_output: %s" % self.pid_output)

        # Little change(Stable): abs(self.pid_output - self.last_pid_output) > self.term_i * error
        # Much change(Unstable): abs(self.pid_output - self.last_pid_output) <= self.term_i * error
        # Remember last counter and last error for next calculation, or no change
        print("delta_counter: %s" % delta_counter)
        # print("abs(u(k) - u(k-1)): %s" % abs(self.pid_output - self.last_pid_output))
        # print("k_i * error: %s " % (self.k_i * error))

        # skip at the first round
        if counter != 1:
            # significant change in a system
            # if abs(self.pid_output - self.last_pid_output) > self.k_i * error:
            #     print("SIGNIFICANT CHANGE")
            #     self._last_error = error
            #     self.last_counter = counter
            if abs(self.pid_output - self.last_pid_output) > self.setpoint * self.k_d / 100:
                print("SIGNIFICANT CHANGE")
                self.last_error = error
                self.last_counter = counter

            # Not much change change in a system
            else:
                print("NOT MUCH CHANGE")
                # keep last state (last_error, last_time, pid_output)
                self.pid_output = self.last_pid_output
        else:
            self.last_error = error
            self.last_counter = counter

        # self.last_error = error
        #
        # # Remember last time and last error for next calculation
        # self.last_counter = self.last_counter
        # self.last_error = error
        self.sign_of_last_output = np.sign(self.pid_output)

        self.clamp_output_limit()

        return self.pid_output

    def cyclic_compute_for_cache_capacity_auto_scaling(self, error, counter):
        """Calculates PID value for given reference feedback
           The formula is shown in PID controller from Wikipedia (https://en.wikipedia.org/wiki/PID_controller)
        """

        delta_counter = counter - self.last_counter  # time or number of iteration

        self.term_p = error

        if self.k_i != 0:
            # clamping term_i by the signs of current error and previous output, and output limit(over and
            # under saturation)
            self.term_i += error  # / delta_counter * delta_time or number of iteration(?)

        if self.k_d != 0:
            # compute term_d
            delta_error = error - self.last_error
            self.term_d = delta_error / delta_counter  # / delta_time or number of iteration(?)

        self.pid_output = self.k_p * self.term_p + (self.k_i * self.term_i) + (self.k_d * self.term_d)
        # print("Kp * term_p: %s * %s (%s)    Ki * term_i: %s * %s (%s)    Kd * term_d: %s * %s (%s)" %
        #       (format(self.k_p, ","), format(self.term_p, ","), format(self.k_p * self.term_p, ","),
        #        format(self.k_i, ","), format(self.term_i, ","), format(self.k_i * self.term_i, ","),
        #        format(self.k_d, ","), format(self.term_d, ","), format(self.k_d * self.term_d, ",")))

        if counter > 1:
            # Significant change in a system
            if abs(self.pid_output - self.last_pid_output) > self.k_i * error:
                print("Unsteady state: pid_output ( %s ) - last_pid_output ( %s ) > k_i * error ( %s )" %
                      (format(self.pid_output, ","), format(self.last_pid_output, ","), format(self.k_i * error, ",")))

                if (self.pid_output_max is not None) and (self.pid_output_min is not None):
                    self.avoid_saturation(error)

            # Not much change change in a system
            else:
                print("Steady state: pid_output ( %s ) - last_pid_output ( %s ) <= k_i * error ( %s )" %
                      (format(self.pid_output, ","), format(self.last_pid_output, ","), format(self.k_i * error, ",")))

        self.last_error = error
        self.last_counter = counter
        self.last_pid_output = self.pid_output

        # self.sign_of_last_output = np.sign(self.pid_output)
        #
        # self.clamp_output_limit()

        # print("Done to compute")
        return self.pid_output, self._control_interval

    def avoid_saturation(self, error):
        is_reached_to_limit = False
        # Clamping
        if self.pid_output > self._pid_output_max:
            self.pid_output = self._pid_output_max
            is_reached_to_limit = True
        elif self.pid_output < self._pid_output_min:
            self.pid_output = self._pid_output_min
            is_reached_to_limit = True

        # Checking trend and saturation
        if self.sign_of_last_output == np.sign(error) and is_reached_to_limit:
            # No update term_i to prevent saturation
            self.term_i -= error

    def clamp_output_limit(self):   # Will be deprecated

        if self.pid_output < self._pid_output_min:
            self.pid_output = self._pid_output_min
        elif self.pid_output > self._pid_output_max:
            self.pid_output = self._pid_output_max

        if self.pid_output == self._pid_output_min or self.pid_output == self._pid_output_max:
            self._is_reached_to_limit = True
        else:
            self._is_reached_to_limit = False

    @property
    def last_error(self):
        return self._last_error

    @last_error.setter
    def last_error(self, last_error):
        self._last_error = last_error

    @property
    def sign_of_last_output(self):
        return self._sign_of_last_output

    @sign_of_last_output.setter
    def sign_of_last_output(self, sign_of_last_output):
        self._sign_of_last_output = sign_of_last_output

    @property
    def pid_output_max(self):
        return self._pid_output_max

    @pid_output_max.setter
    def pid_output_max(self, pid_output_max):
        self._pid_output_max = pid_output_max

    @property
    def pid_output_min(self):
        return self._pid_output_min

    @pid_output_min.setter
    def pid_output_min(self, pid_output_min):
        self._pid_output_min = pid_output_min

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, setpoint):
        self._setpoint = setpoint

    @property
    def overshooting_guard(self):
        return self._overshooting_guard

    @overshooting_guard.setter
    def overshooting_guard(self, overshooting_guard):
        self._overshooting_guard = overshooting_guard

    @property
    def pid_output(self):
        return self._pid_output

    @pid_output.setter
    def pid_output(self, pid_output):
        self._pid_output = pid_output

    @property
    def last_pid_output(self):
        return self._last_pid_output

    @last_pid_output.setter
    def last_pid_output(self, last_pid_output):
        self._last_pid_output = last_pid_output

    @property
    def term_p(self):
        return self._term_p

    @term_p.setter
    def term_p(self, term_p):
        self._term_p = term_p

    @property
    def term_i(self):
        return self._term_i

    @term_i.setter
    def term_i(self, term_i):
        self._term_i = term_i

    @property
    def term_d(self):
        return self._term_d

    @term_d.setter
    def term_d(self, term_d):
        self._term_d = term_d
