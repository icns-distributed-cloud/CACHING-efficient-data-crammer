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
# title           : EDCrammer.py
# description     : python efficient data crammer
# author          : Yunkon(Alvin) Kim
# date            : 20190220
# version         : 0.1
# python_version  : 3.6
# notes           : This EDCrammer is an implementation of a efficient data crammer for caching data on edge node
#                   in the Python Programming Language.
# ==============================================================================
"""
In order to test PIDController, test_pid.py from IvPID is used with a few modifications.
"""
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
from paho.mqtt import publish
from scipy.interpolate import make_interp_spline  # Switched to BSpline

import PIDController

SDC_id = "SDC_1"
is_finish = False

# Gains from professor's paper : -0.5, 0.125, -0.125
# Gains from Internet : 1.2, 2, 0.001
k_p = 2
k_i = 0
k_d = 0
L = 100

pid = PIDController.PIDController(k_p, k_i, k_d)

pid.setpoint = (1 << 20) * 0.9
pid.sample_time = 0.1

END = L
feedback = 0
counter = 1

feedback_list = []
time_list = []
setpoint_list = []

lock = threading.Lock()
"""Self-test PID class
    .. note::
    ...
    for i in range(1, END):
        pid.update(feedback)
        output = pid.output
        if pid.SetPoint > 0:
            feedback += (output - (1/i))
        if i>9:
            pid.SetPoint = 1
        time.sleep(0.02)
    ---
"""


# -------------------------------------------------------MQTT--------------------------------------------------------#
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected - Result code: " + str(rc))
        client.subscribe("core/edge/" + SDC_id + "/feedback")
        client.subscribe("core/edge/" + SDC_id + "/done_to_test")

    else:
        print("Bad connection returned code = ", rc)
        print("ERROR: Could not connect to MQTT")


def on_message(client, userdata, msg):
    # print("Cart new message: " + msg.topic + " " + str(msg.payload))
    global feedback
    global counter
    global is_finish
    global feedback_list
    global time_list
    global setpoint_list
    global lock

    lock.acquire()
    message = str(msg.payload.decode("utf-8"))
    print("Arrived topic: %s" % msg.topic)
    print("Arrived message: %s" % message)

    if msg.topic == "core/edge/" + SDC_id + "/feedback":
        # compute feedback
        feedback = float(message)
        print("feedback: %s" % feedback)
        pid.compute_by_cycle(feedback, counter)
        output = pid.output
        compensated_feedback = feedback + output - (1 / counter)  # / counter???
        # feedback += output
        print("Feedback: %s /// Output: %s /// Compensated feedback: %s" % (feedback, output, compensated_feedback))

        feedback_list.append(compensated_feedback)
        setpoint_list.append(pid.setpoint)
        time_list.append(counter)

        counter += 1

        # feedback is lower than 0... problem
        if feedback > 0:
            # split and distribute data
            f = open("10MB.zip", "rb")
            data = f.read(int(compensated_feedback))
            print(len(data))
            publish.single("core/edge/" + SDC_id + "/data", data, hostname="163.180.117.37", port=1883)
        else:
            publish.single("core/edge/" + SDC_id + "/flow_control", "Controlling flow", hostname="163.180.117.37",
                           port=1883)

    elif msg.topic == "core/edge/" + SDC_id + "/done_to_test":
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Test is finished")
        is_finish = True
    else:
        print("Unknown - topic: " + msg.topic + ", message: " + message)
    lock.release()


def on_publish(client, userdata, mid):
    print("mid: " + str(mid))


def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def on_log(client, userdata, level, string):
    print(string)


# The below lines will be used to publish the topics
# publish.single("elevator/starting_floor_number", "3", hostname="163.180.117.195", port=1883)
# publish.single("elevator/destination_floor_number", "2", hostname="163.180.117.195", port=1883)
# ------------------------------------------------------------------------------------------------------------------#


if __name__ == "__main__":

    # MQTT connection
    message_client = mqtt.Client("EDCrammer")
    message_client.on_connect = on_connect
    message_client.on_message = on_message

    # Connect to MQTT broker
    message_client.connect("163.180.117.37", 1883, 60)

    print("MQTT client start")
    message_client.loop_start()

    # wait until done to test
    while not is_finish:
        time.sleep(0.001)

    print(feedback_list)
    print(time_list)
    print(setpoint_list)

    time_sm = np.array(time_list)
    time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)

    # feedback_smooth = spline(time_list, feedback_list, time_smooth)
    # Using make_interp_spline to create BSpline
    helper_x3 = make_interp_spline(time_list, feedback_list)
    feedback_smooth = helper_x3(time_smooth)

    plt.plot(time_smooth, feedback_smooth)
    plt.plot(time_list, setpoint_list)
    plt.xlim((1, counter))
    # plt.ylim((min(feedback_list) - 0.5, max(feedback_list) + 0.5))
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('TEST PID')

    # plt.ylim((1 - 0.5, 1 + 0.5))

    plt.grid(True)
    plt.show()

    message_client.loop_stop()
    # Threads completely executed
