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
import csv
import os
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
from paho.mqtt import publish

import PIDController

SDC_id = "SDC_1"
is_finish = False

MQTT_HOST = "163.180.117.236"
MQTT_PORT = 1883

# Gains from professor's paper : -0.5, 0.125, -0.125
# Gains from Internet : 1.2, 2, 0.001
# My previous gains: 0.8, 3, 0.004
# from CMU 3, 0.8, 0.7
k_p = 1.0
k_i = 0.0  # 0.45
k_d = 0.0  # 1.5
# k_p = 0.7
# k_i = 0.5  # 0.45
# k_d = 0.7  # 1.5
L = 100

CACHE_MAX_SIZE = (5 << 20)  # - (1 << 19)  # 1 << 19 == 512KB
SAMPLE_TIME = 0.025
MASS = 10

pid = PIDController.PIDController(k_p=k_p, k_i=k_i, k_d=k_d, setpoint=CACHE_MAX_SIZE * 0.9,
                                  remaining_ratio_upto_the_max=0.1)

END = L
feedback = 0
counter = 1

feedback_list = []
percentage_feedback_list = []
output_list = []
percentage_output_list = []
time_list = []
setpoint_list = []
percentage_setpoint_list = []

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

test_start_time = 0
test_end_time = 0
scenario_counter = 1


# -------------------------------------------------------MQTT--------------------------------------------------------#
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected - Result code: " + str(rc))
        client.subscribe("core/edge/" + SDC_id + "/feedback")
        client.subscribe("core/edge/" + SDC_id + "/init_for_testing")
        client.subscribe("core/edge/" + SDC_id + "/done_to_test")
        client.subscribe("core/edge/" + SDC_id + "/all_test_complete")


    else:
        print("Bad connection returned code = ", rc)
        print("ERROR: Could not connect to MQTT")


def on_message(client, userdata, msg):
    # print("Cart new message: " + msg.topic + " " + str(msg.payload))
    global feedback
    global counter
    global is_finish
    global feedback_list
    global percentage_feedback_list
    global output_list
    global percentage_output_list
    global time_list
    global setpoint_list
    global percentage_setpoint_list
    global lock
    global test_start_time
    global test_end_time
    global scenario_counter

    # compensated_feedback = 0.0

    lock.acquire()
    message = str(msg.payload.decode("utf-8"))
    print("\nArrived topic: %s" % msg.topic)
    print("Arrived message: %s" % message)

    if msg.topic == "core/edge/" + SDC_id + "/feedback":
        # compute feedback
        counter += 1
        feedback = float(message)
        print("feedback: %s" % feedback)
        pid.cyclic_compute_basic(feedback, counter)
        output = pid.output

        # if pid.setpoint > 0:
        #     compensated_feedback = feedback + output - (1 / counter)  # / counter???
        # if counter > 9:
        #     pid.setpoint = CACHE_MAX_SIZE * 0.9

        # target_utilization = output * pid.delta_time + feedback

        # compensated_feedback = feedback + output - (1 / counter)  # / counter???
        # compensated_feedback = (target_utilization - feedback) / pid.delta_time
        # compensated_feedback = target_utilization
        # print("Feedback: %s /// Output: %s /// Target_utilization: %s /// Compensated feedback: %s" %
        #       (feedback, output, target_utilization, compensated_feedback))

        # compensated_feedback = output

        # Value chart
        # feedback_list.append(compensated_feedback)
        # setpoint_list.append(pid.setpoint)
        # Percentage chart

        # feedback is lower than 0... problem
        print("Output: %s" % output)
        if output != 0:
            # split and distribute data
            f = open("100MB.zip", "rb")
            data = f.read(int(output))
            print("Cram %s of data" % len(data))
            publish.single("core/edge/" + SDC_id + "/data", data, hostname=MQTT_HOST, port=MQTT_PORT, qos=2)

            # print("Cram %s of data" % int(output))
            # publish.single("core/edge/" + SDC_id + "/data", int(output), hostname=MQTT_HOST, port=MQTT_PORT)
        else:
            print("Flow_control(Skip to send data)")
            publish.single("core/edge/" + SDC_id + "/flow_control", "Controlling flow", hostname=MQTT_HOST,
                           port=MQTT_PORT, qos=2)

        feedback_list.append(feedback)
        percentage_feedback_list.append(feedback / CACHE_MAX_SIZE * 100)
        output_list.append(output)
        percentage_output_list.append(output / CACHE_MAX_SIZE * 100)
        setpoint_list.append(pid.setpoint)
        percentage_setpoint_list.append(pid.setpoint / CACHE_MAX_SIZE * 100)
        time_list.append(counter)

        print("Counter: %s" % counter)

    elif msg.topic == "core/edge/" + SDC_id + "/init_for_testing":
        print("Initialize for testing")
        feedback_list = []
        percentage_feedback_list = []
        output_list = []
        percentage_output_list = []
        time_list = []
        setpoint_list = []
        percentage_setpoint_list = []
        scenario_counter = int(msg.payload)
        print("Scenario Number: %s" % scenario_counter)

        feedback = 0
        counter = 0
        pid.initialize(setpoint=CACHE_MAX_SIZE * 0.9, remaining_ratio_upto_the_max=0.1)
        print("Setpoint: %s" % pid.setpoint)
        pid.sample_time = SAMPLE_TIME
        test_start_time = time.time()
        feedback_list.append(0)
        percentage_feedback_list.append(0)
        output_list.append(0)
        percentage_output_list.append(0)
        setpoint_list.append(pid.setpoint)
        percentage_setpoint_list.append(pid.setpoint / CACHE_MAX_SIZE * 100)
        time_list.append(0)
        publish.single("core/edge/" + SDC_id + "/start_testing", "Start!!", hostname=MQTT_HOST,
                       port=MQTT_PORT, qos=2)

    elif msg.topic == "core/edge/" + SDC_id + "/done_to_test":
        print("A test is finished")
        test_end_time = time.time()
        file_name = str(scenario_counter) + "-" + time.strftime("%Y%m%d%H%M%S") + ".csv"
        print(file_name)
        full_path = os.path.join(os.path.join(".", "testlog"), file_name)
        print(full_path)

        error_square_list2 = [(percentage_feedback_list[i] - percentage_setpoint_list[i]) ** 2 for i in
                              range(2, len(percentage_feedback_list))]
        variance2 = sum(error_square_list2) / (len(percentage_feedback_list) - 2)
        standard_deviation2 = variance2 ** 0.5

        with open(full_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            # print running time
            writer.writerow([test_start_time, test_end_time, test_end_time - test_start_time])
            # print variance and stdev
            writer.writerow([variance2, standard_deviation2])
            # print data
            for idx in range(len(feedback_list)):
                writer.writerow(
                    [time_list[idx], percentage_feedback_list[idx], output_list[idx], percentage_setpoint_list[idx], feedback_list[idx],
                     abs(feedback_list[idx] - pid.setpoint) / pid.setpoint * 100])
        csvfile.close()

        # scenario_counter = scenario_counter % 4 + 1

    elif msg.topic == "core/edge/" + SDC_id + "/all_test_complete":
        print("Test complete!!")
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
# publish.single("elevator/starting_floor_number", "3", hostname=MQTT_HOST, port=MQTT_PORT)
# publish.single("elevator/destination_floor_number", "2", hostname=MQTT_HOST, port=MQTT_PORT)
# ------------------------------------------------------------------------------------------------------------------#


if __name__ == "__main__":

    # MQTT connection
    message_client = mqtt.Client("EDCrammer")
    message_client.on_connect = on_connect
    message_client.on_message = on_message

    # Connect to MQTT broker
    message_client.connect(MQTT_HOST, MQTT_PORT, 60)

    print("MQTT client start")
    message_client.loop_start()

    # wait until done to test
    while not is_finish:
        time.sleep(0.001)

    # print(percentage_feedback_list)
    # print(time_list)
    # print(percentage_setpoint_list)
    #
    # error_square_list = [(percentage_feedback_list[i] - percentage_setpoint_list[i]) ** 2 for i in
    #                      range(2, len(percentage_feedback_list))]
    # variance = sum(error_square_list) / (len(percentage_feedback_list) - 2)
    # standard_deviation = variance ** 0.5
    #
    # print("Variance: %s" % variance)
    # print("Standard deviation: %s" % standard_deviation)
    #
    # time_sm = np.array(time_list)
    # time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)

    ##################################

    # feedback_smooth = spline(time_list, percentage_list, time_smooth)
    # Using make_interp_spline to create BSpline

    # Smooth graph
    # helper_x3 = make_interp_spline(time_list, percentage_feedback_list)
    # feedback_smooth = helper_x3(time_smooth)
    #
    # helper_x3 = make_interp_spline(time_list, percentage_output_list)
    # output_smooth = helper_x3(time_smooth)
    #
    # plt.plot(time_smooth, feedback_smooth, marker='o', markersize=3, linestyle='-')
    # plt.plot(time_smooth, output_smooth, marker='o', markersize=3, linestyle='-')
    # plt.plot(time_list, percentage_setpoint_list)

    # # Real value graph
    # plt.plot(time_list, percentage_feedback_list, marker='o', markersize=3, linestyle='-')
    # plt.plot(time_list, percentage_output_list, marker='o', markersize=3, linestyle='-')
    # plt.plot(time_list, percentage_setpoint_list)
    #
    # plt.xlim((1, counter))
    # # plt.ylim((min(percentage_list) - 0.5, max(percentage_list) + 0.5))
    # # plt.ylim(0, 100)
    # plt.xlabel('Round no.')
    # plt.ylabel('PID (PV)')
    # plt.title('TEST PID')
    #
    # plt.grid(True)
    # plt.show()
    # # plt.ylim((1 - 0.5, 1 + 0.5))

    message_client.loop_stop()
    # Threads completely executed
