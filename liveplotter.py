# importing libraries
import time

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

matplotlib.use('TkAgg')
plt.style.use('ggplot')


def live_plotter(x_vec, y1_data, line1, identifier='', pause_time=0.1):

    if line1 == []:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13, 6))
        ax = fig.add_subplot(111)

        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec, y1_data, '-o', alpha=1)

        # update plot label/title
        plt.ylabel('PID output')
        plt.title('Title: {}'.format(identifier))
        plt.show()

    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    # print(x_vec)
    # line1.axes.set_xticklabels([str(item) for item in x_vec])
    # Remove lable of x-axis
    line1.axes.set_xticks([])

    # adjust limits if new data goes beyond bounds
    if np.min(y1_data) <= line1.axes.get_ylim()[0] or np.max(y1_data) >= line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data) - np.std(y1_data), np.max(y1_data) + np.std(y1_data)])

    # # adjust limits if new data goes beyond bounds
    # if np.min(x_vec) <= line1.axes.get_xlim()[0] or np.max(x_vec) >= line1.axes.get_xlim()[1]:
    #     plt.xlim([np.min(x_vec) - np.std(x_vec), np.max(x_vec) + np.std(x_vec)])

    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)

    # return line so we can update it again in the next iteration
    return line1

