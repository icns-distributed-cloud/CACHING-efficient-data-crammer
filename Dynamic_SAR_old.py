# -*- coding: utf-8 -*-

"""
http://virtualizedfrog.wordpress.com/   2014
Translated from http://www.amibroker.com/library/detail.php?id=268
Requires pandas to load csv files, and matplotlib to chart the data
The main expects table.csv file. Valid files can be downloaded on Yahoo Finance
eg: http://real-chart.finance.yahoo.com/table.csv?s=%5EGSPC&ignore=.csv
"""
from time import sleep
import random
import pandas as pd
import numpy as np

# import matplotlib.pyplot as plt

barsdata = pd.DataFrame(data=[[500], [493]], index=range(0, 2), columns=['Close'])
iaf = 0.04
dynamic_af = 0

trigger_point = 300
up_bound = 700
down_bound = 200

trigger_point2 = 700
up_bound2 = 1100
down_bound2 = 600
down_bound3 = 1000
cnt_container = 1
location = []
cover = 0


###### Global variable with barsdata and iaf  ######
def ema(L, alpha=None):
    ema_data = []

    if not alpha:
        alpha = 1 / (len(L) + 1.25)

    if (alpha < 0) or (alpha > 1):
        raise ValueError("0 < smoothing factor <= 1")

    alpha_bar = float(1 - alpha)
    num_terms_list = [sorted(L[:i], reverse=True) for i in range(len(L), len(L) + 1)]
    for nterms in num_terms_list:
        pre_exp_factor = [float(alpha_bar ** (i - 1)) for i in range(1, len(nterms))]
        ema_data.append(
            alpha * float(sum(float(a) * float(b) for a, b in zip(tuple(pre_exp_factor), tuple(nterms[:-1])))) + (
                        alpha_bar ** (len(nterms) - 1)) * float(nterms[-1]))

    return ema_data


def psar(tmp, maxaf=0.6):
    a = pd.DataFrame(data=[tmp], columns=['Close'])
    global barsdata
    global iaf
    global dynamic_af
    barsdata = barsdata.append(a)  # barsdata = barsdata.reset_index(drop=True)
    length = len(barsdata)
    close = list(barsdata['Close'])
    psar = close[0:len(close)]
    psarbull = [0] * length
    psarbear = [0] * length
    bull = True
    af = iaf
    ep = close[0]

    for i in range(1, length):
        psar[i] = psar[i - 1] + (af + dynamic_af) * (ep - psar[i - 1])

        reverse = False

        if bull:
            if close[i] < psar[i]:
                bull = False
                reverse = True
                psar[i] = ep
                ep = close[i]
                af = iaf
        else:
            if close[i] > psar[i]:
                bull = True
                reverse = True
                psar[i] = ep
                ep = close[i]
                af = iaf

        if not reverse:
            if bull:
                if close[i] > ep:
                    ep = close[i]
                    af = min(af + iaf, maxaf)
                if close[i - 1] < psar[i]:
                    psar[i] = close[i - 1]
                if close[i - 2] < psar[i]:
                    psar[i] = close[i - 2]
            else:
                if close[i] < ep:
                    ep = close[i]
                    af = min(af + iaf, maxaf)
                if close[i - 1] > psar[i]:
                    psar[i] = close[i - 1]
                if close[i - 2] > psar[i]:
                    psar[i] = close[i - 2]

        if bull:
            psarbull[i] = psar[i]
        else:
            psarbear[i] = psar[i]

    return {"close": close, "psar": psar, "psarbear": psarbear, "psarbull": psarbull}


def normalization(num):
    if num < 0:
        num = -(num)
    tmp = (num * 0.2) / 300

    #	print(tmp)
    return tmp


def check(bull, bear):
    global trigger_point
    global up_bound
    global down_bound
    global trigger_point2
    global up_bound2
    global down_bound2
    global down_bound3
    global cnt_container
    global location

    if cnt_container == 1:
        if bull > 0:
            print("#1-1#")
            if bull > trigger_point:
                print("Incease container 2")
                cnt_container = cnt_container + 1
                location.append([800])
            else:
                print("Nothing happen77777")
                location.append([400])
        elif bear > 0:
            print("#1-2#")
            if bear > trigger_point:
                print("Increase Container 4")
                cnt_container = cnt_container + 1
                location.append([800])
            else:
                print("Nothing happen88888")
                location.append([400])
        else:
            print("Nothing happen11111")
    elif cnt_container == 2:
        if bull > 0:
            print("#2-1#")
            if bull > down_bound and bull < up_bound:
                print("Keep the container__7")
                location.append([800])
            elif bull > up_bound:
                print("increase the number of container")
                cnt_container = cnt_container + 1
                location.append([1200])
            elif bull < down_bound:
                print("decrease the number of container")
                cnt_container = cnt_container - 1
                location.append([400])
            else:
                print("NH")
        elif bear > 0:
            print("#2-2#")
            if bear > down_bound and bull < up_bound:
                print("keep the container__8")
                location.append([800])
            elif bear < down_bound:
                print("decrease number of container")
                cnt_container = cnt_container - 1
                location.append([400])
            elif bear > up_bound:
                print("increase the number of container")
                cnt_container = cnt_container + 1
                location.append([1200])
            else:
                print("NH")
        else:
            print("Nothing happen22222")
    elif cnt_container == 3:
        if bull > 0:
            print("#3-1#")
            if bull > down_bound2 and bull < up_bound2:
                print("Keep the container")
                location.append([1200])
            elif bull > up_bound2:
                print("increase the number of container")
                cnt_container = cnt_container + 1
                location.append([1600])
            elif bull < down_bound2:
                print("decrease the number of container")
                cnt_container = cnt_container - 1
                location.append([800])
            else:
                print("Nothing happen")
        elif bear > 0:
            print("#3-2#")
            if bear > down_bound2 and bear < up_bound2:
                print("Keep the container")
                location.append([1200])
            elif bear < down_bound2:
                print("decrease the number of container")
                cnt_container = cnt_container - 1
                location.append([800])
            elif bear > up_bound2:
                print("increase the number of container")
                cnt_container = cnt_container + 1
                location.append([1600])
            else:
                print("Nothing happen")
        else:
            print("Nothing happen")
    else:
        if bear > 0:
            if bear < down_bound3:
                location.append([1200])
                cnt_container = cnt_container - 1
            else:
                location.append([1600])
        elif bull > 0:
            if bull < down_bound3:
                location.append([1200])
                cnt_container = cnt_container - 1
            else:
                location.append([1600])
        else:
            print("nh")


if __name__ == "__main__":
    import sys
    import os
# array_data = [508, 519, 531, 547, 564, 550, 532, 511, 491, 480, 462, 431, 403, 428, 449, 463, 489, 505, 493, 461, 436, 407, 381, 355, 322, 302, 348, 381, 427, 469, 501, 538, 572, 601, 621, 563, 509, 442, 381, 319, 258, 175, 121, 102, 183, 261, 318, 389, 477, 580, 698, 830, 974, 998, 974, 961, 841, 668, 573, 461, 372, 308, 261, 182, 147, 103, 123, 371, 518, 670, 819, 999, 991, 985, 908, 804, 640, 420, 428, 444, 461, 483, 498, 501, 511, 528, 537, 544, 563, 589, 602, 587, 564, 552, 531, 543, 550, 541]
array_data = [508, 519, 531, 547, 564, 550, 532, 511, 491, 480, 518, 573, 649, 693, 748, 804, 927, 973, 999, 946, 841,
              732, 612, 434, 217, 109, 100, 113, 125, 131, 180, 238, 299, 378, 618, 920, 927, 970, 997, 936, 847, 752,
              642, 484, 317, 109, 100, 113, 127, 131, 146, 180, 236, 289, 378, 598, 607, 584, 555, 526, 498, 451, 407,
              361, 303, 298, 317, 355, 396, 441, 489, 493, 500, 493, 461, 437, 414, 400, 427, 469, 501, 517, 536, 544,
              563, 589, 602, 587, 564, 552, 531, 543, 550, 532, 511, 491, 480, 458]
#############    Put data above this line      #################

short_ema = []
long_ema = []
aver_ema = []
average = []
sar_array = []
bull_array = []
bear_array = []
sub = []
i = 0
subtmp = 0
while i < len(array_data):
    tmp = array_data[i]
    startidx = 0
    endidx = len(barsdata)
    result = psar(tmp)
    sarv = result['psar'][len(result['psar']) - 1]
    bull = result['psarbull'][len(result['psarbull']) - 1]
    bear = result['psarbear'][len(result['psarbear']) - 1]
    #	print(sarv)
    sar_array.append(sarv)
    bull_array.append(bull)
    bear_array.append(bear)

    if i > 3:
        new_data = array_data[i - 4:i + 1]
        short = ema(new_data)
        short_ema.extend([short])
    else:
        short_ema.extend([0])

    if i > 10:
        new_data2 = array_data[i - 11:i + 1]
        longg = ema(new_data2)
        long_ema.extend([longg])
    else:
        long_ema.extend([0])
        location.append([400])

    if i > 10:
        subtmp = np.array(short) - np.array(longg)
        #		print(subtmp[0])  # Calcuate the gap between shortEma-longEma
        sub.extend(subtmp)
        dynamic_af = normalization(subtmp[0])  ## Dynamic af changes by the value of the gap!!
        #		print(dynamic_af)
        sumtmp = (np.array(short) + np.array(longg)) / 2  ## Average of Short ema & Long ema
        real_value = array_data[i]
        gap = real_value - sumtmp
        average.extend(sumtmp)
        check(bull, bear)
    #	check(bull, bear)

    i = i + 1
## I need normailzation in here and rearrange the iaf ##
## Below is the result of data
# print(short_ema)
# print(long_ema)
# print(sar_array)
print(bull_array)
print(bear_array)
# print(average)
# print(location)
# print(sub)
