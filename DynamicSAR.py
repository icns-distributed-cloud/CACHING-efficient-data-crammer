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
# title           : DynamicSAR.py
# description     : python dynamic trend tracker based on moving average and parabolic Stop And Reverse (SAR)
# author          : Yunkon(Alvin) Kim
# date            : 20190206
# version         : 0.1
# python_version  : 3.6
# notes           : This DynamicSAR is an implementation of trend tracker based on parabolic SAR applying gap of
#                   short MA and long MA in the Python Programming Language.
# ==============================================================================
import CircularList


class DynamicSAR:
    """
    출처: 파라볼릭(Prabolic SAR) 계산해 보기
    (http://blog.daum.net/_blog/BlogTypeView.do?blogid=0W7UP&articleno=1833&categoryId=61&regdt=20140227005609)

    Parabolic SAR (Stop And Reverse)의 개념:
    시간의 추이에 따라 주가의 움직임을 변수로 하는 함수를 만들어, 시간의 경과와 일치하는 주가의 변화가 일어나지 않을 경우
    이를 추세의 전환시점으로 보고 기존 포지션을 중지하고 그와는 반대의 포지션을 취할 수 있는 기준을 제공해 준다.
    계산식:
        EP: Extreme Price (상승추세에서는 신고가, 하락추세에서는 신저가)
        AF: Acceleration Factor (가속변수)
            - 시간경과에 따른 가속 변수
            - 0.02부터 시작해서 고가/저가를 갱신할 떄마다 0.02씩 증가
            - 최대값(0.2)을 넘어갈 수 없음. (0.02 * 10일 = 0.2, 즉 고가/저가를 10번 갱신하면 가속 변수는 0.2로 고정됨)

    변수값
        - AF(가속변수): 0.02를 주로 사용
        - 최대값(최대AF): 0.2를 주로 사용
    """

    def __init__(self, short_term_sampling_size, long_term_sampling_size):
        # a temporal list to store data
        self.data_list = []
        self.data_index = 0
        # a circular list of short term moving average
        self.short_term = CircularList.CircularList(short_term_sampling_size)
        self.__short_term_average = None
        # a circular list of long term moving average
        self.long_term = CircularList.CircularList(long_term_sampling_size)
        self.__long_term_average = None

        # The member variables for calculation of Dynamic SAR, which utilizes the moving average and Parabolic SAR
        # lsmaaf: Long Short Moving Average Acceleration Factor
        self.psar_list = []
        self.psarbull_list = []
        self.psarbear_list = []
        self.bull = True
        self.lsmaaf = 0
        self.ep = 0
        self.af = self.iaf = 0.02
        self.maxaf = 0.2
        self.reverse = False

    def put(self, data):
        self.data_list[self.data_index] = data
        self.short_term.put(data)
        short_list = self.short_term.list
        if None not in short_list:
            self.__short_term_average = sum(short_list) / float(len(short_list))

        self.long_term.put(data)
        long_list = self.long_term.list
        if None not in long_list:
            self.__long_term_average = sum(long_list) / float(len(long_list))

        if self.__short_term_average is None or self.__short_term_average is None:
            self.psar_list[self.data_index] = self.data_list[self.data_index]
            self.ep = self.data_list[self.data_index]
        else:
            diff_mean = self.__short_term_average - self.__long_term_average
            # Normalize is necessary.
            self.lsmaaf = self.normalization(diff_mean)
            self.parabolic_sar(self.lsmaaf)

        self.data_index += 1

    def parabolic_sar(self, lsmaaf):
        psar = self.psar_list
        i = self.data_index
        psar[i] = psar[i - 1] + (self.af + lsmaaf) * (self.ep - psar[i - 1])

        if self.bull:
            if self.data_list[i] < psar[i]:
                self.bull = False
                self.reverse = True
                psar[i] = self.ep
                self.ep = self.data_list[i]
                self.af = self.iaf
        else:
            if self.data_list[i] > psar[i]:
                self.bull = True
                self.reverse = True
                psar[i] = self.ep
                self.ep = self.data_list[i]
                self.af = self.iaf

        if not self.reverse:
            if self.bull:
                if self.data_list[i] > self.ep:
                    self.ep = self.data_list[i]
                    self.af = min(self.af + self.iaf, self.maxaf)
                if self.data_list[i-1] < psar[i]:
                    psar[i] = self.data_list[i-1]
                if self.data_list[i-2] < psar[i]:
                    psar[i] = self.data_list[i-2]
            else:
                if self.data_list[i] < self.ep:
                    self.ep = self.data_list[i]
                    self.af = min(self.af + self.iaf, self.maxaf)
                if self.data_list[i-1] > psar[i]:
                    psar[i] = self.data_list[i-1]
                if self.data_list[i-2] > psar[i]:
                    psar[i] = self.data_list[i-2]

        if self.bull:
            self.psarbull_list[i] = psar[i]
            self.psarbear_list[i] = None
        else:
            self.psarbear_list[i] = psar[i]
            self.psarbull_list[i] = None

    # The below function should be updated due to unclear gains of normalization, 0.2, 300
    @staticmethod
    def normalization(num):
        if num < 0:
            num = -num
        temp = (num * 0.2) / 300

        return temp

    # @property
    # def short_term_average(self):
    #     if self.__short_term_average is not None:
    #         return self.__short_term_average
    #     else:
    #         return "Less sampling data"
    #
    # @property
    # def long_term_average(self):
    #     if self.__long_term_average is not None:
    #         return self.__long_term_average
    #     else:
    #         return "Less sampling data"
