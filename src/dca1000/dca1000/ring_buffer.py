import numpy as np
from queue import Queue
from ctypes import *
from circular_buffer import *
from numpy.ctypeslib import ndpointer


class RingBuffer:
    def __init__(self, max_len, frame_size, dtype=np.int16):
        self.max_len = c_int64(max_len)
        self.data = np.zeros(max_len, dtype=dtype)
        self.queue = Queue()
        self.put_idx = c_int64(0)
        self.frame_size = c_int64(frame_size)
        self.pop_array = c_int16(-1)
        self.total = []

    def add_zeros(self,num_zeros):
        add_zeros(num_zeros,
                    self.data,
                    self.max_len,
                    byref(self.put_idx),
                    self.frame_size,
                    byref(self.pop_array))
        self.add_to_queue()

    def add_msg(self, msg):

        add_msg(msg,
                len(msg),
                self.data,
                self.max_len,
                byref(self.put_idx),
                self.frame_size,
                byref(self.pop_array))
        self.add_to_queue()

    def pad_and_add_msg(self, seq_c, seq_n, msg):
        pad_and_add_msg(seq_c,
                        seq_n,
                        msg,
                        len(msg),
                        self.data,
                        self.max_len,
                        byref(self.put_idx),
                        self.frame_size,
                        byref(self.pop_array))
        self.add_to_queue()

    def add_to_queue(self):
        if self.pop_array.value != -1:
            data = self.data[self.frame_size.value * self.pop_array.value:self.frame_size.value * (self.pop_array.value + 1)].copy()
            self.queue.put(data)
