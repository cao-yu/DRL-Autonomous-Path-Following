#!/usr/bin/env python

import numpy as np



class StraightLine:
    def __init__(self, *args):
        if args[0] is None:
            self.a = 2.0 # default
        else:
            self.a = args[0][0]
        self.len = self.a

    def X(self, s):
        return s

    def dX(self, s):
        return np.ones_like(s)

    def Y(self, s):
        return np.zeros_like(s)

    def dY(self, s):
        return np.zeros_like(s)

    

class LemniscateCurve:
    def __init__(self, *args):
        if args[0] is None:
            self.a = 1.0 # default
        else:
            self.a = args[0][0]
        self.len = 2 * np.pi

    def X(self, s):
        return self.a * np.sin(s)

    def dX(self, s):
        return self.a * np.cos(s)

    def Y(self, s):
        return self.a * np.sin(s) * np.cos(s)

    def dY(self, s):
        return self.a * (np.cos(s) ** 2 - np.sin(s) ** 2)

    

class LaneChange:
    def __init__(self, *args):
        if args[0] is None:
            self.b = 1.5
            self.c = 1.5
            self.k = 30.0
        else:
            self.b = args[0][0]
            self.c = args[0][1]
            self.k = args[0][2]

        self.len = 2 * self.c 

    def X(self, s):
        return s

    def dX(self, s):
        return np.ones_like(s)

    def Y(self, s):
        return (self.b) / (1 + np.exp(-self.k*(s-self.c)))

    def dY(self, s):
        return (self.b * self.k * np.exp(-self.k*(s-self.c))) / (1 + np.exp(-self.k*(s-self.c))) ** 2

