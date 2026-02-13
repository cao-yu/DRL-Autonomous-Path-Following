#!/usr/bin/env python

import math
import numpy as np

from scipy.integrate import quad
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize, newton, root_scalar
from param_fcn import StraightLine, LemniscateCurve, LaneChange


class ArcReferencePath:
    def __init__(self, *args):
        path = args[0]
        if len(args) > 1:
            var = args[1:]
        else:
            var = None

        if path.lower() in ["straight", "eight", "change"]:
            if path.lower() == "straight":
                self.path = StraightLine(var)
            elif path.lower() == "eight":
                self.path = LemniscateCurve(var)
            elif path.lower() == "change":
                self.path = LaneChange(var)
        else:
            print("No such path!")        
        self.X, self.Y, self.len = arc_parameterized(self.path)
        self.dX = self.X.derivative(1)	
        self.dY = self.Y.derivative(1)
         
    def calc_yaw(self, s):     
        dx, dy = self.dX(s), self.dY(s)   
        return np.arctan2(dy, dx)
    
    def find_nearest_point(self, s0, x, y):
        """
            Find the value of s = argmin||(xr, yr) - (x, y)||  
        """
        def distance_squared(_s, *args):
            _x, _y = self.X(_s), self.Y(_s) 
            return (_x - args[0]) ** 2 + (_y - args[1]) ** 2
        
        def derivative(_s, *args):
            _x, _y = self.X(_s), self.Y(_s) 
            _dx, _dy = self.dX(_s), self.dY(_s)  
            return 2 * _dx * (_x - args[0]) + 2 *_dy *(_y-args[1])
    
        result = minimize(distance_squared, x0=s0, jac=derivative, 
                          args=(x, y), method='CG')
        return result.x[0]
    
    def find_lookahead_point(self, s0, x, y, L):
        """
            Find the value of s that satisfies ||(xr, yr) - (x, y)|| = L 
        """
        def distance_squared(_s):  
            return  (self.X(_s) - x)**2 + (self.Y(_s) - y)**2 - L**2
        
        def derivative(_s):
            return 2 * (self.X(_s) - x) * self.dX(_s) + 2 * (self.Y(_s) - y) * self.dY(_s)
        
        sol = root_scalar(f=distance_squared, x0=s0, fprime=derivative, 
                          method='newton') 
        #sol = newton(distance_squared, x0=s0, fprime=derivative) 
        return sol.root
       
    def calc_error(self, x, y, yaw, s):
        """
            compute the CTE and orientation error from a reference point
        """
        x_ref, y_ref = self.X(s), self.Y(s) 
        tx, ty = self.dX(s), self.dY(s) 
        
        # cross track error (left to path, e>0; right to path, e<0)
        dx, dy = x - x_ref, y - y_ref
        cte = (dy * tx - dx * ty) / np.sqrt(tx ** 2 + ty ** 2)
        
        # orientation error
        yaw_ref = self.calc_yaw(s)   
        psi = angle_normalize(yaw - yaw_ref)
        return cte, psi

    def seed(self, seed=None):
        self.np_random = np.random.RandomState(seed)



def angle_normalize(theta): # in the range *[-pi, pi]*
    return math.atan2(math.sin(theta), math.cos(theta)) 



def arc_len(s, dX, dY):
    integrand = lambda u:np.sqrt(dX(u)**2 + dY(u)**2)
    result, _ = quad(integrand, 0, s)
    return result
    
    
 
def arc_parameterized(path):
    s_values = np.linspace(0, path.len, 300)
    arc_length = np.array([arc_len(s, path.dX, path.dY) for s in s_values])	
    length = arc_length[-1]
    x, y = path.X(s_values), path.Y(s_values)
    x_interp = CubicSpline(arc_length, x)
    y_interp = CubicSpline(arc_length, y)
    return x_interp, y_interp, length

