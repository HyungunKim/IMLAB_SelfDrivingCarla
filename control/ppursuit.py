
import numpy as np
import matplotlib.pyplot as plt
from fkmodel import TimeTest
from geomodule import *

"""
gdict = {'s' : start, 'length' : length, 'x' : x, 'y' : y, 'hdg' :hdg, 'type' : (line or arc), 'curvature' : optional curvature} 
"""

class refline:
    def __init__(self, geometries):
        self.geometries = geometries
        self.length = 0
        for gdict in geometries:
            self.length += gdict['length']

    def getpoint(self, u):
        for gdict in self.geometries:
            if (gdict['s'] <= u and u <= gdict['s'] + gdict['length']):
                l = u - gdict['s']
                
                return gdictpoint(gdict,l)

def LookAHead(x, y, road, hdg, ld):
    u0 = ternarySearch(x, y, road)

    l = u
    r = road.length
    
    while (r - l > 0.2):
        p = 2.0/3.0 * l + 1.0/3.0 * r
        q = 1.0/3.0 * l + 2.0/3.0 * r
        l2p = np.abs(np.sqrt(L2(x, y, road, p)) - ld)
        l2q = np.abs(np.sqrt(L2(x, y, road, q)) - ld)
        if l2p < l2q:
            r = q
        else:
            l = p
    return road.getpoint((l + r)/2.0)


def pstep(state, target):
    # state = vehicle.x, vehicle.y, vehicle.yaw
    # target = Gx, Gy (x, y point in reference line)
    xw = target[0] - state[0]
    yw = target[1] - state[1]
    yaw = state[2]

    ls = xw**2 + yw**2

    x = np.cos(yaw)*xw + np.sin(yaw)*yw
    y = np.cos(yaw)*yw - np.sin(yaw)*xw

    gamma = 2*x/ls

    steer_angle = np.arctan(gamma)
    steer_control = steer_angle / STEER_GAIN

    return steer_control