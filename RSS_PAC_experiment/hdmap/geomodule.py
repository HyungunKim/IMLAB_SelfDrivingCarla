import math
import numpy as np

from collections import defaultdict



def xu(u, v_u, x0, hdg):
    beta = hdg + math.atan2(v_u,u)
    x = x0 + math.sqrt(u**2 + v_u**2) * math.cos(beta)
    return x

def yu(u, v_u, y0, hdg):
    beta = hdg + math.atan2(v_u,u)
    y = y0 + math.sqrt(u**2 + v_u**2) * math.sin(beta)
    return y

def linepoint(s, x, y, hdg, length):
    fxu = lambda u: xu(u, 0, x, hdg)
    fyu = lambda u: yu(u, 0, y, hdg)
    
    return fxu(length), -fyu(length) # minus sign convention in carla

def arcpoint(s, x, y, hdg, curvature, length):
    
    ulocal = lambda l: np.sin((l)*curvature)/curvature
    vlocal = lambda l: np.cos((l)*curvature)/curvature - 1/curvature
    
    fxl = lambda l: xu(ulocal(l), vlocal(l), x, hdg)
    fyl = lambda l: yu(ulocal(l), vlocal(l), y, hdg)

    return fxl(length), -fyl(length) # minus sign convention in carla
    
def parapoly(s, a, b, c, d):
    s = float(s)
    a = float(a)
    b = float(b)
    c = float(c)
    d = float(d)
    return lambda u: a + b*(u-s) + c*(u-s)**2 + d*(u-s)**3

class parapoly3:
    def __init__(self, s, e, a, b, c, d):
        self.s = float(s)
        self.e = float(e)
        self.a = float(a)
        self.b = float(b)
        self.c= float(c)
        self.d = float(d)

        self.poly = parapoly(s, a, b, c, d)

    def __call__(self, u):
        if (self.s <= u and u <= self.e):
            return self.poly(u)
        else:
            return None

    def __add__(self, other):
        s = self.s
        e = self.e
        a = self.a + other.a
        b = self.b + other.b
        c = self.c + other.c
        d = self.d + other.d
        return parapoly3(s, e, a, b, c, d)

    def mult(self, num):
        s = self.s
        e = self.e
        a = self.a * num
        b = self.b * num
        c = self.c * num
        d = self.d * num
        return parapoly3(s, e, a, b, c, d)

class parapolylinks:
    def __init__(self, precedings = [], parapolys = []):
        self.precedings = precedings.copy()
        self.parapolys = parapolys.copy()

    def __call__(self, u, cumulative=True):
        ans = 0
        for pre in self.precedings:
            ans += pre(u, False)
        for poly in self.parapolys:
            val = poly(u)
            if val is not None:
                if cumulative:
                    return val + ans
                return val
        if cumulative:
            return ans
        return 0

    def swidth(self, u):
        ans = 0
        for pre in self.precedings:
            ans += pre(u, False)
        return ans

    def ewidth(self, u):
        return self.__call__(u)




def gdictpoint(gdict, length):
    x = gdict['x']
    y = gdict['y']
    hdg = gdict['hdg']
    if (gdict['type'] == 'line'):
        return linepoint(0, x, y, hdg, length)
    else:
        curvature = gdict['curvature']
        return arcpoint(0, x, y, hdg, curvature, length)



def L2(x, y, road, u):
    rx, ry = road.getpoint(u)
    return (x -  rx)**2 + (y - ry)**2

def ternarySearch(x, y, road):
    l=0
    r = road.length
    
    while (r - l > 0.2):
        p = 2.0/3.0 * l + 1.0/3.0 * r
        q = 1.0/3.0 * l + 2.0/3.0 * r
        l2p = L2(x, y, road, p)
        l2q = L2(x, y, road, q)
        if l2p < l2q:
            r = q
        else:
            l = p
    return (l + r)/2.0

def prdist(x, y, road):
    u = ternarySearch(x, y, road)
    l2 = L2(x, y, road, u)
    return math.sqrt(l2)

def prdist2(x, y, road, getu=False):
    u = ternarySearch(x, y, road)
    l2 = L2(x, y, road, u)
    
    
    rx0, ry0 = road.getpoint(u - 0.1)
    if (rx0 is None):
        rx0, ry0 = road.getpoint(u)
    
    rx1, ry1 = road.getpoint(u + 0.1)
    if (rx1 is None):
        rx1, ry1 = road.getpoint(u + 0.05)
    
    p = np.array([x, y, 0]) - np.array([rx0, ry0, 0])
    du = np.array([rx1, ry1, 0]) - np.array([rx0, ry0, 0])
    duxp = np.cross(du, p)
    if getu:
        return -math.sqrt(l2)*np.sign(duxp[-1]), u
    return -math.sqrt(l2)*np.sign(duxp[-1])


