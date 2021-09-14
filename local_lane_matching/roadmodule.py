
import math
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
import xml.etree.ElementTree as elemTree
from queue import PriorityQueue

from geomodule import *

#tree = elemTree.parse("Town06.xodr")
iload_world = "Town01"
tree = elemTree.parse(f"{iload_world}.xodr")
roads = []
for road in tree.findall("road"):
    roads.append(road)

roadsdict = {} # {road_id: <class: Roadf>, ...}
junctionsdict = {} # {-1: [road_a, ...], jid: [road_b, road_c, ..]}

class Lanef:
    def __init__(self, road_id, section_id, lane_id, ss, es, widthf, predecessor_id, successor_id, lane_type="unknown"):
        self.road_id = road_id
        self.section_id = section_id
        self.lane_id = lane_id
        self.lane_type = lane_type
        self.ss = ss
        self.es = es
        self.widthf = widthf
        self.predecessor_id = predecessor_id
        self.successor_id = successor_id
        self.predecessors = []
        self.successors = []
        self.leftlane = []
        self.rightlane = []

    def link_front(self, other):
        self.successors.append(other)
        other.predecessors.append(self)

    def link_right(self, other):
        self.rightlane.append(other)
        other.leftlane.append(self)

    def link_left(self, other):
        self.leftlane.append(other)
        other.rightlane.append(self)


class LaneSection:
    def __init__(self, road_id, section_id, ss, es, leftlanes=[], rightlanes=[]):
        self.road_id = road_id # <int> id of road
        self.section_id = section_id # <int> id of section
        self.ss = ss # <float> section start length
        self.es = es # <float> section end length
        self.length = es - ss
        self.predecessor = [] # predecessor lane section
        self.successor = [] # successor lane section
        self.leftlanes = leftlanes # left lanes xml in this section
        self.rightlanes = rightlanes # right lanes xml in this section


        self.leftlanes.sort(key = lambda x: abs(int(x.items()[0][1])))
        self.rightlanes.sort(key = lambda x: abs(int(x.items()[0][1])))

        self.lanes = {} # <Lanef> for all drivable roads;
        self.offset = parapolylinks(precedings=[],parapolys=roadsdict[self.road_id].laneoffsets)
        if self.road_id == 1196:
            letsdebug = True


    def link(self, other): # other is successor of self
        self.successor.append(other)
        other.predecessor.append(self)

    def buildlanes(self):
        sn = 1
        if len(self.lanes) != 0: return
       
        mxlane = len(self.leftlanes)
        preceding_lanes = [self.offset]

        for i in range(mxlane):
            lane = self.leftlanes[i]
            lane_type = lane.items()[1][1]
            if lane_type == 'none': #???
                continue
            try:
                pred = int(lane.find("link/predecessor").items()[0][1])
            except:
                pred = None
                
            try:
                succ = int(lane.find("link/successor").items()[0][1])
            except:
                succ = None
            lid = int(lane.items()[0][1])
            parapolys = []
            for j, widthxml in enumerate(lane.findall("width")):
                s = self.ss + float(widthxml.items()[0][1])
                if j == len(lane.findall("width")) - 1:
                    e = self.es
                else:
                    e = self.ss + float(lane.findall("width")[j+1].items()[0][1])

                a = float(widthxml.items()[1][1])
                b = float(widthxml.items()[2][1])
                c = float(widthxml.items()[3][1])
                d = float(widthxml.items()[4][1])

                parapolys.append(parapoly3(s, e, a, b, c, d))
            swidth = parapolylinks(preceding_lanes, parapolys)
            preceding_lanes.append(swidth)

            self.lanes[lid] = Lanef(self.road_id, self.section_id, lid, self.ss, self.es, swidth,  pred, succ, lane_type=lane_type)
                
        preceding_lanes = [self.offset]
        mxlane = len(self.rightlanes)
        for i in range(mxlane):
            lane = self.rightlanes[i]
            lane_type = lane.items()[1][1]
            if lane_type == 'none': #???
                continue
            try:
                pred = int(lane.find("link/predecessor").items()[0][1])
            except:
                pred = None
                
            try:
                succ = int(lane.find("link/successor").items()[0][1])
            except:
                succ = None
            lid = int(lane.items()[0][1])
            parapolys = []
            for j, widthxml in enumerate(lane.findall("width")):
                s = self.ss + float(widthxml.items()[0][1])
                if j == len(lane.findall("width")) - 1:
                    e = self.es
                else:
                    e = self.ss + float(lane.findall("width")[j+1].items()[0][1])

                a = -float(widthxml.items()[1][1])
                b = -float(widthxml.items()[2][1])
                c = -float(widthxml.items()[3][1])
                d = -float(widthxml.items()[4][1])

                parapolys.append(parapoly3(s, e, a, b, c, d))
            swidth = parapolylinks(preceding_lanes, parapolys)
            preceding_lanes.append(swidth)

            self.lanes[lid] = Lanef(self.road_id, self.section_id, lid, self.ss, self.es, swidth,  pred, succ, lane_type=lane_type)
        

    def isin(self, x, y):
        if len(self.lanes) == 0: self.buildlanes()
        ans = []
        w, u = prdist2(x, y, roadsdict[self.road_id], getu=True)
        for lane in self.lanes.items():
            #lsign = np.sign(lane[0])
            lb = min(lane[1].widthf.swidth(u), lane[1].widthf.ewidth(u))
            ub = max(lane[1].widthf.swidth(u), lane[1].widthf.ewidth(u))
            if(lb <= w and w <= ub): 
                ans.append(lane[1])
        
        return ans


        

class Roadf:
    def __init__(self, roadxml):
        self.name = roadxml.items()[0][1]
        self.length = float(roadxml.items()[1][1])
        self.id = int(roadxml.items()[2][1])
        self.junction = int(roadxml.items()[3][1])
        self.geometries = []
        self.laneSections = []
        self.nsections = 0

        try:
            self.predecessor = roadxml.find("link/predecessor").items()[0][1], int(roadxml.find("link/predecessor").items()[1][1]), roadxml.find("link/predecessor").items()[2][1]
        except:
            self.predecessor = roadxml.find("link/predecessor").items()[0][1], int(roadxml.find("link/predecessor").items()[1][1]), "junction"
        try:
            self.successor = roadxml.find("link/successor").items()[0][1], int(roadxml.find("link/successor").items()[1][1]) , roadxml.find("link/successor").items()[2][1] 
        except:
            self.successor = roadxml.find("link/successor").items()[0][1], int(roadxml.find("link/successor").items()[1][1]) , "junction"

        roadsdict[self.id] = self
        
        for geometry in roadxml.findall("planView/geometry"):
            gdict = {}
            gdict['s'] =  float(geometry.items()[0][1])
            gdict['x'] = float(geometry.items()[1][1])
            gdict['y'] = float(geometry.items()[2][1])
            gdict['hdg'] = float(geometry.items()[3][1])
            gdict['length'] = float(geometry.items()[4][1])
            if geometry.find("line") != None:
                gdict['type'] = "line"

            else:
                gdict['type'] = "arc"
                gdict['curvature'] = -float(geometry.find("arc").items()[0][1])

            self.geometries.append(gdict)
            
        laneoffsetsx = roadxml.findall("lanes/laneOffset")

        self.laneoffsets = []

        for i, loff in enumerate(laneoffsetsx):
            s = float(loff.items()[0][1])
            a = float(loff.items()[1][1])
            b = float(loff.items()[2][1])
            c = float(loff.items()[3][1])
            d = float(loff.items()[4][1])
            if (i == len(laneoffsetsx) - 1):
                e = self.length
            else:
                e = laneoffsetsx[i + 1].items()[0][1]
            self.laneoffsets.append(parapoly3(s, e, a, b, c, d))

        for i, lsx in enumerate(roadxml.findall("lanes/laneSection")):
            ss = float(lsx.items()[0][1])
            if i == len(roadxml.findall("lanes/laneSection")) - 1:
                es = self.length
            else:
                es = float(roadxml.findall("lanes/laneSection")[i + 1].items()[0][1])

            leftlanes = lsx.findall("left/lane")
            rightlanes = lsx.findall("right/lane")
            self.laneSections.append(LaneSection(self.id, self.nsections, ss, es, leftlanes=leftlanes, rightlanes=rightlanes))
            self.nsections += 1

        
    
    def buildlanes(self):
        for lanesection in  self.laneSections:
            lanesection.buildlanes()

    def getpoint(self, u):
        for gdict in self.geometries:
            if (gdict['s'] <= u and u <= gdict['s'] + gdict['length']):
                l = u - gdict['s']
                
                return gdictpoint(gdict,l)
        
        return None, None
    
    def draw(self, endpoints=False):
        X = []
        Y = []
        for l in np.arange(0, self.length, 0.5):
            x, y =self.getpoint(l)
            if x is not None:
                X.append(x)
                Y.append(y)
        plt.plot(X, Y)
        if endpoints:
            plt.plot(X[0], Y[0], "x")
            plt.plot(X[-1], Y[-1], "o")
        
    def isin(self, x, y):
        ans = []

        w = prdist2(x, y, self)

        if np.abs(w) > 50:
            return ans

        for lanesection in self.laneSections:
            ans += lanesection.isin(x, y)
        
        return ans

roadfs = []
for road in roads:
    r =  Roadf(road)
    
    roadfs.append(r)
    try:
        junctionsdict[r.junction].append(r)
    except:
        junctionsdict[r.junction] = [r]

def findlane(x, y):
    isin = []
    rnum = -1
    lnum = -1
    for road in roadfs:
        isin += road.isin(x, y)
    return isin

