import math
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
import xml.etree.ElementTree as elemTree
from queue import PriorityQueue

from geomodule import *
from roadmodule import *


def neighbors(lane, sameRoad=True, direction=1): # direction = 1 or -1, 
    ns = []
    road_id = lane.road_id
    section_id = lane.section_id
    lane_id = lane.lane_id
    road = roadsdict[road_id]
    
    predecessor_type = road.predecessor[0]
    predecessor_id = road.predecessor[1]
    successor_type = road.successor[0]
    successor_id = road.successor[1]
    pcontact = road.predecessor[2]
    scontact = road.successor[2]
    predecessors = []
    successors = []
    sidelanes = []

    if section_id == 0:
        if predecessor_type=='junction':
            for jroad in junctionsdict[predecessor_id]:
                if pcontact == 'end' and ((jroad.successor[0] == 'junction' and road.junction == jroad.successor[1]) or (jroad.successor[0] == 'road' and jroad.successor[1] == road_id)):
                    for jlane in jroad.laneSections[-1].lanes.items():
                        if jlane[1].successor_id == lane_id:
                            predecessors.append(jlane[1])

                elif pcontact == 'start' and ((jroad.predecessor[0] == 'junction' and road.junction == jroad.predecessor[1]) or (jroad.predecessor[0] == 'road' and jroad.predecessor[1] == road_id)):
                    for jlane in jroad.laneSections[0].lanes.items():
                        if jlane[1].predecessor_id == lane_id:
                            predecessors.append(jlane[1])
   
        elif predecessor_type=='road':
            proad = roadsdict[predecessor_id]
            if pcontact == 'end':
                try:
                    predecessors.append(proad.laneSections[-1].lanes[lane.predecessor_id])
                except:
                    print("broken link error")

            elif pcontact == 'start':
                try:   
                    predecessors.append(jroad.laneSections[0].lanes[lane.predecessor_id])
                except:
                    print("broken link error")

    if section_id == len(road.laneSections) - 1:
        if successor_type=='junction':
            for jroad in junctionsdict[successor_id]:
                if scontact == 'end' and ((jroad.successor[0] == 'junction' and road.junction == jroad.successor[1]) or (jroad.successor[0] == 'road' and jroad.successor[1] == road_id)):
                    for jlane in jroad.laneSections[-1].lanes.items():
                        if jlane[1].successor_id == lane_id:
                            successors.append(jlane[1])

                elif scontact == 'start' and ((jroad.predecessor[0] == 'junction' and road.junction == jroad.predecessor[1]) or (jroad.predecessor[0] == 'road' and jroad.predecessor[1] == road_id)):
                    for jlane in jroad.laneSections[0].lanes.items():
                        if jlane[1].predecessor_id == lane_id:
                            successors.append(jlane[1])

        elif successor_type=='road':
            sroad = roadsdict[successor_id]
            if scontact == 'end':
                try:
                    successors.append(sroad.laneSections[-1].lanes[lane.successor_id])
                except:
                    print("broken link error")

            elif scontact == 'start':
                try:   
                    successors.append(jroad.laneSections[0].lanes[lane.successor_id])
                except:
                    print("broken link error")

    if section_id > 0:
        predecessors.append(road.laneSections[section_id-1].lanes[lane.predecessor_id])

    if section_id < len(road.laneSections) - 1:
        successors.append(road.laneSections[section_id+1].lanes[lane.successor_id])

    for clane in road.laneSections[section_id].lanes.items():
        if clane[0] == lane_id or clane[1].lane_type != 'driving' or np.sign(clane[0]) != np.sign(lane_id):
            continue
        if np.abs(clane[0] - lane_id) == 1:
            sidelanes.append(clane[1])
    
    #ns += predecessors + successors + sidelanes

    if direction * lane_id < 0:
        ns += successors
    else:
        ns += predecessors

    if sameRoad:
        ns += sidelanes

    return ns

def lanedist(lane1, lane2):
    road_id1 = lane1.road_id
    lane_id1 = lane1.lane_id
    road_id2 = lane2.road_id
    lane_id2 = lane2.lane_id
    
    if road_id1 == road_id2:
        return (np.abs(lane1.swidth - lane2.ewidth) + np.abs(lane2.swidth - lane2.ewidth))/2.0
    
    road1 = roadsdict[road_id1]
    road2 = roadsdict[road_id2]
    
    return (road1.length + road2.length)/2

def getNode(node_id):
    try:
        return roadsdict[node_id[0]].lanes[node_id[1]]
    except:
        return None


def Dijkstra(sisin, eisin):
    distances = defaultdict(lambda: 2_147_483_647)
    parents = {}
    pq = PriorityQueue()
    for slain in sisin:
        pq.put((0, (slain.road_id, slain.lane_id), ((slain.road_id, slain.lane_id)))) # (dist, node_id, par_id)
    
    while (not pq.empty()):
        top = pq.get()
        dist = top[0]
        node_id = top[1]
        node_lane = getNode(node_id)
        par_id = top[2]
        
        if distances[node_id] <= dist:
            continue
        distances[node_id] = dist
        parents[node_id] = par_id
        
        if node_lane in eisin:
            traj = [node_id]
            while node_id != par_id:
                node_id = par_id
                par_id = parents[node_id]
                traj.append(node_id)

            return dist, traj
        
        for nlane in neighbors(node_lane):
            ndist = dist + lanedist(nlane, node_lane)
            if distances[(nlane.road_id, nlane.lane_id)] > ndist:
                pq.put((ndist, (nlane.road_id, nlane.lane_id), node_id))
        
    return -1, []