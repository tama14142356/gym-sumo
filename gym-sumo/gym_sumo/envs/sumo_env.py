import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os, sys
import traci

import random
import numpy as np

from gym_sumo.envs._graph import Graph

# action
LEFT = 0
RIGHT = 1
STRAIGHT = 2
UTARN = 3
PARLEFT = 4
PARRIGHT = 5
STOP = 6


DIRECTION = ['l', 'r', 's', 'T', 'L', 'R']

VEH_SIGNALS = {
    0: "VEH_SIGNAL_BLINKER_RIGHT",
    1: "VEH_SIGNAL_BLINKER_LEFT",
    2: "VEH_SIGNAL_BLINKER_EMERGENCY",
    3: "VEH_SIGNAL_BRAKELIGHT",
    4: "VEH_SIGNAL_FRONTLIGHT",
    5: "VEH_SIGNAL_FOGLIGHT",
    6: "VEH_SIGNAL_HIGHBEAM",
    7: "VEH_SIGNAL_BACKDRIVE",
    8: "VEH_SIGNAL_WIPER",
    9: "VEH_SIGNAL_DOOR_OPEN_LEFT",
    10: "VEH_SIGNAL_DOOR_OPEN_RIGHT",
    11: "VEH_SIGNAL_EMERGENCY_BLUE",
    12: "VEH_SIGNAL_EMERGENCY_RED",
    13: "VEH_SIGNAL_EMERGENCY_YELLOW",
}

class SumoEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    sumoMap = os.path.join(os.path.dirname(__file__), 'sumo_configs/waseda')
    def __init__(self, isgraph = True, filepath = sumoMap, carnum = 100, mode='gui', simulation_end=36000):
        self.mappath = filepath
        self.netpath = filepath + "/osm.net.xml"
        self.sumocfg = self.mappath + "/osm.sumocfg"
        self.carnum = carnum
        self.isgraph = isgraph
        self.previousAccel = [0.0] * carnum
        self.currentSpeed = [0.0] * carnum
        self.graph = Graph(self.netpath)
        self.initGraph = self.graph.graph
        if not self.isgraph:
            self.initGraph()
        self.routeEdge = {}
        self.observation = self.reset()
        # 5action and accel, brake
        self.action_space = []
        for i in range(carnum):
            self.action_space.append(spaces.Tuple((spaces.Discrete(5), spaces.Box(low=np.array([-1.0], dtype=np.float32), high=np.array([1.0], dtype=np.float32), dtype=np.float32))))
        self.observation_space = spaces.Box(low=1, high=self.graph.graph.num_edges, shape=(np.shape(self.observation)))
        # self.routes = {}
        self.step = 0

    def step(self, action):
        traci.simulationStep()
        # calculate current step action
        v_list = traci.vehicle.getIDList()
        for v in v_list:
            index = int(v[3:])
            prespeed = traci.vehicle.getSpeed(v)
            curspeed = prespeed + self.previousAccel[index]
            traci.vehicle.setSpeed(v, curspeed)
            self.currentSpeed[index] = curspeed
        # determine next step action
        self.takeAction(action)


    def reset(self, mode='gui'):
        # traci start
        self.initSimulator()
        # init simulate
        traci.simulationStep()
        observation = self.observation()
        return observation

    def render(self, mode='human'):
        pass

    def close(self):
        traci.close()
        sys.stdout.flush()

    def takeAction(self, action):
        for i, act in enumerate(action):
            vehID = 'veh{}'.format(i)
            maxaccel = traci.vehicle.getAccel(vehID)
            maxdeccel = traci.vehicle.getDecel(vehID)
            futureAccel = 0.0
            if act[1] >= 0:
                # accelreation
                futureAccel = maxaccel * act[1]
            else:
                # decceleration
                futureAccel = maxdeccel * act[1]
            curspeed = self.currentSpeed[i]
            futureSpeed = curspeed + futureAccel
            self.previousAccel[i] = futureAccel
            if futureSpeed < 0:
                futureSpeed = 0.0
                self.previousAccel[i] = -curspeed
            curpos, curLaneID = self.curLanPos(vehID)
            curEdgeID = traci.lane.getEdgeID(curLaneID)
            curEdgeIndex = self.graph.normalEdgeIDdict[curEdgeID]
            if act[0] == STOP:
                self.previousAccel[i] = -curspeed
            else:
                dir = DIRECTION[act[0]]
                if self.isStreet(vehId, futureSpeed, curpos, curLaneID):
                    if act[0] != STRAIGHT:
                        # give penalty
                        print("nothing!")
                else:
                    nextEdgeIndexList = self.graph.normalEdgeConnection[curEdgeIndex]['0']
                    if dir not in nextEdgeIndexList:
                        # give penalty
                        print("nothing!")
                    else:
                        nextEdgeIndex = nextEdgeIndexList[dir][0]
                        nextEdgeID = self.graph.normalEdgeIDList[nextEdgeIndex]
                        nextLaneID = nextEdgeID + '_{}'.format(0)
                        nodeIndex = g.graph.edge_index[0][nextEdgeIndex]
                        nodeID = self.graph.nodeIDList[nodeIndex]
                        x, y = traci.junction.getPosition(nodeID)
                        traci.vehicle.moveToXY(v, nextEdgeID, 0, x, y, angle=angle, keepRoute=4)
    
    # whether the car is on street or on junction from now to next step
    def isStreet(self, vehID, futureSpeed, curpos=None, curLaneID=None):
        index = int(vehID[3:])
        if curpos is None or curLaneID is None:
            curpos, curlaneID = self.curLanPos(vehID)
        length = traci.lane.getLength(curlaneID)
        futurepos = curpos + futureSpeed
        if curpos >= length:
            return True
        if futurepos > length:
            return False
        return True
    
    def curLanPos(self, vehID):
        index = int(vehID[3:])
        curedgeID = traci.vehicle.getRoadID(vehID)
        curlaneID = traci.vehicle.getLaneID(vehID)
        roadLength = traci.lane.getLength(curlaneID)
        pos = traci.vehicle.getLanePosition(vehID)
        curspeed = self.currentSpeed[index]
        curLanePosition = pos + curspeed
        if curLanePosition <= roadLength:
            return curLanePosition, curlaneID
        curLanePosition = curLanePosition - roadLength
        route = traci.vehicle.getRoute(vehID)
        isStart = False
        for edge in route:
            if isStart:
                laneID = edge + '_{}'.format(0)
                length = traci.lane.getLength(laneID)
                if curLanePosition <= length:
                    return curLanePosition, laneID
                curLanePosition = curLanePosition - length
            if edge == curedgeID:
                isStart = True
        return 0.0

    def observation(self):
        observation = None
        vehIDlist = traci.vehicle.getIDList()
        length = len(vehIDlist)
        # print(self.graph.graph, length)
        if self.isgraph:
            originGraph = self.graph.graph
            for vehID in vehIDlist:
                traci.vehicle.getRoadID(vehID)
                v_info = {}
                v_info['pos'] = list(map(float, traci.vehicle.getPosition(vehID)))
                v_info['speed'] = [float(traci.vehicle.getSpeed(vehID))]
                self.graph.addNode(vehID, traci.vehicle.getRoadID(vehID), v_info)
            # print(self.graph.graph, "obs")
            observation = self.graph.graph
            self.graph.graph = originGraph
        else:
            observation = [self.nodelist, self.edgelist]
            v_info = []
            v_info.append(list(traci.vehicle.getPosition(self.vehID)))
            v_info.append(traci.vehicle.getSpeed(self.vehID))
            v_info.append(traci.vehicle.getRoadID(self.vehID))
            observation.append(v_info)
        return observation

    def initSimulator(self, mode='gui', routing_alg='dijkstra', step_length=0.01):
        sumocfg = self.sumocfg
        self.step = 0
        if mode == 'gui':
            sumoCmd = ['sumo-gui', '-c', sumocfg, '--routing-algorithm', routing_alg, '--step-length', str(step_length)]
        elif mode == 'cui':
            sumoCmd = ['sumo', '-c', sumocfg, '--routing-algorithm', routing_alg, '--step-length', str(step_length)]
        else :
            raise AttributeError("not supported mode!!")
        traci.start(sumoCmd)
        # self.generateRoute('route1')
        # traci.vehicle.add(self.vehID, 'route1')
        # self.addCar(self.carnum - 1)
        self.addCar(self.carnum)
    
    def isNewRoute(self, fromedgeindex, toedgeindex):
        if fromedgeindex not in self.routeEdge:
            self.routeEdge[fromedgeindex] = []
            self.routeEdge[fromedgeindex].append(toedgeindex)
            return True
        elif toedgeindex not in self.routeEdge[fromedgeindex]:
            self.routeEdge[fromedgeindex].append(toedgeindex)
            return True
        return False
        
    def generateRoute(self, routeID):
        i = 0
        while True:
            # print(i, "test", routeID, "route")
            while True:
                ns = self.random_int(0, self.graph.num['edge_normal'], 2)
                fromedge = ns[0]
                toedge = ns[1]
                if self.isNewRoute(fromedge, toedge):
                    break
            try:
                fromEdgeID = self.graph.normalEdgeIDList[fromedge]
                toEdgeID = self.graph.normalEdgeIDList[toedge]
                route = traci.simulation.findRoute(fromEdgeID, toEdgeID)
                traci.route.add(routeID, route.edges)
                return fromEdgeID, toEdgeID
            except traci.exceptions.TraCIException as routeErr:
                i+=1
                # print(i, route.edges)
                if i == 10:
                    print(routeErr)
                    break
        return None, None

    def addCar(self, carnum):
        for i in range(carnum):
            vehID = 'veh{}'.format(i)
            routeID = 'route{}'.format(i)
            fromedge, _ = self.generateRoute(routeID)
            traci.vehicle.add(vehID, routeID)
            #set speed mode
            traci.vehicle.setSpeedMode(vehID, 0)
            # insert car instantly
            laneID = fromedge + '_{}'.format(0)
            length = traci.vehicle.getLength(vehID)
            length = min(length, traci.lane.getLength(laneID))
            start = self.graph.normalEdgeIDdict[fromedge]
            if len(self.routeEdge[start]) <= 1:
                traci.vehicle.moveTo(vehID, laneID, length)
                traci.vehicle.setSpeed(vehID, 0.0)

    def random_int(self, a, b, num):
        ns = []
        while len(ns) < num:
            n = random.randrange(a, b)
            if n not in ns:
                ns.append(n)
        return ns
    
    def initGraph(self):
        self.nodelist = list(self.graph.graph.pos.numpy())
        self.edgelist = self.graph.getEdges()
