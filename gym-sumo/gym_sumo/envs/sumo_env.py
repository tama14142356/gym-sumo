import gym
# from gym import error, spaces, utils
from gym import spaces
# from gym.utils import seeding

import os
import sys
import traci

import numpy as np

from gym_sumo.envs._graph import Graph
from gym_sumo.envs._myvehicle import CurVehicle
from gym_sumo.envs._util import randomTuple

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

    def __init__(self, isgraph=True, filepath=sumoMap, carnum=100, mode='gui',
                 step_length=0.01):
        self.mappath = filepath
        self.netpath = filepath + "/osm.net.xml"
        self.sumocfg = self.mappath + "/osm.sumocfg"
        self.__carnum = carnum
        self.__vehIDList = []
        self.__stepLength = step_length
        self.isgraph = isgraph
        self.graph = Graph(self.netpath)
        self.initGraph = self.graph.getGraph()
        if not self.isgraph:
            self.initiallizeGraph()
        self.routeEdge = {}
        self.observation = self.reset()
        if 'curVehicle' not in dir(self):
            self.curVehicle = CurVehicle(self.__vehIDList,
                                         self.__stepLength, self.graph)
        # 5action and accel, brake
        self.action_space = []
        for i in range(carnum):
            self.action_space.append(
                spaces.Tuple(
                    (spaces.Discrete(5),
                     spaces.Box(low=np.array([-1.0], dtype=np.float32),
                                high=np.array([1.0], dtype=np.float32),
                                dtype=np.float32))))
        self.observation_space = spaces.Box(low=1,
                                            high=self.initGraph.num_edges,
                                            shape=(np.shape(self.observation)))
    
    def step(self, action):
        # calculate vehicle info in current step
        v_list = traci.vehicle.getIDList()
        self.curVehicle.calcCurInfo()
        for v in v_list:
            curSpeed = self.curVehicle.getCurSpeed(v)
            traci.vehicle.setSpeed(v, curSpeed)
        # determine next step action
        for i, act in enumerate(action):
            self.__takeAction(i, act)
        traci.simulationStep()
        self.curVehicle.setInValid()
        observation = self.observation()
        reward = self.reward()
        return observation, reward

    def reset(self, mode='gui'):
        # reset
        self.close()
        # traci start & init simulate
        self.initSimulator(mode, step_length=self.__stepLength)
        observation = self.observation()
        traci.simulationStep()
        self.curVehicle.setInValid()
        return observation

    def render(self, mode='human'):
        print(self.observation)

    def close(self):
        try:
            traci.close()
            sys.stdout.flush()
        except traci.exceptions.FatalTraCIError as ci:
            print(ci)
    
    def reward(self):
        return -1
    
    def observation(self):
        observation = None
        vehIDList = traci.vehicle.getIDList()
        # print(self.graph.getGraph(), length)
        if self.isgraph:
            if 'curVehicle' not in dir(self):
                self.curVehicle = CurVehicle(self.__vehIDList,
                                             self.__stepLength,
                                             self.graph)
            self.curVehicle.calcCurInfo()
            originGraph = self.graph.getGraph()
            for vehID in vehIDList:
                v_info = {}
                v_info['pos'] = list(map(float,
                                     self.curVehicle.getPosition(vehID)))
                v_info['speed'] = [float(self.curVehicle.getCurSpeed(vehID))]
                self.graph.addNode(vehID,
                                   traci.vehicle.getRoadID(vehID), v_info)
            # print(self.graph.graph, "obs")
            observation = self.graph.getGraph()
            self.graph.setGraph(originGraph)
        else:
            observation = [self.nodelist, self.edgelist]
            v_info = []
            v_info.append(list(traci.vehicle.getPosition(self.vehID)))
            v_info.append(traci.vehicle.getSpeed(self.vehID))
            v_info.append(traci.vehicle.getRoadID(self.vehID))
            observation.append(v_info)
        return observation
    
    def initSimulator(self, mode='gui', routing_alg='dijkstra',
                      step_length=0.01):
        sumocfg = self.sumocfg
        self.__stepLength = step_length
        sumoCommand = mode
        if mode == 'gui':
            sumoCommand = 'sumo-gui'
        elif mode == 'cui':
            sumoCommand = 'sumo'
        else:
            raise AttributeError("not supported mode!!")
        sumoCmd = [sumoCommand, '-c', sumocfg, '--routing-algorithm',
                   routing_alg, '--step-length', str(step_length)]
        traci.start(sumoCmd)
        self.addCar(self.__carnum)

    def addCar(self, carnum):
        for i in range(carnum):
            vehID = 'veh{}'.format(i)
            self.__vehIDList.append(vehID)
            routeID = 'route{}'.format(i)
            fromEdgeID, _ = self.generateRoute(routeID)
            traci.vehicle.add(vehID, routeID)
            # set speed mode
            traci.vehicle.setSpeedMode(vehID, 0)
            # insert car instantly
            laneID = fromEdgeID + '_{}'.format(0)
            length = min(traci.vehicle.getLength(vehID),
                         traci.lane.getLength(laneID))
            start = self.graph.getEdgeIndex(fromEdgeID)
            # initiallize the car
            if len(self.routeEdge[start]) <= 1:
                traci.vehicle.moveTo(vehID, laneID, length)
                traci.vehicle.setSpeed(vehID, 0.0)
    
    def initiallizeGraph(self):
        self.nodelist = list(self.graph.getGraph().pos.numpy())
        self.edgelist = self.graph.getEdges()

    def turn(self, vehID, curEdgeID, direction):
        nextEdgeID = self.graph.getNextInfoTo(curEdgeID, None, direction)
        if nextEdgeID is None:
            return False
        else:
            # move indirectly
            route = traci.vehicle.getRoute(vehID)
            targetEdgeID = route[len(route) - 1]
            newRoute = traci.simulation.findRoute(nextEdgeID, targetEdgeID)
            # print(newRoute, "newroute")
            newEdgeList = [curEdgeID]
            newEdgeList[len(newEdgeList):len(newRoute.edges)] = newRoute.edges
            if len(newRoute.edges) == 0:
                print(vehID, "empty")
                newEdgeList.append(nextEdgeID)
            traci.vehicle.setRoute(vehID, newEdgeList)
        return True

    def __takeAction(self, vehIndex, action):
        isTake = False
        vehID = self.curVehicle.getVehID(vehIndex)
        curSpeed = self.curVehicle.getCurSpeed(vehID)
        futureSpeed = self.curVehicle.changeSpeed(vehID, action[1])
        curPos, curLaneID = self.curVehicle.calcCurLanePos(vehID)
        if curLaneID is None or len(curLaneID) <= 0:
            return False
        curEdgeID = traci.lane.getEdgeID(curLaneID)
        if action[0] == STOP:
            self.curVehicle.setAccel(vehID, -curSpeed)
            isTake = True
        else:
            if self.curVehicle.couldTurn(futureSpeed, vehID):
                direction = DIRECTION[action[0]]
                if self.turn(vehID, curEdgeID, direction):
                    isTake = True
        return isTake
        
    def generateRoute(self, routeID):
        num_edge = self.graph.getNum('edge_normal')
        fromEdgeID = None
        toEdgeID = None
        for i in range(10):
            # print(i, "test", routeID, "route")
            fromEdgeIndex, toEdgeIndex = randomTuple(0, num_edge,
                                                     2, self.routeEdge)
            fromEdgeID = self.graph.getEdgeID(fromEdgeIndex)
            toEdgeID = self.graph.getEdgeID(toEdgeIndex)
            route = traci.simulation.findRoute(fromEdgeID, toEdgeID)
            if len(route.edges) > 0:
                traci.route.add(routeID, route.edges)
                break
        return fromEdgeID, toEdgeID
