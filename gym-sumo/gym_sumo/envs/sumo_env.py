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
BACK = 3
STOP = 4

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
        # if action[0] ==
        self.step += 1

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

    # def takeAction(self, action):
    #     for i, act in enumerate(action):
    #         vehID = 'veh{}'.format(i)
    #         if act[0] == LEFT:
    #         elif act[0] == RIGHT:
    #         elif act[0] == STRAIGHT:
    #         elif act[0] == BACK:
    #         elif act[0] == STOP:
    #             traci.vehicle.setSpeed(vehID, 0)
    #         traci.vehicle.setAccel(vehID, act[1])
    
    # def isStreet(self, vehID):
    #     curedgeID = traci.vehicle.getRoadID(vehID)
    #     curlaneID = traci.vehicle.getLaneID(vehID)
    #     roadLength = traci.lane.getLength(curlaneID)
    #     pos = traci.vehicle.getLanePosition(vehID)
    #     speed = traci.vehicle.getSpeed(vehID)
    #     curLanePosition = pos + speed

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
                route = traci.simulation.findRoute(self.graph.normalEdgeIDList[fromedge], self.graph.normalEdgeIDList[toedge])
                traci.route.add(routeID, route.edges)
                break
            except traci.exceptions.TraCIException as routeErr:
                i+=1
                # print(i, route.edges)
                if i == 10:
                    print(routeErr)
                    break

    def addCar(self, carnum):
        for i in range(carnum):
            vehID = 'veh{}'.format(i)
            routeID = 'route{}'.format(i)
            self.generateRoute(routeID)
            traci.vehicle.add(vehID, routeID)
            #set speed mode
            traci.vehicle.setSpeedMode(vehID, 0)

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
