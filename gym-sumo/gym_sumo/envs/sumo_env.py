import gym
# from gym import error, spaces, utils
from gym import spaces, error
# from gym.utils import seeding

import os
import sys

try:
    import traci
    from traci import constants as tc
    from traci.exceptions import TraCIException
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))

import numpy as np

from gym_sumo.envs._graph import Graph
from gym_sumo.envs._util import randomTuple

# action
STRAIGHT = 0
UTARN = 1
LEFT = 2
PARLEFT = 3
RIGHT = 4
PARRIGHT = 5
STOP = 6

# standard length for adding car
SPOS = 10.5

DIRECTION = ['s', 'T', 'l', 'L', 'r', 'R']

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

    def __init__(self, isgraph=True, area='nishiwaseda', carnum=100,
                 mode='gui', step_length=0.01, simulation_end=100):
        super(SumoEnv, self).__init__()
        sumoConfig = 'sumo_configs/' + area
        sumoMap = os.path.join(os.path.dirname(__file__), sumoConfig)
        self.__netpath = os.path.join(sumoMap, 'osm.net.xml')

        self.metadata = {'render.modes': ['human']}
        self.__sumocfg = os.path.join(sumoMap, 'osm.sumocfg')
        self.__carnum = carnum
        self.__mode = mode
        self.__vehIDList = {}
        self.__removeIDList = []
        self.__stepLength = step_length
        self.__simulation_end = simulation_end
        self.__isgraph = isgraph
        self.__graph = Graph(self.__netpath)
        initGraph = self.__graph.getGraph()

        if not self.__isgraph:
            self.initiallizeGraph()

        self.routeEdge = []
        self.observation = self.reset()
        # 5action and accel, brake
        self.action_space = []
        for i in range(carnum):
            self.action_space.append(
                spaces.Tuple(
                    (spaces.Discrete(7),
                     spaces.Box(low=np.array([-1.0], dtype=np.float32),
                                high=np.array([1.0], dtype=np.float32),
                                dtype=np.float32))))
        self.observation_space = spaces.Box(low=1,
                                            high=initGraph.num_edges,
                                            shape=(np.shape(self.observation)))
        self.reward_range = [(-5, 10) for i in range(carnum)]

    def _isDone(self):
        removeList = self.__removeIDList
        removeNum = len(removeList)
        step = traci.simulation.getTime()
        vList = traci.vehicle.getIDList()
        vnum = len(vList)
        if self.__carnum <= removeNum:
            return True
        elif step >= self.__simulation_end:
            return True
        elif vnum <= 0:
            return True
        return False

    def step(self, action):
        # determine next step action
        isTakeAction = [True] * self.__carnum
        for i, act in enumerate(action):
            vehID = list(self.__vehIDList)[i]
            isTake = self.__takeAction(vehID, act)
            isTakeAction[i] = isTake
        traci.simulationStep()
        isDone = self._isDone()
        observation = self.observation()
        reward = self.reward(isTakeAction)
        return observation, reward, isDone, {}

    def reset(self):
        mode = self.__mode
        # reset
        self.close()
        self.__graph = Graph(self.__netpath)
        self.routeEdge = []
        # traci start & init simulate
        self.initSimulator(mode=mode, step_length=self.__stepLength)
        self.__vehIDList.clear()
        self.__removeIDList.clear()
        self.addCar(self.__carnum)
        observation = self.observation()
        traci.simulationStep()
        return observation

    def render(self, mode='human'):
        if self.__isgraph:
            self.__graph.check_graph(self.observation)
        print(self.observation)

    def close(self):
        try:
            traci.close()
            sys.stdout.flush()
        except traci.exceptions.FatalTraCIError as ci:
            print(ci)

    def reward(self, isTakeAction):
        v_list = traci.vehicle.getIDList()
        collisionList = traci.simulation.getCollidingVehiclesIDList()
        rewardList = [0] * self.__carnum
        removedList = self.__removeIDList
        # existNum = self.__carnum - len(removedList)
        for i, vehID in enumerate(self.__vehIDList):
            reward = 0
            if vehID in v_list and vehID not in removedList:
                if isTakeAction[i]:
                    curLength = traci.vehicle.getDistance(vehID)
                    totalLen = self.getRouteLength(vehID)
                    # if this car has never moved, reward=0
                    tmp = 0
                    # if this car has already moved,
                    # reward=ratio of route length
                    reward_max = self.reward_range[i][1]
                    if totalLen > 0:
                        tmp = int((curLength * reward_max) / totalLen)
                    reward += tmp
                    speed = traci.vehicle.getSpeed(vehID)
                    if speed <= 0.0:
                        reward -= tmp
                    if vehID in collisionList:
                        reward -= 2
                else:
                    if vehID not in self.__removeIDList:
                        reward -= 2
                        traci.vehicle.remove(vehID, tc.REMOVE_VAPORIZED)
                        self.__removeIDList.append(vehID)

        return rewardList

    def observation(self):
        observation = None
        vehIDList = traci.vehicle.getIDList()
        # print(self.__graph.getGraph(), length)
        if self.__isgraph:
            originGraph = self.__graph.getGraph()
            for vehID in vehIDList:
                v_info = {}
                v_info['pos'] = list(traci.vehicle.getPosition(vehID))
                v_info['speed'] = [traci.vehicle.getSpeed(vehID)]
                self.__graph.addNode(vehID,
                                     traci.vehicle.getRoadID(vehID), v_info)
            # print(self.__graph.graph, "obs")
            observation = self.__graph.getGraph()
            self.__graph.setGraph(originGraph)
        else:
            observation = [self.nodelist, self.edgelist]
            v_info = []
            v_info.append(list(traci.vehicle.getPosition(self.vehID)))
            v_info.append(traci.vehicle.getSpeed(self.vehID))
            v_info.append(traci.vehicle.getRoadID(self.vehID))
            observation.append(v_info)
        return observation

    def updateAddCar(self):
        v_list = traci.vehicle.getIDList()
        for i, vehID in enumerate(self.__vehIDList):
            if vehID not in self.__removeIDList and vehID not in v_list:
                routeID = self.__vehIDList[vehID]
                route = traci.route.getEdges(routeID)
                laneID = route[0] + '_0'
                laneVehIDList = traci.lane.getLastStepVehicleIDs(laneID)
                minPos = traci.lane.getLength(laneID)
                for vehID in laneVehIDList:
                    lanePos = traci.vehicle.getLanePosition(vehID)
                    minPos = min(lanePos, minPos)
                if minPos >= SPOS:
                    self.insertCar(vehID, routeID, route[0])

    def initSimulator(self, mode='gui', routing_alg='dijkstra',
                      step_length=0.01):
        sumocfg = self.__sumocfg
        self.__stepLength = step_length
        sumoCommand = mode
        if mode == 'gui':
            sumoCommand = 'sumo-gui'
        elif mode == 'cui':
            sumoCommand = 'sumo'
        else:
            raise AttributeError("not supported mode!!")
        sumoCmd = [sumoCommand, '-c', sumocfg, '--routing-algorithm',
                   routing_alg, '--step-length', str(step_length),
                   '--collision.action', 'remove',
                   '--collision.check-junctions', str(True),
                   '--tls.all-off', str(True)]
        traci.start(sumoCmd, numRetries=100)

    def addCar(self, carnum):
        for i in range(carnum):
            vehID = 'veh{}'.format(i)
            routeID = 'route{}'.format(i)
            self.__vehIDList[vehID] = routeID
            fromEdgeID, _ = self.generateRoute(routeID)
            start = self.__graph.getEdgeIndex(fromEdgeID)
            if start not in self.routeEdge:
                self.routeEdge.append(start)
                self.insertCar(vehID, routeID, fromEdgeID)

    def insertCar(self, vehID, routeID, startEdgeID=None):
        if startEdgeID is None:
            startEdgeID = traci.route.getEdges(routeID)[0]
        # register car
        traci.vehicle.add(vehID, routeID)
        # set speed mode
        traci.vehicle.setSpeedMode(vehID, 0)
        laneID = startEdgeID + '_0'
        length = min(traci.vehicle.getLength(vehID),
                     traci.lane.getLength(laneID))
        # insert car instantly
        traci.vehicle.moveTo(vehID, laneID, length)
        traci.vehicle.setSpeed(vehID, 0.0)

    def generateRoute(self, routeID):
        num_edge = self.__graph.getNum('edge_normal') - 1
        fromEdgeID = None
        toEdgeID = None
        for i in range(10):
            # print(i, "test", routeID, "route")
            edges = randomTuple(0, num_edge, 2, self.routeEdge)
            fromEdgeID = self.__graph.getEdgeID(edges[0])
            toEdgeID = self.__graph.getEdgeID(edges[1])
            try:
                route = traci.simulation.findRoute(fromEdgeID, toEdgeID)
                if len(route.edges) > 0:
                    traci.route.add(routeID, route.edges)
                    break
            except TraCIException as tr:
                print(tr)
        return fromEdgeID, toEdgeID

    def initiallizeGraph(self):
        self.nodelist = list(self.__graph.getGraph().pos.numpy())
        self.edgelist = self.__graph.getEdges()

    def turn(self, vehID, direction, curEdgeID=None):
        if curEdgeID is None:
            curEdgeID = traci.vehicle.getRoadID(vehID)
        nextEdgeID = self.__graph.getNextInfoTo(curEdgeID, direction)
        if nextEdgeID is None:
            return False
        else:
            # move indirectly
            route = traci.vehicle.getRoute(vehID)
            targetEdgeID = route[len(route) - 1]
            newRoute = traci.simulation.findRoute(nextEdgeID, targetEdgeID)
            newEdgeList = [curEdgeID]
            newEdgeList[len(newEdgeList):len(newRoute.edges)] = newRoute.edges
            if len(newRoute.edges) == 0:
                print(vehID, "empty")
                newEdgeList.append(nextEdgeID)
            try:
                traci.vehicle.setRoute(vehID, newEdgeList)
            except TraCIException as tr:
                print(tr)
                return False
        return True

    def __takeAction(self, vehID, action):
        isTake = False
        v_list = traci.vehicle.getIDList()
        if vehID not in v_list:
            return True
        removedList = self.__removeIDList
        if vehID in removedList:
            return False
        curSpeed = traci.vehicle.getSpeed(vehID)
        futureSpeed = curSpeed + action[1][0]
        isJunction = self.isJunction(vehID)
        if action[0] == STOP:
            futureSpeed = 0.0
            isTake = True
        else:
            direction = DIRECTION[action[0]]
            if self.couldTurn(vehID, futureSpeed, direction):
                curEdgeID = None
                if isJunction:
                    routeIndex = traci.vehicle.getRouteIndex(vehID)
                    route = traci.vehicle.getRoute(vehID)
                    curEdgeID = route[routeIndex]
                if self.turn(vehID, direction, curEdgeID):
                    isTake = True
            else:
                if isJunction:
                    routeIndex = traci.vehicle.getRouteIndex(vehID)
                    route = traci.vehicle.getRoute(vehID)
                    if routeIndex + 1 < len(route):
                        toEdgeID = route[routeIndex + 1]
                        preEdgeID = route[routeIndex]
                        direct = self.__graph.getnextInfoDirect(
                            curEdgeID=preEdgeID, toEdgeID=toEdgeID)
                        if direct == direction:
                            isTake = True
                else:
                    if direction == DIRECTION[STRAIGHT]:
                        isTake = True
        traci.vehicle.setSpeed(vehID, futureSpeed)
        return isTake

    def couldTurn(self, vehID, futureSpeed, direction=None):
        curLanePos = traci.vehicle.getLanePosition(vehID)
        curLaneID = traci.vehicle.getLaneID(vehID)
        roadLength = traci.lane.getLength(curLaneID)
        if self.isJunction(vehID):
            # 基本的に交差点にすでにいる場合は今から向きを変えるのは不可能という意味でreject
            route = traci.vehicle.getRoute(vehID)
            distance = traci.vehicle.getDistance(vehID)
            if distance > roadLength:
                return False
            # 初期位置なので、この交差点もはじめのedgeの一部とみなす。
            routeIndex = traci.vehicle.getRouteIndex(vehID)
            laneID = route[routeIndex]
            roadLength += traci.lane.getLength(laneID)
        futureLanePos = curLanePos + futureSpeed * self.__stepLength
        if futureLanePos > roadLength:
            return True
        return False

    def isJunction(self, vehID):
        curEdgeID = traci.vehicle.getRoadID(vehID)
        if self.__graph.getEdgeIndex(curEdgeID) == -1:
            return True
        return False

    def getRouteLength(self, vehID):
        route = traci.vehicle.getRoute(vehID)
        length = 0
        num = len(route)
        for i, edgeID in enumerate(route):
            laneID = edgeID + '_0'
            length += traci.lane.getLength(laneID)
            if i < num - 1:
                nextEdgeID = route[i + 1]
                viaLaneID = self.__graph.getNextInfoVia(
                    edgeID, toEdgeID=nextEdgeID)
                length += traci.lane.getLength(viaLaneID)
        return length
