import gym
# from gym import error, spaces, utils
from gym import spaces, error
# from gym.utils import seeding

import os
import sys

try:
    import traci
    from traci import constants as tc
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))

import numpy as np

from gym_sumo.envs._graph import Graph
from gym_sumo.envs._myvehicle import CurVehicle, DIRECTION, STOP, STRAIGHT
from gym_sumo.envs._util import randomTuple


class SumoEnv(gym.Env):

    def __init__(self, isgraph=True, area='nishiwaseda', carnum=100,
                 mode='gui', step_length=0.01, simulation_end=100):
        sumoConfig = 'sumo_configs/' + area
        sumoMap = os.path.join(os.path.dirname(__file__), sumoConfig)
        netpath = os.path.join(sumoMap, 'osm.net.xml')

        self.metadata = {'render.modes': ['human']}
        self.sumocfg = os.path.join(sumoMap, 'osm.sumocfg')
        self.__carnum = carnum
        self.__vehIDList = []
        self.__stepLength = step_length
        self.__simulation_end = simulation_end
        self.isgraph = isgraph
        self.graph = Graph(netpath)
        self.initGraph = self.graph.getGraph()

        if not self.isgraph:
            self.initiallizeGraph()

        self.routeEdge = {}
        self.observation = self.reset(mode=mode)
        if 'curVehicle' not in dir(self):
            self.curVehicle = CurVehicle(self.__vehIDList,
                                         self.__stepLength, self.graph)
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
                                            high=self.initGraph.num_edges,
                                            shape=(np.shape(self.observation)))
        self.reward_range = [(-5, 10) for i in range(carnum)]

    def _isDone(self, vehID):
        removeList = self.curVehicle.getRemoveList()
        step = traci.simulation.getTime()
        if vehID in removeList:
            return True
        elif step >= self.__simulation_end:
            return True
        return False

    def step(self, action):
        # calculate vehicle info in current step
        v_list = traci.vehicle.getIDList()
        self.curVehicle.calcCurInfo()
        for v in v_list:
            curSpeed = self.curVehicle.getCurSpeed(v)
            traci.vehicle.setSpeed(v, curSpeed)
        # determine next step action
        isTakeAction = []
        isDone = []
        for i, act in enumerate(action):
            isTake = self.__takeAction(i, act)
            isTakeAction.append(isTake)
            vehID = self.curVehicle.getVehID(i)
            tmp = self._isDone(vehID)
            isDone.append(tmp)
        traci.simulationStep()
        self.curVehicle.setInValid()
        observation = self.observation()
        reward = self.reward(isTakeAction, action)
        return observation, reward, isDone, {}

    def reset(self, mode='gui'):
        # reset
        self.close()
        # traci start & init simulate
        self.initSimulator(mode, step_length=self.__stepLength)
        observation = self.observation()
        self.curVehicle.setInValid()
        traci.simulationStep()
        return observation

    def render(self, mode='human'):
        if self.isgraph:
            self.graph.check_graph(self.observation)
        print(self.observation)

    def close(self):
        try:
            traci.close()
            sys.stdout.flush()
        except traci.exceptions.FatalTraCIError as ci:
            print(ci)

    def reward(self, isTakeAction, action):
        v_list = traci.vehicle.getIDList()
        collisionList = traci.simulation.getCollidingVehiclesIDList()
        rewardList = [0] * self.__carnum
        removedList = self.curVehicle.getRemoveList()
        for i, vehID in enumerate(self.__vehIDList):
            reward = 0
            if vehID in v_list and vehID not in removedList:
                if isTakeAction[i]:
                    curLength = traci.vehicle.getDistance(vehID)
                    + self.curVehicle.getCurLanePos(vehID)
                    totalLen = self.curVehicle.getRouteLength(vehID)
                    # if this car has never moved, reward=0
                    tmp = 0
                    # if this car has already moved,
                    # reward=ratio of route length
                    if totalLen > 0:
                        tmp = int((curLength * 10) / totalLen)
                    reward += tmp

                    if vehID in collisionList:
                        reward -= 2
                else:
                    reward -= 2
                    traci.vehicle.remove(vehID, tc.REMOVE_VAPORIZED)
                    self.curVehicle.removeVeh(vehID)

        return rewardList

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
                                         self.curVehicle.getCurPos(vehID)))
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
                   routing_alg, '--step-length', str(step_length),
                   '--collision.action', 'remove',
                   '--collision.check-junctions', str(True),
                   '--tls.all-off', str(True)]
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

    def generateRoute(self, routeID):
        num_edge = self.graph.getNum('edge_normal')
        fromEdgeID = None
        toEdgeID = None
        for i in range(10):
            # print(i, "test", routeID, "route")
            edges = (randomTuple(0, num_edge, 2, self.routeEdge))
            fromEdgeID = self.graph.getEdgeID(edges[0])
            toEdgeID = self.graph.getEdgeID(edges[1])
            route = traci.simulation.findRoute(fromEdgeID, toEdgeID)
            if len(route.edges) > 0:
                traci.route.add(routeID, route.edges)
                break
        return fromEdgeID, toEdgeID

    def initiallizeGraph(self):
        self.nodelist = list(self.graph.getGraph().pos.numpy())
        self.edgelist = self.graph.getEdges()

    def turn(self, vehID, curEdgeID, direction):
        nextEdgeID = self.graph.getNextInfoTo(curEdgeID, None, direction)
        if nextEdgeID is None:
            return False
        else:
            # move indirectly
            targetEdgeID = self.curVehicle.getTarget(vehID)
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
        removedList = self.curVehicle.getRemoveList()
        if vehID in removedList:
            return False
        curSpeed = self.curVehicle.getCurSpeed(vehID, vehIndex=vehIndex)
        futureSpeed = self.curVehicle.changeSpeed(vehID, action[1],
                                                  vehIndex=vehIndex)
        curPos, curLaneID = self.curVehicle.calcCurLanePos(vehID,
                                                           vehIndex=vehIndex)
        if curLaneID is None or len(curLaneID) <= 0:
            return False
        curEdgeID = traci.lane.getEdgeID(curLaneID)
        if action[0] == STOP:
            self.curVehicle.setCurAccel(vehID, -curSpeed, vehIndex=vehIndex)
            isTake = True
        else:
            direction = DIRECTION[action[0]]
            if self.curVehicle.couldTurn(futureSpeed, vehID,
                                         vehIndex=vehIndex):
                if self.turn(vehID, curEdgeID, direction):
                    isTake = True
            else:
                if self.curVehicle.isJunction(vehID, vehIndex, curEdgeID):
                    routeIndex = self.curVehicle.getCurRouteIndex(
                        vehID, vehIndex=vehIndex)
                    route = traci.vehicle.getRoute(vehID)
                    if routeIndex + 1 < len(route):
                        toEdgeID = route[routeIndex + 1]
                        preEdgeID = route[routeIndex]
                        direct = self.graph.getnextInfoDirect(
                            curEdgeID=preEdgeID, toEdgeID=toEdgeID)
                        if direct == direction:
                            isTake = True
                else:
                    if direction == DIRECTION[STRAIGHT]:
                        isTake = True
        return isTake
