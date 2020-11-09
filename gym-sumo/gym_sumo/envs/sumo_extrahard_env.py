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
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(
            e
        )
    )

import numpy as np

from ._graph import Graph
from ._myvehicle import CurVehicle, DIRECTION, STOP, STRAIGHT
from ._util import random_tuple


class SumoExtraHardEnv(gym.Env):
    def __init__(
        self,
        isgraph=True,
        area="nishiwaseda",
        carnum=100,
        mode="gui",
        step_length=0.01,
        simulation_end=100,
    ):
        super().__init__()
        sumoConfig = "sumo_configs/" + area
        sumoMap = os.path.join(os.path.dirname(__file__), sumoConfig)
        self.__netpath = os.path.join(sumoMap, "osm.net.xml")

        self.metadata = {"render.modes": ["human"]}
        self.__sumocfg = os.path.join(sumoMap, "osm.sumocfg")
        self.__carnum = carnum
        self.__mode = mode
        self.__vehIDList = {}
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
                    (
                        spaces.Discrete(7),
                        spaces.Box(
                            low=np.array([-1.0], dtype=np.float32),
                            high=np.array([1.0], dtype=np.float32),
                            dtype=np.float32,
                        ),
                    )
                )
            )
        self.observation_space = spaces.Box(
            low=1, high=initGraph.num_edges, shape=(np.shape(self.observation))
        )
        self.reward_range = [(-5, 10) for i in range(carnum)]

    def _isDone(self, vehID):
        removeList = self.curVehicle.getRemoveList()
        if vehID in removeList:
            return True
        return False

    def _isDones(self):
        removeList = self.curVehicle.getRemoveList()
        removeNum = len(removeList)
        step = traci.simulation.getTime()
        vList = traci.vehicle.getIDList()
        vnum = len(vList)
        ans = [False] * self.__carnum
        if self.__carnum <= removeNum:
            return [True] * self.__carnum
        elif step >= self.__simulation_end:
            return [True] * self.__carnum
        elif vnum <= 0:
            return [True] * self.__carnum
        else:
            for i, vehID in enumerate(vList):
                ans[i] = self._isDone(vehID)
        return ans

    def step(self, action):
        # calculate vehicle info in current step
        v_list = traci.vehicle.getIDList()
        self.curVehicle.calcCurInfo()
        for v in v_list:
            curSpeed = self.curVehicle.getCurSpeed(v)
            traci.vehicle.setSpeed(v, curSpeed)
        # determine next step action
        isTakeAction = [True] * self.__carnum
        for i, act in enumerate(action):
            isTake = self.__takeAction(i, act)
            isTakeAction[i] = isTake
            # vehID = self.curVehicle.getVehID(i)
            # tmp = self._isDone(vehID)
            # isDone[i] = tmp
        traci.simulationStep()
        isDone = self._isDones()
        self.curVehicle.setInValid()
        observation = self.observation()
        reward = self.reward(isTakeAction, action)
        return observation, reward, isDone, {}

    def reset(self):
        mode = self.__mode
        # reset
        self.close()
        self.__graph = Graph(self.__netpath)
        self.routeEdge = []
        # traci start & init simulate
        self.initSimulator(mode=mode, step_length=self.__stepLength)
        self.curVehicle = CurVehicle(self.__vehIDList, self.__stepLength, self.__graph)
        observation = self.observation()
        self.curVehicle.setInValid()
        traci.simulationStep()
        return observation

    def render(self, mode="human"):
        if self.__isgraph:
            self.__graph.check_graph(self.observation)
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
        # existNum = self.__carnum - len(removedList)
        for i, vehID in enumerate(self.__vehIDList):
            reward = 0
            if vehID in v_list and vehID not in removedList:
                if isTakeAction[i]:
                    curLength = traci.vehicle.getDistance(vehID)
                    +self.curVehicle.getCurLanePos(vehID)
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
                    if vehID not in self.curVehicle.getRemoveList():
                        reward -= 2
                        traci.vehicle.remove(vehID, tc.REMOVE_VAPORIZED)
                        self.curVehicle.removeVeh(vehID)

        return rewardList

    def observation(self):
        observation = None
        vehIDList = traci.vehicle.getIDList()
        # print(self.__graph.getGraph(), length)
        if self.__isgraph:
            self.curVehicle.calcCurInfo()
            originGraph = self.__graph.getGraph()
            for vehID in vehIDList:
                v_info = {}
                v_info["pos"] = list(map(float, self.curVehicle.getCurPos(vehID)))
                v_info["speed"] = [float(self.curVehicle.getCurSpeed(vehID))]
                self.__graph.addNode(vehID, traci.vehicle.getRoadID(vehID), v_info)
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

    def initSimulator(self, mode="gui", routing_alg="dijkstra", step_length=0.01):
        sumocfg = self.__sumocfg
        self.__stepLength = step_length
        sumoCommand = mode
        if mode == "gui":
            sumoCommand = "sumo-gui"
        elif mode == "cui":
            sumoCommand = "sumo"
        else:
            raise AttributeError("not supported mode!!")
        sumoCmd = [
            sumoCommand,
            "-c",
            sumocfg,
            "--routing-algorithm",
            routing_alg,
            "--step-length",
            str(step_length),
            "--collision.action",
            "remove",
            "--collision.check-junctions",
            str(True),
            "--tls.all-off",
            str(True),
        ]
        traci.start(sumoCmd, numRetries=100)
        self.__vehIDList.clear()
        self.addCar(self.__carnum)

    def addCar(self, carnum):
        for i in range(carnum):
            vehID = "veh{}".format(i)
            routeID = "route{}".format(i)
            self.__vehIDList[vehID] = routeID
            fromEdgeID, _ = self.generateRoute(routeID)
            start = self.__graph.getEdgeIndex(fromEdgeID)
            if start not in self.routeEdge:
                self.routeEdge.append(start)
                # register car
                traci.vehicle.add(vehID, routeID)
                # set speed mode
                traci.vehicle.setSpeedMode(vehID, 0)
                laneID = fromEdgeID + "_0"
                length = min(
                    traci.vehicle.getLength(vehID), traci.lane.getLength(laneID)
                )
                # insert car instantly
                traci.vehicle.moveTo(vehID, laneID, length)
                traci.vehicle.setSpeed(vehID, 0.0)

    def generateRoute(self, routeID):
        num_edge = self.__graph.getNum("edge_normal") - 1
        fromEdgeID = None
        toEdgeID = None
        for i in range(10):
            # print(i, "test", routeID, "route")
            edges = random_tuple(0, num_edge, 2, self.routeEdge)
            fromEdgeID = self.__graph.getEdgeID(edges[0])
            toEdgeID = self.__graph.getEdgeID(edges[1])
            try:
                route = traci.simulation.findRoute(fromEdgeID, toEdgeID)
            except TraCIException as tr:
                print(tr)
            if len(route.edges) > 0:
                traci.route.add(routeID, route.edges)
                break
        return fromEdgeID, toEdgeID

    def initiallizeGraph(self):
        self.nodelist = list(self.__graph.getGraph().pos.numpy())
        self.edgelist = self.__graph.getEdges()

    def turn(self, vehID, curEdgeID, direction):
        nextEdgeID = self.__graph.getNextInfoTo(curEdgeID, direction)
        if nextEdgeID is None:
            return False
        else:
            # move indirectly
            targetEdgeID = self.curVehicle.getTarget(vehID)
            newRoute = traci.simulation.findRoute(nextEdgeID, targetEdgeID)
            # print(newRoute, "newroute")
            newEdgeList = [curEdgeID]
            newEdgeList[len(newEdgeList) : len(newRoute.edges)] = newRoute.edges
            if len(newRoute.edges) == 0:
                print(vehID, "empty")
                newEdgeList.append(nextEdgeID)
            try:
                traci.vehicle.setRoute(vehID, newEdgeList)
            except TraCIException as tr:
                print(tr)
                return False
        return True

    def __takeAction(self, vehIndex, action):
        isTake = False
        isExist = self.curVehicle.isExist(vehIndex=vehIndex)
        if not isExist:
            return True
        vehID = self.curVehicle.getVehID(vehIndex)
        removedList = self.curVehicle.getRemoveList()
        if vehID in removedList:
            return False
        curSpeed = self.curVehicle.getCurSpeed(vehID, vehIndex=vehIndex)
        futureSpeed = self.curVehicle.changeSpeed(
            vehID, action[1][0], vehIndex=vehIndex
        )
        curPos, curLaneID = self.curVehicle.calcCurLanePos(vehID, vehIndex=vehIndex)
        if curLaneID is None or len(curLaneID) <= 0:
            return False
        curEdgeID = traci.lane.getEdgeID(curLaneID)
        if action[0] == STOP:
            self.curVehicle.setCurAccel(
                vehID, -curSpeed * self.__stepLength, vehIndex=vehIndex
            )
            isTake = True
        else:
            direction = DIRECTION[action[0]]
            if self.curVehicle.couldTurn(futureSpeed, vehID, vehIndex=vehIndex):
                if self.turn(vehID, curEdgeID, direction):
                    isTake = True
            else:
                if self.curVehicle.isJunction(vehID, vehIndex, curEdgeID):
                    routeIndex = self.curVehicle.getCurRouteIndex(
                        vehID, vehIndex=vehIndex
                    )
                    route = traci.vehicle.getRoute(vehID)
                    if routeIndex + 1 < len(route):
                        toEdgeID = route[routeIndex + 1]
                        preEdgeID = route[routeIndex]
                        direct = self.__graph.getnextInfoDirect(
                            curEdgeID=preEdgeID, toEdgeID=toEdgeID
                        )
                        if direct == direction:
                            isTake = True
                else:
                    if direction == DIRECTION[STRAIGHT]:
                        isTake = True
        return isTake
