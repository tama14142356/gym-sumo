import gym
# from gym import error, spaces, utils
from gym import spaces, error
from gym.utils import seeding

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
from gym_sumo.envs._util import randomTuple, vector_decomposition, get_degree
from gym_sumo.envs._util import in_many_shape
from gym_sumo.envs._util import get_rectangle_positions, in_rect, get_base_angle

# action
STRAIGHT = 0
UTARN = 1
LEFT = 2
PARLEFT = 3
RIGHT = 4
PARRIGHT = 5
STOP = 6

STANDARD_SPEED = 100.0 / 9.0

# standard length for adding car
SPOS = 10.5

# visible range of the car (circle with that radius to that range) [m]
VISIBLE_RANGE = 10

# the number of recognizable car per a car
VISIBLE_NUM = 5

# category variable(one hot encoding)
# for vehicle
VEHICLE_CATEGORY = [0, 1]
VEH_CATEGORY = 1
NONE_VEH_CATEGORY = 0

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


class SumoSimpleEnv(gym.Env):

    def __init__(self, isgraph=True, area='nishiwaseda', carnum=100, mode='cui',
                 step_length=0.01, simulation_end=100, seed=None):
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
        self.__seed = self.seed(seed)
        self.__graph = Graph(self.__netpath)
        self.routeEdge = []
        self.observation = self.reset()
        self.vehID = list(self.__vehIDList)[0]
        # steer, accel, brake
        low = np.array([-np.inf, -1.0, 0.0])
        high = np.array([np.inf, 1.0, 1.0])
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        # self.observation_space = spaces.Box(
        #     low=-np.inf, high=np.inf, dtype=np.float,
        #     shape=(carnum, 2, 3 + (VISIBLE_NUM * 2)))
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, dtype=np.float, shape=((3 + (VISIBLE_NUM * 3)), ))
        # self.reward_range = (-1, 1)
        # super(SumoLightEnv, self).__init__()

    def _isDone(self, vehID):
        removeList = self.__removeIDList
        return vehID in removeList

    def step(self, action):
        # determine next step action
        self._update_add_car()
        vehID = list(self.__vehIDList)[0]
        reward_state = (self._take_action(vehID, action))
        traci.simulationStep()
        self.observation = self._observation(vehID)
        reward = self.reward(vehID, reward_state)
        isDone = self._isDone(vehID)
        return self.observation, reward, isDone, {}

    def reset(self):
        mode = self.__mode
        # reset
        self.close()
        self.routeEdge = []
        # traci start & init simulate
        self._init_simulator(mode=mode, step_length=self.__stepLength)
        self.__vehIDList.clear()
        self.__removeIDList.clear()
        self._add_car(self.__carnum)
        vehID = list(self.__vehIDList)[0]
        self.observation = self._observation(vehID)
        traci.simulationStep()
        return self.observation

    def render(self, mode='human'):
        print(self.observation)

    def close(self):
        try:
            traci.close()
            sys.stdout.flush()
        except traci.exceptions.FatalTraCIError as ci:
            print(ci)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reward(self, vehID, reward_state):
        v_list = traci.vehicle.getIDList()
        collisionList = traci.simulation.getCollidingVehiclesIDList()
        reward = 0
        if vehID in v_list:
            if vehID in collisionList:
                self.__removeIDList.append(vehID)
                reward -= 1
            else:
                if reward_state[0]:
                    reward += 0.1
                else:
                    reward -= 0.1
                if reward_state[1]:
                    reward += 0.2
                else:
                    reward -= 0.2
                # todo
                # ここで道路に沿って行動しているならプラスの報酬を与えたい
                # 外れたなら、マイナスの報酬を与えたい
        return reward

    def _observation(self, vehID):
        pos = traci.vehicle.getPosition(vehID)
        if pos[0] == tc.INVALID_DOUBLE_VALUE or pos[1] == tc.INVALID_DOUBLE_VALUE:
            pos[0], pos[1] = -np.inf, -np.inf
        posList = [pos]
        neighborList = self._get_neighbor_list(posList, VISIBLE_RANGE)
        level = 3 + (VISIBLE_NUM * 3)
        for neighbor in neighborList:
            v_obs = [0.0] * level
            num = VISIBLE_NUM * 3
            speed = traci.vehicle.getSpeed(vehID)
            pos = traci.vehicle.getPosition(vehID)
            v_x, v_y = pos
            v_obs[0], v_obs[1], v_obs[2] = speed, v_x, v_y
            num_neighbor = len(neighbor)
            for j in range(0, num, 3):
                index = j // 3
                if num_neighbor > index:
                    veh_index, neighbor_pos = neighbor[index]
                    r_x, r_y = neighbor_pos
                    category = float(VEHICLE_CATEGORY[VEH_CATEGORY])
                else:
                    r_x, r_y = 0.0, 0.0
                    category = float(VEHICLE_CATEGORY[NONE_VEH_CATEGORY])
                v_obs[j + 3], v_obs[j + 4], v_obs[j + 5] = r_x, r_y, category
        self.observation = np.array(v_obs, dtype=np.float)
        return self.observation

    def _update_add_car(self):
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
                    self._insert_car(vehID, routeID, route[0])

    def _init_simulator(self, mode='gui', routing_alg='dijkstra', step_length=0.01):
        sumocfg = self.__sumocfg
        self.__stepLength = step_length
        sumoCommand = mode
        if mode == 'gui':
            sumoCommand = 'sumo-gui'
        elif mode == 'cui':
            sumoCommand = 'sumo'
        else:
            raise AttributeError("not supported mode!!")
        sumoCmd = [sumoCommand, '-c', sumocfg, '--routing-algorithm', routing_alg,
                   '--step-length', str(step_length), '--collision.action', 'remove',
                   '--collision.check-junctions', 'true', '--tls.all-off', 'true',
                   '--no-warnings', 'true',
                   '--no-step-log', 'true', '--duration-log.disable', 'true']
        traci.start(sumoCmd, numRetries=100)

    def _add_car(self, carnum):
        for i in range(carnum):
            vehID = 'veh{}'.format(i)
            routeID = 'route{}'.format(i)
            fromEdgeID, toEdgeID = self._generate_route(routeID)
            veh_element = {'route': routeID,
                           'start': fromEdgeID, 'goal': toEdgeID}
            self.__vehIDList[vehID] = veh_element
            start = self.__graph.getEdgeIndex(fromEdgeID)
            if start not in self.routeEdge:
                self.routeEdge.append(start)
                self._insert_car(vehID, routeID, fromEdgeID)

    def _insert_car(self, vehID, routeID, startEdgeID=None):
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

    def _generate_route(self, routeID):
        num_edge = self.__graph.getNum('edge_normal') - 1
        fromEdgeID = None
        toEdgeID = None
        for i in range(10):
            # print(i, "test", routeID, "route")
            edges = randomTuple(0, num_edge, 2, self.routeEdge, self.np_random)
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

    def _take_action(self, vehID, action):
        steer, accel, brake = action
        delta_t = traci.simulation.getDeltaT()
        max_accel = traci.vehicle.getAccel(vehID)
        max_decel = traci.vehicle.getDecel(vehID)
        cur_speed = traci.vehicle.getSpeed(vehID)
        cur_accel = max_accel * abs(accel) * delta_t
        cur_decel = max_decel * abs(brake) * delta_t
        future_accel = cur_accel - cur_decel
        future_speed = cur_speed + future_accel
        traci.vehicle.setSpeed(vehID, future_speed)
        cur_x, cur_y = traci.vehicle.getPosition(vehID)
        delta_x, delta_y = vector_decomposition(future_speed * delta_t, steer)
        next_x, next_y = cur_x + delta_x, cur_y + delta_y
        edgeID = traci.vehicle.getRoadID(vehID)
        lane = traci.vehicle.getLaneIndex(vehID)
        laneID = traci.vehicle.getLaneID(vehID)
        veh_len = traci.vehicle.getLength(vehID)
        angle = traci.vehicle.getAngle(vehID)
        rear_x, rear_y = vector_decomposition(veh_len, angle)
        rear_pos = [cur_x - rear_x, cur_y - rear_y]
        if len(laneID) <= 0:
            in_road = False
            match_degree = False
        else:
            lane_pos = traci.lane.getShape(laneID)
            width = traci.lane.getWidth(laneID)
            pos_num = len(lane_pos)
            if pos_num <= 2:
                # todo
                # 道路から外れているかどうかの判定をする。
                # これにより、報酬を決定したいのと、目的地についてるのかも決めたい。
                degree = get_degree(lane_pos[0], lane_pos[1])
                road_pos = get_rectangle_positions(
                    lane_pos[0], lane_pos[1], width)
                in_road = (in_rect(road_pos, [cur_x, cur_y])
                           or in_rect(road_pos, rear_pos))
                match_degree = abs(degree - get_base_angle(angle)) < 0.01
            else:
                match_degree = True
                in_road = (in_many_shape(lane_pos, [cur_x, cur_y])
                           or in_many_shape(lane_pos, rear_pos))
        traci.vehicle.moveToXY(vehID, edgeID, lane, next_x, next_y, steer, 2)
        return in_road, match_degree

    def _is_exist(self, pos, r, target):
        posx, posy = pos

        def distance(targetpos):
            x, y = targetpos
            return np.sqrt((posx - x)**2 + (posy - y)**2)

        dis = distance(target)
        relative_pos = [x - y for (x, y) in zip(target, pos)]
        return dis <= r, relative_pos

    def _get_neighbor(self, posList, central_index, visible_range):
        neighborList = []
        centralpos = posList[central_index]
        for i, pos in enumerate(posList):
            if i == central_index:
                continue
            x, y = pos
            if x == -np.inf or y == -np.inf:
                isexist, dis = False, np.inf
            else:
                isexist, dis = self._is_exist(centralpos, visible_range, pos)
            if isexist:
                tmp = [i, dis]
                neighborList.append(tmp)
        return neighborList

    def _get_neighbor_list(self, posList, visible_range):
        neighborList = []
        for index, pos in enumerate(posList):
            x, y = pos
            if x != -np.inf and y != -np.inf:
                tmp = self._get_neighbor(posList, index, visible_range)
            else:
                tmp = []
            neighborList.append(tmp)
        return neighborList
