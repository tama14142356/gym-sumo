from IPython import embed
# from gym import error, spaces, utils
from gym import spaces, error

try:
    import traci
    from traci import constants as tc
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))

import numpy as np

from .sumo_base_env import SumoBaseEnv as BaseEnv
from gym_sumo.envs._util import vector_decomposition, get_degree, in_many_shape
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


class SumoSimpleEnv(BaseEnv):

    def __init__(self, isgraph=True, area=0, carnum=100, mode='gui', step_length=0.01,
                 simulation_end=100, seed=None):
        super().__init__(isgraph, area, carnum, mode, step_length, simulation_end, seed)
        self.vehID = list(self._vehID_list)[0]
        # steer, accel, brake
        low = np.array([-np.inf, -1.0, 0.0])
        high = np.array([np.inf, 1.0, 1.0])
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, dtype=np.float, shape=((3 + (VISIBLE_NUM * 3)), ))

    def step(self, action):
        # determine next step action
        vehID = list(self._vehID_list)[0]
        reward_state = (self._take_action(vehID, action))
        if self._mode == 'gui':
            self.screenshot_and_simulation_step()
        else:
            traci.simulationStep()
        observation = self._observation(vehID)
        reward = self.reward(vehID, reward_state)
        done = self._is_done(vehID)
        return observation, reward, done, {}

    def reset(self):
        cur_time = traci.simulation.getTime()
        self._reset_simulate_time(cur_time)
        self._update_add_car()
        self._removed_vehID_list.clear()
        vehID = list(self._vehID_list)[0]
        if self._mode == 'gui':
            viewID = traci.gui.DEFAULT_VIEW
            traci.gui.trackVehicle(viewID, vehID)
            # zoom = traci.gui.getZoom()
            traci.gui.setZoom(viewID, 5000)
        observation = self._observation(vehID)
        if self._mode == 'gui':
            self.screenshot_and_simulation_step()
        else:
            traci.simulationStep()
        return observation

    def reward(self, vehID, reward_state):
        v_list = traci.vehicle.getIDList()
        collision_list = traci.simulation.getCollidingVehiclesIDList()
        reward = 0
        if vehID in v_list:
            if vehID in collision_list:
                self._removed_vehID_list.append(vehID)
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
        pos = list(traci.vehicle.getPosition(vehID))
        if pos[0] == tc.INVALID_DOUBLE_VALUE or pos[1] == tc.INVALID_DOUBLE_VALUE:
            pos[0], pos[1] = -np.inf, -np.inf
        pos_list = [pos]
        neighbor_list = self._sumo_util._get_neighbor_list(
            pos_list, VISIBLE_RANGE)
        level = 3 + (VISIBLE_NUM * 3)
        for neighbor in neighbor_list:
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
        observation = np.array(v_obs, dtype=np.float)
        return observation

    def _remove_car_all(self):
        # v_list = traci.vehicle.getIDList()
        v_list = list(self._vehID_list)
        for vehID in v_list:
            traci.vehicle.remove(vehID)

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
