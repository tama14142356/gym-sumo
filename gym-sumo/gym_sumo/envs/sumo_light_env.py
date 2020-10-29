from IPython import embed
import gym
import tempfile
# from gym import error, spaces, utils
from gym import spaces, error
from gym.utils import seeding
from PIL import Image

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

from .sumo_base_env import SumoBaseEnv as BaseEnv

# action
NO_OP = 0
UTARN = 1
LEFT = 2
PAR_LEFT = 3
RIGHT = 4
PAR_RIGHT = 5

# flag
DIRECT_FLAG = 5

# accel
POS_LARGE = 6
POS_SMALL = 7
NEG_LARGE = 8
NEG_SMALL = 9

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
ACCEL = [1.0, 0.2, -1.0, -0.2]

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


class SumoLightEnv(BaseEnv):

    def __init__(self, isgraph=True, area=0, carnum=100, mode='gui',
                 step_length=0.01, simulation_end=100, seed=None):
        super().__init__(isgraph, area, carnum, mode, step_length, simulation_end, seed)
        # 6action and accel, brake
        self.action_space = spaces.Discrete(10)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, dtype=np.float, shape=((3 + (VISIBLE_NUM * 3)), ))

    def step(self, action):
        # determine next step action
        vehID = list(self._vehID_list)[0]
        is_take = self._take_action(vehID, action)
        if self._mode == 'gui':
            self.screenshot_and_simulation_step()
        else:
            traci.simulationStep()
        observation = self._observation(vehID)

        # calculate reward
        reward = 0.0
        collision_list = traci.simulation.getCollidingVehiclesIDList()
        acheived_list = traci.simulation.getArrivedIDList()
        removed_list = self._removed_vehID_list
        if vehID not in removed_list:
            if vehID in acheived_list:
                self._removed_vehID_list.append(vehID)
                reward = 1.0
            elif vehID in collision_list or not is_take:
                traci.vehicle.remove(vehID, tc.REMOVE_TELEPORT)
                self._removed_vehID_list.append(vehID)
                reward = -1.0

        isdone = self._is_done(vehID)
        return observation, reward, isdone, {}

    def reset(self):
        vehID = list(self._vehID_list)[0]
        self._reposition_car()
        self._removed_vehID_list.clear()
        if self._mode == 'gui':
            viewID = traci.gui.DEFAULT_VIEW
            traci.gui.trackVehicle(viewID, vehID)
            # zoom = traci.gui.getZoom()
            traci.gui.setZoom(viewID, 5000)
        observation = self._observation(vehID)
        return observation

    def _observation(self, vehID):
        pos = list(traci.vehicle.getPosition(vehID))
        if pos[0] == tc.INVALID_DOUBLE_VALUE or pos[1] == tc.INVALID_DOUBLE_VALUE:
            pos[0], pos[1] = -np.inf, -np.inf
        posList = [pos]
        neighborList = self._sumo_util._get_neighbor_list(
            posList, VISIBLE_RANGE)
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
        observation = np.array(v_obs, dtype=np.float)
        return observation

    def _take_action(self, vehID, action):
        is_take = False
        v_list = traci.vehicle.getIDList()
        removed_list = self._removed_vehID_list
        if vehID not in v_list or vehID in removed_list:
            return is_take
        cur_speed = traci.vehicle.getSpeed(vehID)
        cur_accel = traci.vehicle.getAcceleration(vehID) * self._step_length
        future_speed = cur_speed + cur_accel
        is_junction = self._sumo_util._is_junction(vehID)
        if action == NO_OP:
            is_take = True
        elif action <= DIRECT_FLAG:
            direction = DIRECTION[action]
            route_index = traci.vehicle.getRouteIndex(vehID)
            route = traci.vehicle.getRoute(vehID)
            cur_edgeID = route[route_index]
            if self._sumo_util._could_turn(vehID, future_speed, direction):
                if self._sumo_util.turn(vehID, direction, cur_edgeID):
                    is_take = True
            else:
                if is_junction and route_index + 1 < len(route):
                    to_edgeID = route[route_index + 1]
                    direct = self._graph.getNextInfoDirect(
                        curEdgeID=cur_edgeID, toEdgeID=to_edgeID)
                    is_take = direct == direction
        else:
            accel_rate = ACCEL[action - DIRECT_FLAG - 1]
            accel = traci.vehicle.getAccel(vehID) * abs(accel_rate)
            decel = traci.vehicle.getDecel(vehID) * abs(accel_rate) * -1.0
            future_accel = accel if accel_rate >= 0 else decel
            future_accel *= self._step_length
            future_speed = cur_speed + future_accel
            is_take = True
            if future_speed < 0.0:
                future_speed = 0.0
                is_take = False
            traci.vehicle.setSpeed(vehID, future_speed)
        return is_take
