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
from .sumo_base_env import DIRECTION
from ._util import vector_decomposition, flatten_list

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
            low=-np.inf, high=np.inf, dtype=np.float, shape=(12, ))

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
        goal_pos = self._goal[vehID]['pos']
        relative_goal_pos = list(goal_pos - np.array(pos))
        veh_len = traci.vehicle.getLength(vehID)
        angle = traci.vehicle.getAngle(vehID)
        veh_vector = list(vector_decomposition(veh_len, angle))
        could_reach, cur_laneID = self._sumo_util._could_reach_junction(vehID)
        turn_direction = [0.0] * (DIRECT_FLAG + 1)
        if could_reach:
            cur_edgeID = traci.lane.getEdgeID(cur_laneID)
            for i in range(DIRECT_FLAG):
                direction = DIRECTION[i]
                next_edgeID = self._graph.getNextInfoTo(cur_edgeID, direction)
                turn_direction[i] = 0.0 if next_edgeID is None else 1.0
        goal_vector = list(self._goal[vehID]['direct'])
        obs = [relative_goal_pos, veh_vector, turn_direction, goal_vector]
        obs_flat_list = flatten_list(obs)
        observation = np.array(obs_flat_list, dtype=np.float)
        return observation

    def _take_action(self, vehID, action):
        is_take = False
        v_list = traci.vehicle.getIDList()
        removed_list = self._removed_vehID_list
        if vehID not in v_list or vehID in removed_list:
            return is_take
        cur_speed = traci.vehicle.getSpeed(vehID)
        if action <= DIRECT_FLAG:
            direction = DIRECTION[action]
            could_turn, _ = self._sumo_util._could_turn(vehID, direction)
            if could_turn:
                self._sumo_util.turn(vehID, direction)
                is_take = True
                goal_edgeID = self._sumo_util.get_target(vehID)
                self._reset_goal_element(vehID, goal_edgeID)
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
