from .sumo_base_env import SumoBaseEnv as BaseEnv
from .sumo_base_env import DIRECTION
from .sumo_base_env import STRAIGHT, UTURN, LEFT, PAR_LEFT, RIGHT, PAR_RIGHT
from ._util import vector_decomposition, flatten_list

# from IPython import embed  # for debug
from gym import spaces
import numpy as np

from traci import constants as tc

# action
NO_OP = STRAIGHT

# flag
DIRECT_FLAG = max(STRAIGHT, UTURN, LEFT, PAR_LEFT, RIGHT, PAR_RIGHT)

# accel
POS_LARGE = 6
POS_SMALL = 7
NEG_LARGE = 8
NEG_SMALL = 9

ACCEL = [1.0, 0.2, -1.0, -0.2]


class SumoLightEnv(BaseEnv):
    def __init__(
        self,
        isgraph=True,
        area=0,
        carnum=100,
        mode="gui",
        step_length=0.01,
        simulation_end=100,
        seed=None,
        label="default",
    ):
        super().__init__(
            isgraph, area, carnum, mode, step_length, simulation_end, seed, label
        )
        # 6action and accel, brake
        self.action_space = spaces.Discrete(10)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, dtype=np.float32, shape=(12,)
        )

    def step(self, action, vehID=""):
        # determine next step action, affect environment
        if len(vehID) <= 0:
            vehID = list(self._vehID_list)[0]
        pre_driving_len = self.traci_connect.vehicle.getDistance(vehID)
        is_take = self._take_action(vehID, action)
        if self._mode == "gui":
            self.screenshot_and_simulation_step()
        else:
            self.traci_connect.simulationStep()
        observation = self._observation(vehID)
        cur_driving_len = self.traci_connect.vehicle.getDistance(vehID)

        # calculate reward
        reward = 0.0
        collision_list = self.traci_connect.simulation.getCollidingVehiclesIDList()
        acheived_list = self.traci_connect.simulation.getArrivedIDList()
        removed_list = self._removed_vehID_list
        if vehID not in removed_list:
            if vehID in acheived_list:
                self._removed_vehID_list.append(vehID)
                reward += 100.0
            elif vehID in collision_list or not is_take:
                self.traci_connect.vehicle.remove(vehID, tc.REMOVE_TELEPORT)
                self._removed_vehID_list.append(vehID)
                reward -= 1.0
            else:
                # survive bonus
                reward += 1.0
                # progress bonus
                progress = cur_driving_len - pre_driving_len
                total_length = self._sumo_util._get_route_length(vehID)
                if progress > 0:
                    reward += 0.1
                    reward += 0.0 if total_length <= 0 else progress / total_length

        isdone = self._is_done(vehID)
        return observation, reward, isdone, {}

    def reset(self):
        vehID = list(self._vehID_list)[0]
        self._reposition_car()
        self._removed_vehID_list.clear()
        if self._mode == "gui":
            viewID = self.traci_connect.gui.DEFAULT_VIEW
            self.traci_connect.gui.trackVehicle(viewID, vehID)
            # zoom = self.traci_connect.gui.getZoom()
            self.traci_connect.gui.setZoom(viewID, 5000)
        observation = self._observation(vehID)
        return observation

    def _observation(self, vehID):
        pos = list(self.traci_connect.vehicle.getPosition(vehID))
        if pos[0] == tc.INVALID_DOUBLE_VALUE or pos[1] == tc.INVALID_DOUBLE_VALUE:
            pos[0], pos[1] = -np.inf, -np.inf
        goal_pos = self._goal[vehID]["pos"]
        relative_goal_pos = list(goal_pos - np.array(pos))
        veh_len = self.traci_connect.vehicle.getLength(vehID)
        angle = self.traci_connect.vehicle.getAngle(vehID)
        veh_vector = list(vector_decomposition(veh_len, angle))
        could_reach, cur_laneID = self._sumo_util._could_reach_junction(vehID)
        turn_direction = [0.0] * (DIRECT_FLAG + 1)
        if could_reach:
            cur_edgeID = self.traci_connect.lane.getEdgeID(cur_laneID)
            for i in range(DIRECT_FLAG):
                direction = DIRECTION[i]
                next_edgeID = self._graph.getNextInfoTo(cur_edgeID, direction)
                turn_direction[i] = 0.0 if next_edgeID is None else 1.0
        goal_vector = list(self._goal[vehID]["direct"])
        obs = [relative_goal_pos, veh_vector, turn_direction, goal_vector]
        obs_flat_list = flatten_list(obs)
        observation = np.array(obs_flat_list, dtype=np.float32)
        return observation

    def _take_action(self, vehID, action):
        is_take = False
        v_list = self.traci_connect.vehicle.getIDList()
        removed_list = self._removed_vehID_list
        if vehID not in v_list or vehID in removed_list:
            return is_take
        cur_speed = self.traci_connect.vehicle.getSpeed(vehID)
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
            accel = self.traci_connect.vehicle.getAccel(vehID) * abs(accel_rate)
            decel = self.traci_connect.vehicle.getDecel(vehID) * abs(accel_rate) * -1.0
            future_accel = accel if accel_rate >= 0 else decel
            future_accel *= self._step_length
            future_speed = cur_speed + future_accel
            is_take = True
            if future_speed < 0.0:
                future_speed = 0.0
                is_take = False
            self.traci_connect.vehicle.setSpeed(vehID, future_speed)
        return is_take
