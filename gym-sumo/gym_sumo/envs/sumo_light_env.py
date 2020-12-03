from .sumo_base_env import SumoBaseEnv as BaseEnv
from .sumo_base_env import DIRECTION, INFO
from .sumo_base_env import STRAIGHT, UTURN, LEFT, PAR_LEFT, RIGHT, PAR_RIGHT
from ._util import vector_decomposition, flatten_list, get_base_vector

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
RENDER_TEXT = ["LARGE ACCEL", "SMALL ACCEL", "LARGE DECEL", "SMALL DECEL"]


class SumoLightEnv(BaseEnv):
    def __init__(
        self,
        isgraph=True,
        area=0,
        carnum=100,
        mode="gui",
        step_length=0.01,
        simulation_end=200,
        seed=None,
        label="default",
        debug_view=False,
    ):
        super().__init__(
            isgraph,
            area,
            carnum,
            mode,
            step_length,
            simulation_end,
            seed,
            label,
            debug_view,
        )
        # 6action and accel, brake
        self.action_text = self.action_text + RENDER_TEXT.copy()
        self.action_space = spaces.Discrete(10)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, dtype=np.float32, shape=(12,)
        )

    def step(self, action, vehID=""):
        # determine next step action, affect environment
        if len(vehID) <= 0:
            vehID = list(self._vehID_list)[0]
        pos = self.traci_connect.vehicle.getPosition(vehID)
        goal_pos = self._goal[vehID].get("pos", [0.0, 0.0])
        pre_to_goal_length = self._graph._calc_distance(pos, goal_pos)
        is_take = self._take_action(vehID, action)
        if self._mode == "gui":
            self.screenshot_and_simulation_step(action, vehID, is_take)
        else:
            self.traci_connect.simulationStep()

        removed_list = self._removed_vehID_list
        acheived_list = self.traci_connect.simulation.getArrivedIDList()

        info = INFO.copy()
        info["is_take"] = is_take
        info["is_removed"] = vehID in removed_list
        info["cur_step"] = self._get_cur_step()
        info["cur_sm_step"] = self.traci_connect.simulation.getTime()
        info["is_arrived"] = vehID in acheived_list and not info["is_removed"]
        info["needs_reset"] = info["is_removed"] or info["is_arrived"]
        info["goal"] = self._vehID_list[vehID]["goal"]

        if not info["needs_reset"]:
            info["cur_lane"] = self.traci_connect.vehicle.getLaneID(vehID)
            info["cur_lane_pos"] = self.traci_connect.vehicle.getLanePosition(vehID)
            info["lane_len"] = self.traci_connect.lane.getLength(info["cur_lane"])
            info["speed"] = self.traci_connect.vehicle.getSpeed(vehID)
            info["pos"] = self.traci_connect.vehicle.getPosition(vehID)

        # calculate reward
        reward = 0.0
        if vehID not in removed_list:
            if vehID in acheived_list:
                self._removed_vehID_list.append(vehID)
                self._arrived_vehID_list.append(vehID)
                reward += 100.0
            else:
                # progress bonus
                to_goal_length = self._graph._calc_distance(
                    info["pos"], self._goal[vehID]["pos"]
                )
                # reward += info["speed"]
                dense_reward = pre_to_goal_length - to_goal_length
                reward += dense_reward
        done = self._is_done(vehID)
        observation = self._observation(vehID, done, info["is_arrived"])
        return observation, reward, done, info

    def reset(self):
        self._reposition_car()
        vehID = list(self._vehID_list)[0]
        if self._mode == "gui":
            viewID = self.traci_connect.gui.DEFAULT_VIEW
            # self.traci_connect.gui.trackVehicle(viewID, vehID)
            # zoom = self.traci_connect.gui.getZoom()
            self.traci_connect.gui.setZoom(viewID, 1000)
            self.screenshot_and_simulation_step()
            self._reset_simulate_time()
        observation = self._observation(vehID)
        return observation

    def _remove_observation(self, vehID):
        goal_pos = self._goal[vehID]["pos"]
        start_edgeID = self._vehID_list[vehID]["start"]
        start_pos = self._vehID_list[vehID].get("start_pos", [0.0, 0.0])
        relative_pos = list(np.array(goal_pos) - np.array(start_pos))
        start_vector, _ = self._get_vector_pos_edgeID(start_edgeID)
        turn_direction = [0.0] * (DIRECT_FLAG + 1)
        goal_vector = list(self._goal[vehID]["direct"])
        observation = [relative_pos, list(start_vector), turn_direction, goal_vector]
        return np.array(flatten_list(observation), dtype=np.float32)

    def _done_observation(self, vehID, is_arrived=True):
        if not is_arrived:
            return self._remove_observation(vehID)
        goal_vector = list(self._goal[vehID]["direct"])
        relative_goal_pos = list([0.0, 0.0])
        goal_edgeID = self._vehID_list[vehID]["goal"]
        directions = self._network.get_next_directions(goal_edgeID, 0)
        turn_direction = list(
            map(lambda direct: 1.0 if direct in directions else 0.0, DIRECTION)
        )
        observation = [relative_goal_pos, goal_vector, turn_direction, goal_vector]
        return np.array(flatten_list(observation), dtype=np.float32)

    def _observation(self, vehID, is_done=False, is_arrived=False):
        if is_done:
            return self._done_observation(vehID, is_arrived)
        pos = list(self.traci_connect.vehicle.getPosition(vehID))
        if pos[0] == tc.INVALID_DOUBLE_VALUE or pos[1] == tc.INVALID_DOUBLE_VALUE:
            pos[0], pos[1] = -np.inf, -np.inf
        goal_pos = self._goal[vehID]["pos"]
        relative_goal_pos = list(np.array(goal_pos) - np.array(pos))
        veh_len = self.traci_connect.vehicle.getLength(vehID)
        angle = self.traci_connect.vehicle.getAngle(vehID)
        vector = list(vector_decomposition(veh_len, angle))
        veh_vector = list(get_base_vector([0.0, 0.0], vector))
        could_reach, cur_laneID = self._sumo_util._could_reach_junction(vehID)
        directions = [self._sumo_util._get_direction_along_route(vehID)]
        if could_reach:
            cur_edgeID = self.traci_connect.lane.getEdgeID(cur_laneID)
            cur_lane_index = cur_laneID.replace(cur_edgeID, "")[1:]
            directions = self._network.get_next_directions(cur_edgeID, cur_lane_index)
        turn_direction = list(
            map(lambda direct: 1.0 if direct in directions else 0.0, DIRECTION)
        )
        goal_vector = list(self._goal[vehID]["direct"])
        observation = [relative_goal_pos, veh_vector, turn_direction, goal_vector]
        return np.array(flatten_list(observation), dtype=np.float32)

    def _take_action(self, vehID, action):
        is_take = False
        v_list = self.traci_connect.vehicle.getIDList()
        removed_list = self._removed_vehID_list
        if vehID not in v_list or vehID in removed_list:
            self._remove_car_if_necessary(vehID, (not is_take))
            return False
        future_speed = -1.0
        if action > DIRECT_FLAG:
            cur_speed = self.traci_connect.vehicle.getSpeed(vehID)
            accel_rate = ACCEL[action - DIRECT_FLAG - 1]
            accel = self.traci_connect.vehicle.getAccel(vehID) * abs(accel_rate)
            decel = self.traci_connect.vehicle.getDecel(vehID) * abs(accel_rate) * -1.0
            future_accel = accel if accel_rate >= 0 else decel
            future_accel *= self._step_length
            future_speed = cur_speed + future_accel
            is_take = future_speed >= 0.0
            future_speed = future_speed if is_take else 0.0
            self.traci_connect.vehicle.setSpeed(vehID, future_speed)
            if not is_take:
                self._remove_car_if_necessary(vehID, (not is_take))
                return False
        direction = DIRECTION[0] if action > DIRECT_FLAG else DIRECTION[action]
        could_turn, _, _ = self._sumo_util._could_turn(vehID, direction, future_speed)
        is_take = could_turn
        if could_turn:
            self._sumo_util.turn(vehID, direction, future_speed)
            self._reset_goal_element(vehID)
            self._reset_routeID(vehID)
        self._remove_car_if_necessary(vehID, (not is_take))
        return is_take
