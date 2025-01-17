from .sumo_base_env import SumoBaseEnv as BaseEnv
from ._util import vector_decomposition, flatten_list, get_base_vector, calc_distance
import gym_sumo.envs.constans as gc

# from IPython import embed  # for debug
from gym import spaces
import numpy as np

from traci import constants as tc


class SumoLightEnv(BaseEnv):
    def __init__(
        self,
        isgraph=False,
        area=0,
        carnum=100,
        mode="gui",
        step_length=0.01,
        simulation_end=200,
        seed=None,
        label="default",
        debug_view=False,
        is_random_route=True,
        max_length=gc.MAX_LENGTH,
        is_abs_length=False,
        is_length=False,
        is_road_num=False,
        is_end=True,
        is_start=False,
        road_freq=0,
        road_ratio=1.0,
        is_auto=False,
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
            is_random_route,
            max_length,
            is_abs_length,
            is_length,
            is_road_num,
            is_end,
            is_start,
        )
        # 6action and accel, brake
        self.action_text = gc.DIRECTION_TEXT + gc.ACCEL_TEXT
        self.action_space = spaces.Discrete(10)
        self.action_space.seed(seed)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, dtype=np.float32, shape=(12,)
        )
        self.is_init = True
        self.road_freq = road_freq
        self.road_ratio = 1.0 + road_ratio
        self._is_auto = is_auto

    def step(self, action, vehID=""):
        # determine next step action, affect environment
        if len(vehID) <= 0:
            vehID = list(self._vehID_list)[0]
        v_list = self.traci_connect.vehicle.getIDList()
        pos = (-1.0, -1.0)
        route_target_edgeID = ""
        if vehID in v_list:
            pos = self.traci_connect.vehicle.getPosition(vehID)
            route_target_edgeID = self._sumo_util.get_target(vehID=vehID)
        goal_pos = self._goal[vehID].get("pos", [0.0, 0.0])
        pre_to_goal_length = calc_distance(pos, goal_pos)
        is_take = True
        if not self._is_auto:
            is_take = self._take_action(vehID, action)
        if self._mode == "gui":
            self.screenshot_and_simulation_step(action)
        else:
            self.traci_connect.simulationStep()

        removed_list = self._removed_vehID_list
        acheived_list = self.traci_connect.simulation.getArrivedIDList()

        goal_edgeID = self._vehID_list[vehID]["goal"]
        is_arrived = vehID in acheived_list and route_target_edgeID == goal_edgeID

        info = gc.INFO.copy()
        info["is_take"] = is_take
        info["is_removed"] = vehID in removed_list or vehID in acheived_list
        info["cur_step"] = self._get_cur_step()
        info["cur_sm_step"] = self.traci_connect.simulation.getTime()
        info["is_arrived"] = is_arrived
        info["needs_reset"] = info["is_removed"]
        info["goal"] = goal_edgeID
        info["road_num"] = self.road_num

        if not info["needs_reset"]:
            info["cur_lane"] = self.traci_connect.vehicle.getLaneID(vehID)
            info["cur_lane_pos"] = self.traci_connect.vehicle.getLanePosition(vehID)
            info["lane_len"] = self.traci_connect.lane.getLength(info["cur_lane"])
            info["speed"] = self.traci_connect.vehicle.getSpeed(vehID)
            info["pos"] = self.traci_connect.vehicle.getPosition(vehID)

        # calculate reward
        reward = 0.0
        if vehID not in removed_list:
            if info["is_removed"]:
                self._removed_vehID_list.append(vehID)
                reward += 100.0 if is_arrived else 0.0
            else:
                # progress bonus
                to_goal_length = calc_distance(info["pos"], self._goal[vehID]["pos"])
                dense_reward = pre_to_goal_length - to_goal_length
                reward += dense_reward * 0.01
                if not is_take:
                    reward -= 0.01
        done = self._is_done(vehID) or info["needs_reset"]
        observation = self._observation(vehID, done, is_arrived)
        return observation, reward, done, info

    def reset(self):
        self.accumulate_steps = 0.0 if self.is_init else self.accumulate_steps
        if not self.is_init:
            road_freq = self.road_freq
            if road_freq is not None and road_freq > 0:
                pre_steps = self.accumulate_steps
                self.accumulate_steps += self._get_cur_step()
                if pre_steps // road_freq < self.accumulate_steps // road_freq:
                    self.road_num = self.road_num * self.road_ratio
                    self.max_length = self.max_length * self.road_ratio
            self._reposition_car()
        self.is_init = False
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
        turn_direction = [0.0] * (gc.DIRECT_FLAG + 1)
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
            map(lambda direct: 1.0 if direct in directions else 0.0, gc.DIRECTION)
        )
        observation = [relative_goal_pos, goal_vector, turn_direction, goal_vector]
        return np.array(flatten_list(observation), dtype=np.float32)

    def _observation(self, vehID, is_done=False, is_arrived=False):
        if vehID not in self.traci_connect.vehicle.getIDList():
            return self._remove_observation(vehID)
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
        directions = [self._sumo_util._get_direction_along_route(vehID)]
        cur_edgeID = self.traci_connect.vehicle.getRoadID(vehID)
        cur_lane_index = self.traci_connect.vehicle.getLaneIndex(vehID)
        directions = self._network.get_next_directions(cur_edgeID, cur_lane_index)
        turn_direction = list(
            map(lambda direct: 1.0 if direct in directions else 0.0, gc.DIRECTION)
        )
        goal_vector = list(self._goal[vehID]["direct"])
        observation = [relative_goal_pos, veh_vector, turn_direction, goal_vector]
        return np.array(flatten_list(observation), dtype=np.float32)

    def _take_action(self, vehID, action):
        is_take, is_set_speed = False, True
        v_list = self.traci_connect.vehicle.getIDList()
        removed_list = self._removed_vehID_list
        if vehID not in v_list or vehID in removed_list:
            self._remove_car_if_necessary(vehID, True)
            return False
        future_speed = -1.0
        if action > gc.DIRECT_FLAG:
            cur_speed = self.traci_connect.vehicle.getSpeed(vehID)
            accel_rate = gc.ACCEL[action - gc.DIRECT_FLAG - 1]
            accel = self.traci_connect.vehicle.getAccel(vehID) * abs(accel_rate)
            decel = self.traci_connect.vehicle.getDecel(vehID) * abs(accel_rate) * -1.0
            future_accel = accel if accel_rate >= 0 else decel
            future_accel *= self._step_length
            future_speed = cur_speed + future_accel
            is_set_speed = future_speed >= 0.0
            future_speed = future_speed if is_set_speed else 0.0
            self.traci_connect.vehicle.setSpeed(vehID, future_speed)
            self._remove_car_if_necessary(vehID, False)
        pre_direction = self._vehID_list[vehID].get("want_turn_direct", gc.DIRECTION[0])
        direction = gc.DIRECTION[action] if action <= gc.DIRECT_FLAG else pre_direction
        self._vehID_list[vehID]["want_turn_direct"] = direction
        is_take = self._sumo_util.turn(
            vehID, self._vehID_list[vehID], direction, future_speed
        )
        self._reset_routeID(vehID)
        self._remove_car_if_necessary(vehID, False)
        return is_take and is_set_speed
