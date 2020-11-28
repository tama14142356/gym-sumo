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

        acheived_list = self.traci_connect.simulation.getArrivedIDList()

        info = {}
        info["is_take"] = is_take
        info["is_arrived"] = vehID in acheived_list
        info["goal"] = self._vehID_list[vehID]["goal"]
        info["cur_lane"] = (
            "" if info["is_arrived"] else self.traci_connect.vehicle.getLaneID(vehID)
        )
        info["cur_lane_pos"] = (
            -1.0
            if info["is_arrived"]
            else self.traci_connect.vehicle.getLanePosition(vehID)
        )
        info["lane_len"] = (
            0.0
            if len(info["cur_lane"]) <= 0
            else self.traci_connect.lane.getLength(info["cur_lane"])
        )
        info["speed"] = (
            -1.0 if info["is_arrived"] else self.traci_connect.vehicle.getSpeed(vehID)
        )
        info["pos"] = (
            (-1.0, -1.0)
            if info["is_arrived"]
            else self.traci_connect.vehicle.getPosition(vehID)
        )

        observation = (
            self._done_observation(vehID)
            if vehID in acheived_list
            else self._observation(vehID)
        )

        # calculate reward
        reward = 0.0
        collision_list = self.traci_connect.simulation.getCollidingVehiclesIDList()
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
                reward += 0.1
                # progress bonus
                cur_driving_len = self.traci_connect.vehicle.getDistance(vehID)
                progress = cur_driving_len - pre_driving_len
                routeID = self._vehID_list[vehID]["route"]
                total_length = self._route_list[routeID]["length"]
                if progress > 0:
                    reward += 0.1
                    reward += 0.0 if total_length <= 0 else progress / total_length
                info["driving_len"] = cur_driving_len

        isdone = self._is_done(vehID)
        info["is_removed"] = vehID in removed_list
        info["cur_step"] = self._get_cur_step()
        info["cur_sm_step"] = self.traci_connect.simulation.getTime()
        info["needs_reset"] = info["is_removed"]
        return observation, reward, isdone, info

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

    def _done_observation(self, vehID):
        goal_vector = list(self._goal[vehID]["direct"])
        relative_goal_pos = list([0.0, 0.0])
        turn_direction = [0.0] * (DIRECT_FLAG + 1)
        obs = [relative_goal_pos, goal_vector, turn_direction, goal_vector]
        obs = flatten_list(obs)
        observation = np.array(obs, dtype=np.float32)
        return observation

    def _observation(self, vehID):
        pos = list(self.traci_connect.vehicle.getPosition(vehID))
        if pos[0] == tc.INVALID_DOUBLE_VALUE or pos[1] == tc.INVALID_DOUBLE_VALUE:
            pos[0], pos[1] = -np.inf, -np.inf
        goal_pos = self._goal[vehID]["pos"]
        relative_goal_pos = list(np.array(goal_pos) - np.array(pos))
        veh_len = self.traci_connect.vehicle.getLength(vehID)
        angle = self.traci_connect.vehicle.getAngle(vehID)
        veh_vector = list(vector_decomposition(veh_len, angle))
        could_reach, cur_laneID = self._sumo_util._could_reach_junction(vehID)
        directions = []
        if could_reach:
            cur_edgeID = self.traci_connect.lane.getEdgeID(cur_laneID)
            cur_lane_index = cur_laneID.replace(cur_edgeID, "")[1:]
            directions = self._network.get_next_directions(cur_edgeID, cur_lane_index)
        turn_direction = list(
            map(lambda direct: 1.0 if direct in directions else 0.0, DIRECTION)
        )
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
                return False
        direction = DIRECTION[0] if action > DIRECT_FLAG else DIRECTION[action]
        could_turn, _, _ = self._sumo_util._could_turn(vehID, direction, future_speed)
        is_take = could_turn
        if could_turn:
            self._sumo_util.turn(vehID, direction, future_speed)
            self._reset_goal_element(vehID)
            self._reset_routeID(vehID)
        return is_take
