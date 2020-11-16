from ._graph import Graph
from ._util import flatten_list
from .sumo_base_env import SumoBaseEnv as BaseEnv
from .sumo_base_env import STRAIGHT, DIRECTION

from gym import spaces
import numpy as np
import torch
from torch_geometric.data import Data

from traci import constants as tc

# action
STOP = 6


class SumoEnv(BaseEnv):
    def __init__(
        self,
        isgraph=True,
        area=0,
        carnum=100,
        mode="gui",
        step_length=0.01,
        simulation_end=3600,
        seed=None,
        label="default",
    ):
        super().__init__(
            isgraph, area, carnum, mode, step_length, simulation_end, seed, label
        )
        # 7action and accel, brake
        self.action_space = []
        for i in range(carnum):
            self.action_space.append(
                spaces.Tuple(
                    (spaces.Discrete(7), spaces.Box(low=-1, high=1, shape=(1,)))
                )
            )
        observation = self._observation()
        self.observation_space = spaces.Box(
            low=0, high=np.inf, shape=(np.shape(observation))
        )
        self.reward_range = [(-5, 10) for i in range(carnum)]

    def _is_dones(self):
        dones = []
        for i, vehID in enumerate(self._vehID_list):
            is_done = self._is_done(vehID)
            dones.append(is_done)
        return dones

    def step(self, action):
        # err_msg = "%r (%s) invalid" % (action, type(action))
        # assert self.action_space.contains(action), err_msg

        # determine next step action
        is_take_action = [True] * self._carnum
        for i, act in enumerate(action):
            vehID = list(self._vehID_list)[i]
            is_take = self._take_action(vehID, act)
            is_take_action[i] = is_take
        self.traci_connect.simulationStep()
        is_dones = self._is_dones()
        observation = self._observation()
        reward = self.reward(is_take_action)
        return observation, reward, is_dones, {}

    def reset(self):
        mode = self._mode
        self.__is_init = True
        # reset
        self.close()
        self._graph = Graph(self._netpath)
        self._start_edge_list = []
        # traci start & init simulate
        self._init_simulator(mode=mode, step_length=self._step_length)
        self._vehID_list.clear()
        self._removeID_list.clear()
        self._add_car(self._carnum)
        observation = self._observation()
        self.__is_init = False
        self.traci_connect.simulationStep()
        return observation

    def render(self, mode="human"):
        if self._isgraph:
            self._graph.check_graph(self.data)
        print(self.data)

    def reward(self, is_take_action):
        v_list = self.traci_connect.vehicle.getIDList()
        collision_list = self.traci_connect.simulation.getCollidingVehiclesIDList()
        reward_list = [0] * self._carnum
        removed_list = self._removeID_list
        # exist_num = self._carnum - len(removed_list)
        for i, vehID in enumerate(self._vehID_list):
            reward = 0
            if vehID in v_list and vehID not in removed_list:
                if is_take_action[i]:
                    cur_length = self.traci_connect.vehicle.getDistance(vehID)
                    total_len = self.get_route_length(vehID)
                    # if this car has never moved, reward=0
                    tmp = 0
                    # if this car has already moved,
                    # reward=ratio of route length
                    reward_max = self.reward_range[i][1]
                    if total_len > 0:
                        tmp = int((cur_length * reward_max) / total_len)
                    reward += tmp
                    speed = self.traci_connect.vehicle.getSpeed(vehID)
                    if speed <= 0.0:
                        reward -= tmp
                    if vehID in collision_list:
                        reward -= 2
                else:
                    if vehID not in removed_list:
                        reward -= 2
                        self.traci_connect.vehicle.remove(vehID, tc.REMOVE_VAPORIZED)
                        self._removeID_list.append(vehID)

        return reward_list

    def _observation(self):
        vehID_list = self._vehID_list
        if self._isgraph:
            for vehID in vehID_list:
                v_info = {}
                v_info["ID"] = vehID
                v_info["pos"] = list(self.traci_connect.vehicle.getPosition(vehID))
                v_info["speed"] = [self.traci_connect.vehicle.getSpeed(vehID)]
                v_info["exist"] = v_info["speed"] != tc.INVALID_DOUBLE_VALUE
                v_info["curEdgeID"] = self.traci_connect.vehicle.getRoadID(vehID)
                road = self.traci_connect.vehicle.getRoute(vehID)
                road_index = self.traci_connect.vehicle.getRouteIndex(vehID)
                v_info["nextEdgeID"] = None
                if len(road) > road_index:
                    v_info["nextEdgeID"] = road[road_index + 1]
                if self.__is_init:
                    self._graph.addNode(v_info)
                else:
                    self._graph.updateNode(v_info)
            obs = self._graph.getGraph()
            observation = self.change_numpy(obs)
            self.data = obs
        return observation

    def change_numpy(self, data):
        """convert from Data object to one dimension numpy array

        Args:
            data (Data): graph data

        Returns:
            ndarray: numpy array of graph data
        """
        tmp = []
        if data.x is not None:
            tmp.append(data.x.numpy().tolist())
        if data.edge_index is not None:
            tmp.append(data.edge_index.numpy().tolist())
        if data.edge_attr is not None:
            tmp.append(data.edge_attr.numpy().tolist())
        if data.pos is not None:
            tmp.append(data.pos.numpy().tolist())
        tmp_observation = flatten_list(tmp)
        observation = np.array(tmp_observation)
        return observation

    def get_data(self, obsdata=None):
        if obsdata is None:
            return self.data
        data = Data()
        obs = self.data
        tmpobs = obsdata.numpy().tolist()
        observation = np.array(flatten_list(tmpobs))
        num_node = obs.num_nodes
        num_node = 0 if num_node is None else num_node
        num_node_features = obs.num_node_features
        num_node_features = 0 if num_node_features is None else num_node_features
        start = 0
        end = num_node * num_node_features
        x_list = [] if start < end else None
        for i in range(start, end, num_node_features):
            x = observation[i : i + num_node_features].astype(np.float)
            x_list.append(x)
        data.x = torch.tensor(x_list, dtype=torch.float)
        num_edge = obs.num_edges
        num_edge = 0 if num_edge is None else num_edge
        if num_edge > self._carnum:
            start = num_node * num_node_features
            end = start + num_edge
            src = observation[start:end].astype(np.int64)
            start = end
            end = start + num_edge
            dsc = observation[start:end].astype(np.int64)
            edge_index = np.array([src, dsc], dtype=np.int64)
            data.edge_index = torch.tensor(edge_index, dtype=torch.long)
        num_edge_features = obs.num_edge_features
        start = end
        end = start + num_edge * num_edge_features
        edge_attr = [] if start < end else None
        for i in range(start, end, num_edge_features):
            tmp = observation[i : i + num_edge_features]
            edge_attr.append(tmp)
        data.edge_attr = torch.tensor(edge_attr, dtype=torch.float)
        pos_dim = 0 if obs.pos is None else len(obs.pos[0])
        start = end
        end = start + num_node * pos_dim
        pos = [] if start < end else None
        for i in range(start, end, pos_dim):
            tmp = observation[i : i + pos_dim]
            pos.append(tmp)
        data.pos = torch.tensor(pos, dtype=torch.float)
        return data

    def _take_action(self, vehID, action):
        is_take = False
        v_list = self.traci_connect.vehicle.getIDList()
        if vehID not in v_list:
            return True
        removed_list = self._removeID_list
        if vehID in removed_list:
            return False
        cur_speed = self.traci_connect.vehicle.getSpeed(vehID)
        accel_rate = action[1][0]
        accel = self.traci_connect.vehicle.getAccel(vehID) * accel_rate
        decel = self.traci_connect.vehicle.getDecel(vehID) * accel_rate
        future_accel = accel if accel_rate >= 0 else decel
        future_accel *= self._step_length
        future_speed = cur_speed + future_accel
        is_junction = self._sumo_util._is_junction(vehID)
        if action[0] == STOP:
            future_speed = 0.0
            is_take = True
        else:
            direction = DIRECTION[action[0]]
            if self._sumo_util._could_turn(vehID, direction)[0]:
                if is_junction:
                    route_index = self.traci_connect.vehicle.getRouteIndex(vehID)
                    route = self.traci_connect.vehicle.getRoute(vehID)
                if self._sumo_util.turn(vehID, direction):
                    is_take = True
            else:
                if is_junction:
                    route_index = self.traci_connect.vehicle.getRouteIndex(vehID)
                    route = self.traci_connect.vehicle.getRoute(vehID)
                    if route_index + 1 < len(route):
                        toEdgeID = route[route_index + 1]
                        preEdgeID = route[route_index]
                        direct = self._graph.getNextInfoDirect(
                            curEdgeID=preEdgeID, toEdgeID=toEdgeID
                        )
                        if direct == direction:
                            is_take = True
                else:
                    if direction == DIRECTION[STRAIGHT]:
                        is_take = True
        self.traci_connect.vehicle.setSpeed(vehID, future_speed)
        return is_take

    def get_route_length(self, vehID):
        route = self.traci_connect.vehicle.getRoute(vehID)
        length = 0
        num = len(route)
        for i, edgeID in enumerate(route):
            laneID = edgeID + "_0"
            length += self.traci_connect.lane.getLength(laneID)
            if i < num - 1:
                next_edgeID = route[i + 1]
                via_laneID = self._graph.getNextInfoVia(edgeID, toEdgeID=next_edgeID)
                length += self.traci_connect.lane.getLength(via_laneID)
        return length
