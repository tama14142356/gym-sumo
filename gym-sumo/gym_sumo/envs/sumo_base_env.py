import gym
from gym import error
from gym.utils import seeding

import os
import sys

try:
    import traci
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(
            e
        )
    )

import numpy as np
from PIL import Image
import tempfile
from IPython import embed  # for debug

from ._graph import Graph
from ._util import random_tuple, get_base_vector
from .sumo_util import SumoUtil

AREA = ["nishiwaseda", "waseda_university"]
# standard length for adding car
SPOS = 10.5
# standard speed (40km/h) for vehicle
STANDARD_SPEED = 100.0 / 9.0
# standard vehicle length(m)
VEH_LEN = 5.0
# vehicle direction
STRAIGHT = 0
UTURN = 1
LEFT = 2
PAR_LEFT = 3
RIGHT = 4
PAR_RIGHT = 5

DIRECTION = ["s", "T", "l", "L", "r", "R"]

# vehicle signal number
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


class SumoBaseEnv(gym.Env):
    sumo_label = []

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
        super().__init__()
        sumo_config = "sumo_configs/" + AREA[area]
        sumo_map = os.path.join(os.path.dirname(__file__), sumo_config)
        self._netpath = os.path.join(sumo_map, "osm.net.xml")

        self.metadata = {"render.modes": ["human", "rgb_array"]}
        self.np_img = None

        self._sumocfg = os.path.join(sumo_map, "osm.sumocfg")
        self._carnum = carnum
        self._mode = mode
        self.label = label
        self._vehID_list = {}
        self._goal = {}
        self._removed_vehID_list = []
        self._start_edge_list = []
        self.seed(seed)
        self._step_length = step_length
        self._simulation_end = float(simulation_end)
        self._cur_simulation_start = 0.0
        self._is_graph = isgraph
        self._graph = Graph(self._netpath)
        self._init_simulator(mode, step_length=step_length)
        self._add_car(carnum)
        self._sumo_util = SumoUtil(
            self._graph, self._step_length, DIRECTION, self.traci_connect
        )

    def render(self, mode="human"):
        if self._mode == "gui":
            if mode == "rgb_array":
                return self.np_img

    def close(self):
        try:
            self.traci_connect.close()
            sys.stdout.flush()
        except traci.exceptions.FatalTraCIError as ci:
            print(ci)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _is_done(self, vehID):
        removed_list = self._removed_vehID_list
        v_list = self.traci_connect.vehicle.getIDList()
        cur_time = self.traci_connect.simulation.getTime() - self._cur_simulation_start
        return (
            vehID in removed_list
            or vehID not in v_list
            or cur_time >= self._simulation_end
        )

    def _reset_simulate_time(self, cur_time):
        self._cur_simulation_start = cur_time

    def _get_cur_step(self):
        cur_sumo_time = self.traci_connect.simulation.getTime()
        cur_step = cur_sumo_time - self._cur_simulation_start
        return cur_step

    def _reset_goal_element(self, vehID, goal_edgeID):
        self._vehID_list[vehID]["goal"] = goal_edgeID
        goal_laenID = goal_edgeID + "_0"
        goal_element = {}
        lane_pos = self.traci_connect.lane.getShape(goal_laenID)
        goal_element["pos"] = list(lane_pos[1])
        goal_element["direct"] = get_base_vector(lane_pos[0], lane_pos[1])
        self._goal[vehID] = goal_element

    def _init_simulator(self, mode="gui", routing_alg="dijkstra", step_length=0.01):
        sumocfg = self._sumocfg
        sumo_command = mode
        if mode == "gui":
            sumo_command = "sumo-gui"
        elif mode == "cui":
            sumo_command = "sumo"
        else:
            raise AttributeError("not supported mode!!")
        sumo_cmd = [
            sumo_command,
            "-c",
            sumocfg,
            "--routing-algorithm",
            routing_alg,
            "--step-length",
            str(step_length),
            "--collision.action",
            "remove",
            "--collision.check-junctions",
            "true",
            "--tls.all-off",
            "true",
        ]
        label = self.label
        traci_label = SumoBaseEnv.sumo_label
        if label in traci_label:
            index = len(traci_label)
            label = label + "{}".format(index)
        self.label = label
        traci.start(sumo_cmd, numRetries=100, label=label)
        self.traci_connect = traci.getConnection(label)

    def _add_car(self, carnum):
        for i in range(carnum):
            vehID = "veh{}".format(i)
            routeID, start_end_edge = self._generate_route(i)
            from_edgeID, to_edgeID = start_end_edge
            veh_element = {"route": routeID, "start": from_edgeID, "goal": to_edgeID}
            self._vehID_list[vehID] = veh_element
            self._reset_goal_element(vehID, to_edgeID)
            self._start_edge_list.append(self._graph.getEdgeIndex(from_edgeID))
            self.traci_connect.vehicle.add(vehID, routeID)
            self.traci_connect.vehicle.setSpeedMode(vehID, 0)
            self.traci_connect.vehicle.setSpeed(vehID, 0.0)
            self.traci_connect.simulationStep()

        self._reset_simulate_time(self.traci_connect.simulation.getTime())

    def _reposition_car(self):
        self.traci_connect.simulationStep()
        v_list = self.traci_connect.vehicle.getIDList()
        for i, vehID in enumerate(self._vehID_list):
            if vehID in self._removed_vehID_list and vehID not in v_list:
                routeID = self._vehID_list[vehID]["route"]
                edgeID = self._vehID_list[vehID]["start"]
                goal_edgeID = self._sumo_util.get_target(routeID=routeID)
                self._reset_goal_element(vehID, goal_edgeID)
                laneID = edgeID + "_0"
                lane_vehID_list = self.traci_connect.lane.getLastStepVehicleIDs(laneID)
                min_pos = max(SPOS, self.traci_connect.lane.getLength(laneID))
                for vehID in lane_vehID_list:
                    lane_pos = self.traci_connect.vehicle.getLanePosition(vehID)
                    min_pos = min(lane_pos, min_pos)
                if min_pos >= SPOS:
                    self.traci_connect.vehicle.add(vehID, routeID)
                    self.traci_connect.vehicle.setSpeedMode(vehID, 0)
                    self.traci_connect.vehicle.setSpeed(vehID, 0.0)
            self.traci_connect.simulationStep()

        self._reset_simulate_time(self.traci_connect.simulation.getTime())

    def _generate_route(self, index):
        exclude = []
        routeID = "route{}".format(index)
        is_infinite, is_find = (index == 0), False
        find_num, max_find_num = 0, 10
        while not is_find:
            if not is_infinite and find_num >= max_find_num:
                other_index = self.np_random.randint(0, index)
                routeID = "route{}".format(other_index)
                route = self.traci_connect.route.getEdges(routeID)
                start_end_edge = [route[0], route[len(route) - 1]]
                break
            is_find, exclude_edge, start_end_edge = self._find_route(routeID, exclude)
            exclude.append(exclude_edge)
            find_num += 1

        return routeID, start_end_edge

    def _find_route(self, routeID, exclude=[]):
        num_edge = self._graph.getNum("edge_normal") - 1
        is_find = False
        edges = random_tuple(
            0, num_edge, 2, self._start_edge_list, exclude, self.np_random
        )
        from_edgeID = self._graph.getEdgeID(edges[0])
        to_edgeID = self._graph.getEdgeID(edges[1])
        if edges in exclude:
            return is_find, edges, [from_edgeID, to_edgeID]
        route = self.traci_connect.simulation.findRoute(from_edgeID, to_edgeID)
        if len(route.edges) > 0:
            self.traci_connect.route.add(routeID, route.edges)
            is_find = True
        return is_find, edges, [from_edgeID, to_edgeID]

    def _insert_car(self, vehID, routeID, startEdgeID=None):
        if startEdgeID is None:
            startEdgeID = self.traci_connect.route.getEdges(routeID)[0]
        # set speed mode
        self.traci_connect.vehicle.setSpeedMode(vehID, 0)
        laneID = startEdgeID + "_0"
        length = min(
            self.traci_connect.vehicle.getLength(vehID),
            self.traci_connect.lane.getLength(laneID),
        )
        # insert car instantly
        self.traci_connect.vehicle.moveTo(vehID, laneID, length)
        self.traci_connect.vehicle.setSpeed(vehID, 0.0)

    def screenshot_and_simulation_step(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            viewID = self.traci_connect.gui.DEFAULT_VIEW
            img_path = tmpdir + "/screenshot.png"
            self.traci_connect.gui.screenshot(viewID, img_path)
            self.traci_connect.simulationStep()
            with open(img_path, "rb") as f:
                img = Image.open(f).convert("RGB")
                self.np_img = np.array(img)
