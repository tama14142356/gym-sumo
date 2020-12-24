import gym
from gym import error
from gym.utils import seeding

import os
import sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import tempfile

# from IPython import embed  # for debug

from ._graph import Graph
from ._util import random_tuple, get_base_vector
from .sumo_util import SumoUtil

MESSAGE_HINT = (
    "(HINT: you can install sumo or set the path for sumo library by reading README.md)"
)
try:
    import traci
    from traci import constants as tc
except ImportError as e:
    raise error.DependencyNotInstalled("{}.".format(e) + MESSAGE_HINT)


AREA = ["nishiwaseda", "waseda_university"]
# max length for route length
MAX_LENGTH = 1000
# road number of route
MAX_ROAD_NUM = 20
MIN_ROAD_NUM = 2
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
RENDER_TEXT = ["STRAIGHT", "UTURN", "LEFT", "PAR_LEFT", "RIGHT", "PAR_RIGHT"]

INFO = {
    "is_take": False,
    "is_removed": False,
    "cur_step": -1.0,
    "cur_sm_step": -1.0,
    "is_arrived": False,
    "needs_reset": False,
    "goal": {},
    "cur_lane": "",
    "cur_lane_pos": -1.0,
    "lane_len": -1.0,
    "speed": -1.0,
    "pos": (-1.0, -1.0),
    "driving_len": -1.0,
}

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
        simulation_end=200,
        seed=None,
        label="default",
        debug_view=False,
        is_random_route=False,
    ):
        super().__init__()
        sumo_config = "sumo_configs/" + AREA[area]
        self._sumo_map = os.path.join(os.path.dirname(__file__), sumo_config)
        self._netpath = os.path.join(self._sumo_map, "osm.net.xml")
        self._sumocfg = os.path.join(self._sumo_map, "osm.sumocfg")
        if debug_view:
            self._sumocfg = os.path.join(self._sumo_map, "osm_debug.sumocfg")
        # set 1 video frame / 1s
        self.metadata = {
            "render.modes": ["human", "rgb_array"],
            "video.frames_per_second": 1,
        }
        self.np_img = None
        self._road_num = MIN_ROAD_NUM
        self.action_text = RENDER_TEXT.copy()
        self._cur_simulation_start = 0.0
        self._carnum = carnum
        self._mode = mode
        self.label = label
        self._step_length = float(step_length)
        self._simulation_end = float(simulation_end)
        self._is_graph = isgraph
        self._is_random_route = is_random_route
        self._route_list = {}
        self._vehID_list = {}
        self._goal = {}
        self._removed_vehID_list = []
        self._start_edge_list = []
        self._seed = seed
        self.seed(seed)
        self._graph = Graph(self._netpath)
        self._network = self._graph.sumo_network
        self._init_simulator(mode, step_length=step_length)
        self._sumo_util = SumoUtil(self._network, DIRECTION, self.traci_connect)
        self._add_all_car()

    def initiallize_list(self, is_fix_target=True):
        self._vehID_list.clear()
        self._removed_vehID_list.clear()
        self._goal.clear()
        self._route_list.clear()
        if not self.is_close():
            routeID_list = self.traci_connect.route.getIDList()
            for routeID in routeID_list:
                self._reset_routeID(routeID=routeID, is_set_veh=False)
        if is_fix_target:
            self._start_edge_list.clear()

    def render(self, mode="human"):
        if self._mode == "gui":
            if mode == "rgb_array":
                return self.np_img

    def is_close(self):
        return self.traci_connect._socket is None

    def close(self):
        try:
            self.traci_connect.close()
            sys.stdout.flush()
        except self.traci_connect.exceptions.FatalTraCIError as ci:
            print(ci)

    @property
    def np_random(self):
        """Lazily seed the rng since this is expensive and only needed if
        sampling from this space.
        """
        if self._np_random is None:
            self.seed(self._seed)

        return self._np_random

    @property
    def road_num(self):
        if type(self._road_num) != int and type(self._road_num) != float:
            self._road_num = int(MIN_ROAD_NUM)
        return self._road_num

    @road_num.setter
    def road_num(self, length):
        if type(length) != int and type(length) != float:
            raise TypeError("invalid type")
        self._road_num = min(int(length), MAX_ROAD_NUM)

    def seed(self, seed=None):
        self._np_random, seed = seeding.np_random(seed)
        return [seed]

    def _is_done(self, vehID):
        removed_list = self._removed_vehID_list
        v_list = self.traci_connect.vehicle.getIDList()
        cur_time = self._get_cur_step()
        return (
            vehID in removed_list
            or vehID not in v_list
            or cur_time >= self._simulation_end
        )

    def _reset_simulate_time(self, cur_time=-1):
        if cur_time < 0:
            cur_time = self.traci_connect.simulation.getTime()
        self._cur_simulation_start = cur_time

    def _get_cur_step(self):
        cur_sumo_time = self.traci_connect.simulation.getTime()
        cur_step = cur_sumo_time - self._cur_simulation_start
        return cur_step

    def _get_vector_pos_edgeID(self, edgeID, lane_index=0):
        laneID = self._network.get_laneID(edgeID, lane_index)
        lane_pos_list = self.traci_connect.lane.getShape(laneID)
        shape_num = len(lane_pos_list)
        end_pos = lane_pos_list[shape_num - 1]
        start_or_mid_pos = lane_pos_list[shape_num - 2]
        vector = get_base_vector(start_or_mid_pos, end_pos)
        return vector, end_pos

    def _reset_goal_element(self, vehID, goal_edgeID):
        self._vehID_list[vehID]["goal"] = goal_edgeID
        goal_element = {}
        vector, pos = self._get_vector_pos_edgeID(goal_edgeID)
        goal_element["direct"] = vector
        goal_element["pos"] = list(pos)
        self._goal[vehID] = goal_element

    def _reset_routeID(self, vehID="", routeID="", is_set_veh=True):
        if len(vehID) <= 0 and is_set_veh:
            vehID = list(self._vehID_list)[0]
        if len(routeID) <= 0:
            routeID = self.traci_connect.vehicle.getRouteID(vehID)
        if routeID not in self._route_list:
            route_info = self._sumo_util._get_route_info(routeID=routeID)
            self._route_list[routeID] = route_info
        if is_set_veh:
            self._vehID_list[vehID]["route"] = routeID

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
            "warn",
            "--collision.check-junctions",
            "true",
            "--tls.all-off",
            "true",
            "--start",
            "true",
            "--quit-on-end",
            "true",
            "--window-size",
            "1300, 700",
        ]
        label = self.label
        traci_label = SumoBaseEnv.sumo_label
        if label in traci_label:
            index = len(traci_label)
            label = label + "{}".format(index)
        SumoBaseEnv.sumo_label.append(label)
        self.label = label
        traci.start(sumo_cmd, numRetries=100, label=label)
        self.traci_connect = traci.getConnection(label)

    def _remove_car_if_necessary(self, vehID, is_remove):
        collision_list = self.traci_connect.simulation.getCollidingVehiclesIDList()
        vehID_list = self.traci_connect.vehicle.getIDList()
        removed_list = self._removed_vehID_list
        if vehID in removed_list:
            return
        if vehID not in vehID_list:
            self._removed_vehID_list.append(vehID)
            return
        if is_remove or vehID in collision_list:
            self.traci_connect.vehicle.remove(vehID, tc.REMOVE_TELEPORT)
            self._removed_vehID_list.append(vehID)

    def _add_all_car(self):
        for i in range(self._carnum):
            self._add_car(i)
        self.traci_connect.simulationStep()
        self._reset_simulate_time()

    def _add_car(self, index, vehID="", is_create_route=True, is_fix_target=False):
        if len(vehID) <= 0:
            vehID = "veh{}".format(index)
        else:
            index = int(vehID[3:])
        routeID = "route{}".format(index)
        route_list = self.traci_connect.route.getIDList()
        if is_create_route or routeID not in route_list:
            routeID, route, route_info = self._generate_route(index)
            start_edgeID, goal_edgeID = route[0], route[len(route) - 1]
            self._start_edge_list.append(self._network.get_edge_index(start_edgeID))
        else:
            route = self.traci_connect.route.getEdges(routeID)
            index = len(route_list) + index
            goal_edgeID = route[len(route) - 1]
            route_info = self._route_list[routeID]
            if is_fix_target:
                routeID, route, route_info = self._generate_route(index, goal_edgeID)
            start_edgeID, goal_edgeID = route[0], route[len(route) - 1]
        self.traci_connect.route.add(routeID, route)
        self._route_list[routeID] = route_info
        veh_element = {"route": routeID, "start": start_edgeID, "goal": goal_edgeID}
        self._vehID_list[vehID] = veh_element
        self._reset_goal_element(vehID, goal_edgeID)
        self._reset_routeID(vehID, routeID)
        self.traci_connect.vehicle.add(vehID, routeID)
        self.traci_connect.vehicle.setLaneChangeMode(vehID, 0)
        self.traci_connect.vehicle.setSpeedMode(vehID, 0)
        self.traci_connect.vehicle.setSpeed(vehID, 0.0)
        self.traci_connect.simulationStep()
        start_pos = self.traci_connect.vehicle.getPosition(vehID)
        self._vehID_list[vehID]["start_pos"] = list(start_pos)
        is_load = vehID in self.traci_connect.vehicle.getIDList()
        self._vehID_list[vehID]["is_load"] = is_load
        self._vehID_list[vehID]["want_turn_direct"] = DIRECTION[STRAIGHT]
        self._reset_simulate_time()

    def _reposition_car(self, is_fix_target=True):
        self.traci_connect.simulationStep()
        v_list = self.traci_connect.vehicle.getIDList()
        self.initiallize_list(is_fix_target)
        for i in range(self._carnum):
            vehID = "veh{}".format(i)
            if vehID in v_list:
                self.traci_connect.vehicle.remove(vehID, tc.REMOVE_TELEPORT)
                self.traci_connect.simulationStep()
            self._add_car(i, vehID, is_create_route=False, is_fix_target=is_fix_target)

        self.traci_connect.simulationStep()
        self._reset_simulate_time()

    def _generate_route(self, index, to_edgeID=""):
        exclude_list = []
        routeID = "route{}".format(index)
        is_random = self._is_random_route
        is_infinite, is_find = (index == 0), False
        find_num, max_find_num = 0, 10
        routeID_list = self.traci_connect.route.getIDList()
        route_num = len(routeID_list)
        while not is_find:
            if not is_infinite and find_num >= max_find_num and route_num > 0:
                other_routeID = self.np_random.choice(routeID_list)
                if to_edgeID != "":
                    tmp_list = self._sumo_util.get_routeID_list_from_target(to_edgeID)
                    if other_routeID not in tmp_list and len(tmp_list) > 0:
                        other_routeID = self.np_random.choice(tmp_list)
                route = self.traci_connect.route.getEdges(other_routeID)
                route_info = self._sumo_util._get_route_info(
                    routeID=other_routeID, route_info_list=self._route_list
                )
                break
            find_route_info = self._find_route(exclude_list, to_edgeID, is_random)
            is_find, exclude, route_info, route = find_route_info
            if exclude not in exclude_list:
                exclude_list.append(exclude)
            find_num += 1
        return routeID, route, route_info

    def _find_route(self, exclude=[], to_edgeID="", is_random=False):
        if is_random:
            return self._find_route_random(exclude, to_edgeID)
        num_edge = self._graph.get_num("edge_normal") - 1
        if to_edgeID == "":
            edges = random_tuple(
                0, num_edge, 2, self._start_edge_list, exclude, self.np_random
            )
            from_edgeID = self._network.get_edgeID(edges[0])
            to_edgeID = self._network.get_edgeID(edges[1])
        else:
            to_edge_index = self._network.get_edge_index(to_edgeID)
            from_edge_index = random_tuple(
                0, num_edge, 1, self._start_edge_list, exclude, self.np_random
            )[0]
            from_edgeID = self._network.get_edgeID(from_edge_index)
            edges = (from_edge_index, to_edge_index)
        if edges in exclude:
            return False, edges, None, []
        route = self.traci_connect.simulation.findRoute(from_edgeID, to_edgeID)
        route_info = self._sumo_util._get_route_info(route_edges=route.edges)
        length = route_info.get("length", MAX_LENGTH + 1)
        is_find = len(route.edges) > 0 and length <= MAX_LENGTH
        return is_find, edges, route_info, route.edges

    def _find_route_random(self, exclude=[], to_edgeID=""):
        num_edge = self._graph.get_num("edge_normal") - 1
        road_num = self.road_num
        from_edge_index, to_edge_index, is_find = -1, -1, True
        cur_road_num = 1
        to_edge_index = self._network.get_edge_index(to_edgeID)
        if to_edgeID == "":
            to_edge_index = random_tuple(
                0, num_edge, exclude=exclude, random_state=self.np_random
            )[1]
            to_edgeID = self._network.get_edgeID(to_edge_index)
        route = [to_edgeID]
        exclude_edge = []
        while True:
            from_edgeID_list = self._network.get_from_edgeIDs(to_edgeID)
            from_edgeID_list = [e for e in from_edgeID_list if e not in exclude_edge]
            if len(from_edgeID_list) <= 0:
                if len(route) <= 1:
                    break
                if to_edgeID not in exclude_edge:
                    exclude_edge.append(to_edgeID)
                    route.pop(0)
                    if to_edgeID not in route:
                        cur_road_num -= 1
                    to_edgeID = route[0]
                continue
            to_edgeID = self.np_random.choice(from_edgeID_list)
            if to_edgeID not in route:
                cur_road_num += 1
            route.insert(0, to_edgeID)
            if cur_road_num >= road_num:
                break
        from_edge_index = self._network.get_edge_index(to_edgeID)
        edges = (from_edge_index, to_edge_index)
        route_info = self._sumo_util._get_route_info(route_edges=route)
        self.road_num = cur_road_num
        return is_find, edges, route_info, route

    def screenshot_and_simulation_step(self, action=-1, vehID=""):
        with tempfile.TemporaryDirectory() as tmpdir:
            viewID = self.traci_connect.gui.DEFAULT_VIEW
            if len(vehID) <= 0:
                vehID = list(self._vehID_list)[0]
            # gui.trackVehicleの代わり
            x, y = self.traci_connect.gui.getOffset()
            cur_edgeID = ""
            if vehID in self.traci_connect.vehicle.getIDList():
                x, y = self.traci_connect.vehicle.getPosition(vehID)
                cur_edgeID = self.traci_connect.vehicle.getRoadID(vehID)
            self.traci_connect.gui.setOffset(viewID, x, y)
            img_path = tmpdir + "/screenshot.png"
            self.traci_connect.gui.screenshot(viewID, img_path)
            self.traci_connect.simulationStep()
            removed_list = self._removed_vehID_list
            acheived_list = self.traci_connect.simulation.getArrivedIDList()
            is_arrived = (
                vehID in acheived_list and cur_edgeID == self._vehID_list[vehID]["goal"]
            )
            is_removed = vehID in removed_list or vehID in acheived_list
            with open(img_path, "rb") as f:
                img = Image.open(f).convert("RGB")
                # get a font
                fnt = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 40)
                # get a drawing context
                d = ImageDraw.Draw(img)
                # draw multiline text
                text = self.create_render_text(action, vehID, is_removed, is_arrived)
                d.multiline_text((10, 10), text, font=fnt, fill=(255, 0, 0))
                self.np_img = np.array(img)

    def create_render_text(self, action, vehID, is_removed, is_arrived):
        is_reset = action < 0
        action_text = "action: " + ("RESET" if is_reset else self.action_text[action])
        action_text += "(" + str(action) + ")"
        sim_time_tx = "simulation time: " + str(self.traci_connect.simulation.getTime())
        step_tx = "current step: " + ("0.0" if is_reset else str(self._get_cur_step()))
        goal_edgeID = ""
        if vehID in self._vehID_list:
            goal_edgeID = self._vehID_list[vehID].get("goal", "")
        goal_text = "goal: " + goal_edgeID
        text = action_text + "\n" + sim_time_tx + "\n" + step_tx + "\n" + goal_text
        if is_arrived:
            text += "\nARRIVED!!"
        elif is_removed:
            text += "\nCRASH!!"
        elif self._is_done(vehID):
            text += "\nTIME OVER"
        return text
