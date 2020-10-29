import gym
from gym import error
from gym.utils import seeding

import os
import sys

try:
    import traci
    from traci.exceptions import TraCIException
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))

import numpy as np
from PIL import Image
import tempfile
from IPython import embed  # for edbug

from ._graph import Graph
from ._util import randomTuple
from .sumo_util import SumoUtil

AREA = ['nishiwaseda', 'waseda_university']
SPOS = 10.5
VEH_LEN = 5.0


class SumoBaseEnv(gym.Env):
    def __init__(self, isgraph=True, area=0, carnum=100, mode='gui', step_length=0.01,
                 simulation_end=100, seed=None):
        super().__init__()
        sumo_config = 'sumo_configs/' + AREA[area]
        sumo_map = os.path.join(os.path.dirname(__file__), sumo_config)
        self._netpath = os.path.join(sumo_map, 'osm.net.xml')

        self.metadata = {'render.modes': ['human', 'rgb_array']}
        self.np_img = None

        self._sumocfg = os.path.join(sumo_map, 'osm.sumocfg')
        self._carnum = carnum
        self._mode = mode
        self._vehID_list = {}
        self._removed_vehID_list = []
        self._step_length = step_length
        self._simulation_end = float(simulation_end)
        self._cur_simulation_start = 0.0
        self._is_graph = isgraph
        self._graph = Graph(self._netpath)
        self._sumo_util = SumoUtil(self._graph, self._step_length)
        self._start_edge_list = []
        self.seed(seed)
        self._init_simulator(mode, step_length=step_length)
        self._add_car(carnum)

    def render(self, mode='human'):
        if self._mode == 'gui':
            if mode == 'rgb_array':
                return self.np_img

    def close(self):
        try:
            traci.close()
            sys.stdout.flush()
        except traci.exceptions.FatalTraCIError as ci:
            print(ci)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _is_done(self, vehID):
        removed_list = self._removed_vehID_list
        v_list = traci.vehicle.getIDList()
        cur_time = traci.simulation.getTime() - self._cur_simulation_start
        return (vehID in removed_list
                or vehID not in v_list
                or cur_time >= self._simulation_end)

    def _reset_simulate_time(self, cur_time):
        self._cur_simulation_start = cur_time

    def _init_simulator(self, mode='gui', routing_alg='dijkstra', step_length=0.01):
        sumocfg = self._sumocfg
        sumo_command = mode
        if mode == 'gui':
            sumo_command = 'sumo-gui'
        elif mode == 'cui':
            sumo_command = 'sumo'
        else:
            raise AttributeError("not supported mode!!")
        sumoCmd = [sumo_command, '-c', sumocfg, '--routing-algorithm', routing_alg,
                   '--step-length', str(step_length), '--collision.action', 'remove',
                   '--collision.check-junctions', 'true', '--tls.all-off', 'true',
                   ]
        traci.start(sumoCmd, numRetries=100)

    def _add_car(self, carnum):
        for i in range(carnum):
            vehID = 'veh{}'.format(i)
            routeID = 'route{}'.format(i)
            from_edgeID, to_edgeID = self._generate_route(routeID)
            veh_element = {'route': routeID,
                           'start': from_edgeID, 'goal': to_edgeID}
            self._vehID_list[vehID] = veh_element
            traci.vehicle.add(vehID, routeID)
            traci.vehicle.setSpeedMode(vehID, 0)
            traci.vehicle.setSpeed(vehID, 0.0)

        traci.simulationStep()
        self._reset_simulate_time(traci.simulation.getTime())

    def _reposition_car(self):
        traci.simulationStep()
        v_list = traci.vehicle.getIDList()
        for i, vehID in enumerate(self._vehID_list):
            if vehID in self._removed_vehID_list and vehID not in v_list:
                routeID = self._vehID_list[vehID]['route']
                edgeID = self._vehID_list[vehID]['start']
                laneID = edgeID + '_0'
                lane_vehID_list = traci.lane.getLastStepVehicleIDs(laneID)
                min_pos = max(SPOS, traci.lane.getLength(laneID))
                for vehID in lane_vehID_list:
                    lane_pos = traci.vehicle.getLanePosition(vehID)
                    min_pos = min(lane_pos, min_pos)
                if min_pos >= SPOS:
                    traci.vehicle.add(vehID, routeID)
                    traci.vehicle.setSpeedMode(vehID, 0)
                    traci.vehicle.setSpeed(vehID, 0.0)
        traci.simulationStep()
        self._reset_simulate_time(traci.simulation.getTime())

    def _generate_route(self, routeID):
        num_edge = self._graph.getNum('edge_normal') - 1
        from_edgeID = None
        to_edgeID = None
        for i in range(10):
            edges = randomTuple(
                0, num_edge, 2, self._start_edge_list, self.np_random)
            from_edgeID = self._graph.getEdgeID(edges[0])
            to_edgeID = self._graph.getEdgeID(edges[1])
            try:
                route = traci.simulation.findRoute(from_edgeID, to_edgeID)
                if len(route.edges) > 0:
                    traci.route.add(routeID, route.edges)
                    break
            except TraCIException as tr:
                print(tr)
        return from_edgeID, to_edgeID

    def _insert_car(self, vehID, routeID, startEdgeID=None):
        if startEdgeID is None:
            startEdgeID = traci.route.getEdges(routeID)[0]
        # set speed mode
        traci.vehicle.setSpeedMode(vehID, 0)
        laneID = startEdgeID + '_0'
        length = min(traci.vehicle.getLength(vehID),
                     traci.lane.getLength(laneID))
        # insert car instantly
        traci.vehicle.moveTo(vehID, laneID, length)
        traci.vehicle.setSpeed(vehID, 0.0)

    def screenshot_and_simulation_step(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            viewID = traci.gui.DEFAULT_VIEW
            img_path = tmpdir + '/screenshot.png'
            traci.gui.screenshot(viewID, img_path)
            traci.simulationStep()
            with open(img_path, 'rb') as f:
                img = Image.open(f).convert('RGB')
                self.np_img = np.array(img)
