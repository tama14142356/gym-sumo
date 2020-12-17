from .sumo_light_env import SumoLightEnv as LightEnv

from traci import constants as tc

# from IPython import embed  # for debug

START_EDGE = "193327849#0"
GOAL_EDGE = "-94518685#3"


class SumoFixedEnv(LightEnv):
    def __init__(
        self,
        area=0,
        carnum=100,
        mode="gui",
        step_length=0.01,
        simulation_end=200,
        seed=None,
        label="default",
        debug_view=False,
        start_edgeID=START_EDGE,
        end_edgeID=GOAL_EDGE,
        fixed_veh_index=0,
    ):
        super().__init__(
            area=0,
            carnum=carnum,
            mode=mode,
            step_length=step_length,
            simulation_end=simulation_end,
            seed=seed,
            label=label,
            debug_view=debug_view,
        )
        self.fixed_veh_index = fixed_veh_index
        self.fixed_vehID = list(self._vehID_list)[fixed_veh_index]
        self.start_edgID = start_edgeID
        self.end_edgeID = end_edgeID
        self._add_or_reposition_fix_car()

    def _add_or_reposition_fix_car(self):
        vehID = self.fixed_vehID
        if vehID in self.traci_connect.vehicle.getIDList():
            self.traci_connect.vehicle.remove(vehID, tc.REMOVE_TELEPORT)
            self.traci_connect.simulationStep()
        routeID, route, route_info, is_find = self._generate_fix_route()
        self.traci_connect.route.add(routeID, route)
        self._route_list[routeID] = route_info
        self.traci_connect.vehicle.add(vehID, routeID)
        self.traci_connect.vehicle.setLaneChangeMode(vehID, 0)
        self.traci_connect.vehicle.setSpeedMode(vehID, 0)
        self.traci_connect.vehicle.setSpeed(vehID, 0.0)
        self.traci_connect.simulationStep()
        start_pos = self.traci_connect.vehicle.getPosition(vehID)
        self._vehID_list[vehID]["start_pos"] = list(start_pos)
        is_load = vehID in self.traci_connect.vehicle.getIDList()
        self._vehID_list[vehID]["is_load"] = is_load
        self._reset_simulate_time()

    def _generate_fix_route(self):
        index = self.fixed_veh_index
        vehID = self.fixed_vehID
        routeID = "route{}".format(index)
        routeID_list = self.traci_connect.route.getIDList()
        if routeID in routeID_list:
            routeID = "route{}".format(len(routeID_list))
        start_edgeID, end_edgeID = self.start_edgID, self.end_edgeID
        route = self.traci_connect.simulation.findRoute(start_edgeID, end_edgeID)
        route_info = self._sumo_util._get_route_info(route_edges=route.edges)
        is_find = len(route.edges) > 0
        self._reset_goal_element(vehID, end_edgeID)
        start_edge_index = self._network.get_edge_index(start_edgeID)
        if start_edge_index not in self._start_edge_list:
            self._start_edge_list.append(start_edge_index)
        veh_element = {"route": routeID, "start": start_edgeID, "goal": end_edgeID}
        self._vehID_list[vehID] = veh_element
        return routeID, route.edges, route_info, is_find

    def reset(self):
        vehID = self.fixed_vehID
        self._add_or_reposition_fix_car()
        if self._mode == "gui":
            viewID = self.traci_connect.gui.DEFAULT_VIEW
            # self.traci_connect.gui.trackVehicle(viewID, vehID)
            # zoom = self.traci_connect.gui.getZoom()
            self.traci_connect.gui.setZoom(viewID, 1000)
            self.screenshot_and_simulation_step()
            self._reset_simulate_time()
        observation = self._observation(vehID)
        return observation
