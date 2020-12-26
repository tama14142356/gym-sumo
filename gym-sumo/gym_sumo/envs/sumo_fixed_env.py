from .sumo_light_env import SumoLightEnv as LightEnv

from traci import constants as tc

from gym_sumo.envs import route_data_save

# from IPython import embed  # for debug


class SumoFixedEnv(LightEnv):
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
        is_random_route=True,
        road_freq=100,
        is_auto=False,
        route_length=[1000.0, 2000.0],
        max_data=100000,
        is_start=False,
        is_end=True,
        fixed_veh_index=0,
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
            road_freq,
            is_auto,
        )
        self.fixed_veh_index = fixed_veh_index
        self.fixed_vehID = list(self._vehID_list)[fixed_veh_index]
        self._cur_data_index = 0
        length_dir = route_data_save.format_length_dir(route_length)
        data_dir = route_data_save.format_data_num_dir(max_data) + "/"
        def_dir = route_data_save.format_definition_length_dir(is_end, is_start) + "/"
        load_dir = self._sumo_map + "/route/" + data_dir + def_dir + length_dir
        self.route_data = route_data_save.route_data_load(load_dir)
        if self._is_random_route:
            self.np_random.shuffle(self.route_data)
        self._data_num = len(self.route_data)
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
        if not self._is_auto:
            self.traci_connect.vehicle.setLaneChangeMode(vehID, 0)
            self.traci_connect.vehicle.setSpeedMode(vehID, 0)
            self.traci_connect.vehicle.setSpeed(vehID, 0.0)
            self.traci_connect.simulationStep()
            start_pos = self.traci_connect.vehicle.getPosition(vehID)
            self._vehID_list[vehID]["start_pos"] = list(start_pos)
            is_load = vehID in self.traci_connect.vehicle.getIDList()
            self._vehID_list[vehID]["is_load"] = is_load
        self.traci_connect.simulationStep()
        self._reset_simulate_time()

    def _generate_fix_route(self):
        index = self.fixed_veh_index
        vehID = self.fixed_vehID
        routeID = "route{}".format(index)
        routeID_list = self.traci_connect.route.getIDList()
        if routeID in routeID_list:
            routeID = "route{}".format(len(routeID_list))
        start_edgeID, end_edgeID = self.route_data[self._cur_data_index]
        route = self.traci_connect.simulation.findRoute(start_edgeID, end_edgeID)
        route_info = self._sumo_util._get_route_info(route_edges=route.edges)
        is_find = len(route.edges) > 0
        start_edge_index = self._network.get_edge_index(start_edgeID)
        if start_edge_index not in self._start_edge_list:
            self._start_edge_list.append(start_edge_index)
        veh_element = {"route": routeID, "start": start_edgeID, "goal": end_edgeID}
        self._vehID_list[vehID] = veh_element
        self._reset_goal_element(vehID, end_edgeID)
        return routeID, route.edges, route_info, is_find

    def reset(self):
        vehID = self.fixed_vehID
        if not self.is_init:
            self.initiallize_list()
            self._cur_data_index += 1
            self._cur_data_index %= self._data_num
            self._add_or_reposition_fix_car()
        self.is_init = False
        if self._mode == "gui":
            viewID = self.traci_connect.gui.DEFAULT_VIEW
            # self.traci_connect.gui.trackVehicle(viewID, vehID)
            # zoom = self.traci_connect.gui.getZoom()
            self.traci_connect.gui.setZoom(viewID, 1000)
            self.screenshot_and_simulation_step()
            self._reset_simulate_time()
        observation = self._observation(vehID)
        return observation
