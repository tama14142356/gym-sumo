import numpy as np
from gym import error

try:
    import traci
    from traci.exceptions import TraCIException
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))


class SumoUtil:

    def __init__(self, graph, step_length, direction_list):
        self._graph = graph
        self._step_length = step_length
        self._direction_list = direction_list

    def get_target(self, vehID='', routeID=''):
        if len(routeID) > 0:
            route = traci.route.getEdges(routeID)
        else:
            route = traci.vehicle.getRoute(vehID)
        return route[len(route) - 1]

    def _is_along_route(self, vehID, direction):
        """whether vehicle is along route on current step

        Args:
            vehID (str): vehicle id
            direction (str): direction which vehicle will go

        Returns:
            bool: whether vehicle is alogn route on current step
        """
        is_junction = self._is_junction(vehID)
        is_start = self._is_start(vehID)
        if not is_junction or is_start:
            return direction == self._direction_list[0]
        route = traci.vehicle.getRoute(vehID)
        route_index = traci.vehicle.getRouteIndex(vehID) + 1
        if route_index >= len(route):
            return False
        to_edgeID = route[route_index]
        cur_edgeID = route[route_index - 1]
        direct = self._graph.getNextInfoDirect(
            curEdgeID=cur_edgeID, toEdgeID=to_edgeID)
        return direct == direction

    def _is_junction(self, vehID):
        """whether vehicle is on junction on current step

        Args:
            vehID (str): vehicle id

        Returns:
            bool: whether vehicle is on junction on current step
        """
        cur_edgeID = traci.vehicle.getRoadID(vehID)
        if self._graph.getEdgeIndex(cur_edgeID) == -1:
            return True
        return False

    def _is_start(self, vehID):
        """whether vehicle has been moved

        Args:
            vehID (str): vehicle id

        Returns:
            bool: whether vehicle has been moved
        """
        distance = traci.vehicle.getDistance(vehID)
        return distance <= 0.0

    def _could_reach_junction(self, vehID):
        """whether vehicle reach junction until next step

        Args:
            vehID (str): vehicle id

        Returns:
            bool, str: whether vehicle reach junction til next step,
                       if current lane is junction lane, next lane id on road
                       else current lane id
        """
        cur_lane_pos = traci.vehicle.getLanePosition(vehID)
        cur_laneID = traci.vehicle.getLaneID(vehID)
        road_length = traci.lane.getLength(cur_laneID)
        if self._is_junction(vehID):
            route = traci.vehicle.getRoute(vehID)
            route_index = traci.vehicle.getRouteIndex(vehID)
            route_index += (1 if self._is_start(vehID) else 0)
            if route_index >= len(route):
                return False, cur_laneID
            cur_laneID = route[route_index] + '_0'
            road_length += traci.lane.getLength(cur_laneID)
        cur_speed = traci.vehicle.getSpeed(vehID)
        future_lane_pos = cur_lane_pos + cur_speed * self._step_length
        return future_lane_pos > road_length, cur_laneID

    def _could_turn(self, vehID, direction):
        """whether vehicle could turn to the direction

        Args:
            vehID (str): vehicle id
            direction (str): the direction which vehicle will turn to

        Returns:
            bool, str or None: whether vehicle could turn to the direction,
                               next edge id which
                               if vehicle could turn, vehicle will reach
        """
        is_along_route = self._is_along_route(vehID, direction)
        if self._is_junction(vehID) and not is_along_route:
            return False, None
        could_reach, cur_laneID = self._could_reach_junction(vehID)
        if not could_reach:
            return is_along_route, None
        cur_edgeID = traci.lane.getEdgeID(cur_laneID)
        next_edgeID = self._graph.getNextInfoTo(cur_edgeID, direction)
        return next_edgeID is not None, next_edgeID

    def turn(self, vehID, direction):
        """turn to the direction

        Args:
            vehID (str): vehicle id
            direction (str): the direction which vehicle will turn to

        Returns:
            bool: whether vehicle could turn
        """
        could_turn, next_edgeID = self._could_turn(vehID, direction)
        if not could_turn:
            return False
        if next_edgeID is None:
            return True
        # move indirectly
        target_edgeID = self.get_target(vehID)
        new_route = traci.simulation.findRoute(next_edgeID, target_edgeID)
        if len(new_route.edges) == 0:
            print(vehID, "empty")
            traci.vehicle.changeTarget(vehID, next_edgeID)
        else:
            if self._is_junction(vehID):
                new_edge_list = new_route.edges
            else:
                cur_edgeID = traci.vehicle.getRoadID(vehID)
                new_edge_list = [cur_edgeID]
                new_edge_list[
                    len(new_edge_list):len(new_route.edges)] = new_route.edges
            traci.vehicle.setRoute(vehID, new_edge_list)
        return True

    def _get_route_length(self, vehID):
        route = traci.vehicle.getRoute(vehID)
        length = 0
        num = len(route)
        for i, edgeID in enumerate(route):
            laneID = edgeID + '_0'
            length += traci.lane.getLength(laneID)
            if i < num - 1:
                next_edgeID = route[i + 1]
                via_laneID = self._graph.getNextInfoVia(
                    edgeID, toEdgeID=next_edgeID)
                length += traci.lane.getLength(via_laneID)
        return length

    def _is_exist(self, pos, r, target):
        posx, posy = pos

        def distance(targetpos):
            x, y = targetpos
            return np.sqrt((posx - x)**2 + (posy - y)**2)

        dis = distance(target)
        relative_pos = [x - y for (x, y) in zip(target, pos)]
        return dis <= r, relative_pos

    def _get_neighbor(self, pos_list, central_index, visible_range):
        neighbor_list = []
        central_pos = pos_list[central_index]
        for i, pos in enumerate(pos_list):
            if i == central_index:
                continue
            x, y = pos
            if x == -np.inf or y == -np.inf:
                isexist, dis = False, np.inf
            else:
                isexist, dis = self._is_exist(central_pos, visible_range, pos)
            if isexist:
                tmp = [i, dis]
                neighbor_list.append(tmp)
        return neighbor_list

    def _get_neighbor_list(self, pos_list, visible_range):
        neighbor_list = []
        for index, pos in enumerate(pos_list):
            x, y = pos
            if x != -np.inf and y != -np.inf:
                tmp = self._get_neighbor(pos_list, index, visible_range)
            else:
                tmp = []
            neighbor_list.append(tmp)
        return neighbor_list
