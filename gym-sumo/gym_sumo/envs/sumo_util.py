import numpy as np
from gym import error

try:
    import traci
    from traci.exceptions import TraCIException
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))


class SumoUtil:

    def __init__(self, graph, step_length):
        self._graph = graph
        self._step_length = step_length

    def _is_junction(self, vehID):
        cur_edgeID = traci.vehicle.getRoadID(vehID)
        if self._graph.getEdgeIndex(cur_edgeID) == -1:
            return True
        return False

    def _could_turn(self, vehID, future_speed, direction=None):
        cur_lane_pos = traci.vehicle.getLanePosition(vehID)
        cur_laneID = traci.vehicle.getLaneID(vehID)
        road_length = traci.lane.getLength(cur_laneID)
        if self._is_junction(vehID):
            # 基本的に交差点にすでにいる場合は今から向きを変えるのは不可能という意味でreject
            route = traci.vehicle.getRoute(vehID)
            distance = traci.vehicle.getDistance(vehID)
            if distance > road_length:
                return False
            # 初期位置なので、この交差点もはじめのedgeの一部とみなす。
            route_index = traci.vehicle.getRouteIndex(vehID)
            laneID = route[route_index] + '_0'
            road_length += traci.lane.getLength(laneID)
        future_lane_pos = cur_lane_pos + future_speed * self._step_length
        if future_lane_pos > road_length:
            return True
        return False

    def turn(self, vehID, direction, cur_edgeID=None):
        if cur_edgeID is None:
            cur_edgeID = traci.vehicle.getRoadID(vehID)
        next_edgeID = self._graph.getNextInfoTo(cur_edgeID, direction)
        if next_edgeID is None:
            return False
        else:
            # move indirectly
            route = traci.vehicle.getRoute(vehID)
            route_index = traci.vehicle.getRouteIndex(vehID) + 1
            route_edge_num = len(route)
            if route_index < route_edge_num:
                orig_next_edgeID = route[route_index]
                if next_edgeID == orig_next_edgeID:
                    return True
            target_edgeID = route[route_edge_num - 1]
            new_route = traci.simulation.findRoute(next_edgeID, target_edgeID)
            new_edge_list = [cur_edgeID]
            new_edge_list[
                len(new_edge_list):len(new_route.edges)
            ] = new_route.edges
            if len(new_route.edges) == 0:
                print(vehID, "empty")
                traci.vehicle.changeTarget(vehID, next_edgeID)
                return True
            try:
                traci.vehicle.setRoute(vehID, new_edge_list)
            except TraCIException as tr:
                print(tr)
                return False
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
