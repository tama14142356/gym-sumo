import numpy as np
try:
    import traci
    from traci import constants as tc
    from traci.exceptions import TraCIException
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you can install sumo or set the path for sumo library by reading README.md)".format(e))


class SumoUtil:

    def __init__(self, graph, step_length):
        self._graph = graph
        self._step_length = step_length

    def _is_Junction(self, vehID):
        cur_edgeID = traci.vehicle.getRoadID(vehID)
        if self._graph.getEdgeIndex(cur_edgeID) == -1:
            return True
        return False

    def _could_turn(self, vehID, future_speed, direction=None):
        cur_lane_pos = traci.vehicle.getLanePosition(vehID)
        cur_laneID = traci.vehicle.getLaneID(vehID)
        road_length = traci.lane.getLength(cur_laneID)
        if self._is_Junction(vehID):
            # 基本的に交差点にすでにいる場合は今から向きを変えるのは不可能という意味でreject
            route = traci.vehicle.getRoute(vehID)
            distance = traci.vehicle.getDistance(vehID)
            if distance > road_length:
                return False
            # 初期位置なので、この交差点もはじめのedgeの一部とみなす。
            routeIndex = traci.vehicle.getRouteIndex(vehID)
            laneID = route[routeIndex] + '_0'
            road_length += traci.lane.getLength(laneID)
        future_lane_pos = cur_lane_pos + future_speed * self._step_length
        if future_lane_pos > road_length:
            return True
        return False

    def turn(self, vehID, direction, curEdgeID=None):
        if curEdgeID is None:
            curEdgeID = traci.vehicle.getRoadID(vehID)
        nextEdgeID = self._graph.getNextInfoTo(curEdgeID, direction)
        if nextEdgeID is None:
            return False
        else:
            # move indirectly
            route = traci.vehicle.getRoute(vehID)
            targetEdgeID = route[len(route) - 1]
            newRoute = traci.simulation.findRoute(nextEdgeID, targetEdgeID)
            newEdgeList = [curEdgeID]
            newEdgeList[len(newEdgeList):len(newRoute.edges)] = newRoute.edges
            if len(newRoute.edges) == 0:
                print(vehID, "empty")
                newEdgeList.append(nextEdgeID)
            try:
                traci.vehicle.setRoute(vehID, newEdgeList)
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
