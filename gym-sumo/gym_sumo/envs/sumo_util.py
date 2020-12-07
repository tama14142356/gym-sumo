import numpy as np
import copy

# from IPython import embed  # for debug


class SumoUtil:
    def __init__(self, sumo_network, direction_list, traci_connect):
        self._network = sumo_network
        self._direction_list = direction_list
        self.traci_connect = traci_connect
        self._step_length = self.traci_connect.simulation.getDeltaT()

    def get_routeID_list_from_target(self, target_edgeID):
        routeID_list = self.traci_connect.route.getIDList()
        routeID_list_target = []
        for routeID in routeID_list:
            target = self.get_target(routeID=routeID)
            if target == target_edgeID:
                routeID_list_target.append(routeID)
        return routeID_list_target

    def get_target(self, vehID="", routeID=""):
        route = (
            self.traci_connect.route.getEdges(routeID)
            if len(routeID) > 0
            else self.traci_connect.vehicle.getRoute(vehID)
        )
        return route[len(route) - 1]

    def _get_direction_along_route(self, vehID):
        """get direction when along route

        Args:
            vehID (str): vehicle id

        Returns:
            str: direction
        """
        is_junction = self._is_junction(vehID)
        is_start = self._is_start(vehID)
        if not is_junction or is_start:
            return self._direction_list[0]
        route = self.traci_connect.vehicle.getRoute(vehID)
        route_index = self.traci_connect.vehicle.getRouteIndex(vehID) + 1
        cur_edgeID = self.traci_connect.vehicle.getRoadID(vehID)
        cur_lane_index = self.traci_connect.vehicle.getLaneIndex(vehID)
        if route_index >= len(route):
            return self._network.get_from_direction(
                cur_edgeID, route[len(route) - 1], cur_lane_index
            )
        next_edgeID = route[route_index]
        return self._network.get_next_direction(cur_edgeID, next_edgeID, cur_lane_index)

    def _is_along_route(self, vehID, direction):
        along_route_direction = self._get_direction_along_route(vehID)
        return along_route_direction == direction

    def _is_junction(self, vehID, edgeID=""):
        cur_edgeID = self.traci_connect.vehicle.getRoadID(vehID)
        if len(edgeID) > 0:
            cur_edgeID = edgeID
        return self._network.is_internal_edgeID(cur_edgeID)

    def _is_start(self, vehID):
        distance = self.traci_connect.vehicle.getDistance(vehID)
        return distance <= 0.0

    # cur_laneIDが交差点の場合はnext_edgeIDは取得できるが、via_laneが空になる
    # （自分だから）
    # その場合は、自分自身の長さを返すことにする。
    # その時、via_laneIDはからのまま返す
    def get_via_length(self, cur_laneID, direction):
        if cur_laneID == "":
            return -1.0, "", ""
        cur_edgeID = self.traci_connect.lane.getEdgeID(cur_laneID)
        cur_lane_index = self._network.get_lane_index(cur_laneID)
        next_edgeID = self._network.get_next_edgeID(
            cur_edgeID, direction, cur_lane_index
        )
        if next_edgeID == "":
            return -1.0, "", next_edgeID
        via_laneID = self._network.get_via_laneID(
            cur_edgeID, next_edgeID, cur_lane_index
        )
        laneID = cur_laneID if via_laneID == "" else via_laneID
        _, to_laneID = self._network.get_next_to_lane(
            cur_edgeID, next_edgeID, cur_lane_index
        )
        return self.traci_connect.lane.getLength(laneID), via_laneID, to_laneID

    def get_future_lane_pos(self, vehID, speed=-1.0):
        cur_lane_pos = self.traci_connect.vehicle.getLanePosition(vehID)
        cur_speed = (
            speed if speed >= 0.0 else self.traci_connect.vehicle.getSpeed(vehID)
        )
        future_lane_pos = cur_lane_pos + cur_speed * self._step_length
        return future_lane_pos

    def get_could_reach_laneIDs_of_straight(self, vehID, speed=-1.0):
        net = self._network
        future_lane_pos = self.get_future_lane_pos(vehID, speed)
        direction = self._direction_list[0]
        road_length, cur_laneID = self._current_road_length(vehID)
        if road_length >= future_lane_pos or cur_laneID == "":
            return [] if cur_laneID == "" else [cur_laneID], cur_laneID
        reach_laneIDs = [cur_laneID]
        while True:
            cur_edgeID = self.traci_connect.lane.getEdgeID(cur_laneID)
            cur_lane_index = net.get_lane_index(cur_laneID)
            next_edgeID = net.get_next_edgeID(cur_edgeID, direction, cur_lane_index)
            if next_edgeID == "":
                return [], ""
            to_lane_index, next_laneID = net.get_next_to_lane(
                cur_edgeID, next_edgeID, cur_lane_index
            )
            via_laneID = net.get_via_laneID(
                cur_edgeID, next_edgeID, cur_lane_index, to_lane_index
            )
            reach_laneIDs.append(next_laneID)
            road_length += self.traci_connect.lane.getLength(via_laneID)
            road_length += self.traci_connect.lane.getLength(next_laneID)
            if road_length >= future_lane_pos:
                return reach_laneIDs, next_laneID
            cur_laneID = next_laneID

    def _is_over_turn(self, vehID, direction, speed=-1.0):
        """whether over turn is, and whether could turn to direction

        Args:
            vehID (str): vehicle id
            direction (str): direction to be goning to turn
            speed (float, optional): current speed if vehicle's speed will change.
                                     Defaults to -1.0. It means use getSpeed(vehID)

        Returns:
            bool, list, str: whether over turn is, and whether could turn to direction,
                            list of next_laneID where vehicle reach after turn direction
                            next lane id where vehicle reach after turn direction
                            completely
        """
        if direction == self._direction_list[0]:
            reach_laneIDs, next_laneID = self.get_could_reach_laneIDs_of_straight(
                vehID, speed
            )
            return len(reach_laneIDs) <= 0, reach_laneIDs, next_laneID
        road_length, cur_laneID = self._current_road_length(vehID)
        future_lane_pos = self.get_future_lane_pos(vehID, speed)
        via_length, _, next_laneID = self.get_via_length(cur_laneID, direction)
        if via_length < 0.0:
            return True, [], ""
        road_length += via_length
        return future_lane_pos > road_length, [cur_laneID, next_laneID], next_laneID

    def _current_road_length(self, vehID):
        """calculate length of current road from current road vehicle is on
                                            to road in front of next juncton

        Args:
            vehID (str): vehicle id

        Returns:
            float, str: current road length,
                        next lane id if vehicle is on junction
                        else current lane id
        """
        cur_laneID = self.traci_connect.vehicle.getLaneID(vehID)
        road_length = self.traci_connect.lane.getLength(cur_laneID)
        route_index = self.traci_connect.vehicle.getRouteIndex(vehID)
        if self._is_junction(vehID):
            route = self.traci_connect.vehicle.getRoute(vehID)
            route_index += 0 if self._is_start(vehID) else 1
            if route_index >= len(route):
                return road_length, ""
            cur_edgeID = self.traci_connect.vehicle.getRoadID(vehID)
            cur_lane_index = self.traci_connect.vehicle.getLaneIndex(vehID)
            to_lane_index, cur_laneID = self._network.get_next_to_lane(
                cur_edgeID, route[route_index], cur_lane_index
            )
            laneID = self._network.get_laneID(route[route_index])
            laneID = laneID if cur_laneID == "" else cur_laneID
            road_length += self.traci_connect.lane.getLength(laneID)
        return road_length, cur_laneID

    def _could_reach_junction(self, vehID, speed=-1.0):
        road_length, cur_laneID = self._current_road_length(vehID)
        future_lane_pos = self.get_future_lane_pos(vehID, speed)
        return future_lane_pos > road_length, cur_laneID

    def _could_turn(self, vehID, direction, speed=-1.0):
        """whether vehicle could turn to the direction

        Args:
            vehID (str): vehicle id
            direction (str): the direction which vehicle will turn to
            speed (float, optional): vehicle speed
                                     Defaults to -1.0 means not use this speed

        Returns:
            bool, str or None: whether vehicle could turn to the direction,
                               next edge id which
                               if vehicle could turn, vehicle will reach
        """
        is_along_route = self._is_along_route(vehID, direction)
        is_junction = self._is_junction(vehID)
        if is_junction and not is_along_route:
            return False, "", []
        could_reach, cur_laneID = self._could_reach_junction(vehID, speed)
        if not could_reach or cur_laneID == "":
            return is_along_route, "", []
        target_edgeID = self.get_target(vehID)
        if target_edgeID == self.traci_connect.lane.getEdgeID(cur_laneID):
            return True, "", []
        is_over, reach_laneIDs, to_laneID = self._is_over_turn(vehID, direction, speed)
        if is_over:
            return False, "", []
        to_edgeID = self.traci_connect.lane.getEdgeID(to_laneID)
        reach_edgeIDs = list(map(self.traci_connect.lane.getEdgeID, reach_laneIDs))
        return True, to_edgeID, reach_edgeIDs

    def turn(self, vehID, direction, speed=-1.0):
        """turn to the direction

        Args:
            vehID (str): vehicle id
            direction (str): the direction which vehicle will turn to
            speed (float, optional): vehicle speed
                                     Defaults to -1.0 means not use this speed

        Returns:
            bool: whether vehicle could turn
        """
        turn_info = self._could_turn(vehID, direction, speed)
        could_turn, next_edgeID, reach_edgeIDs = turn_info
        if not could_turn:
            return False
        # not reach next junction, but, will move along direction or arrive target
        if next_edgeID == "":
            return True
        # move indirectly
        target_edgeID = self.get_target(vehID)
        new_route = self.traci_connect.simulation.findRoute(next_edgeID, target_edgeID)
        new_edge_list = reach_edgeIDs
        if len(new_route.edges) > 0:
            new_edge_list = reach_edgeIDs[: len(reach_edgeIDs) - 1]
            new_edge_list[len(new_edge_list) : len(new_route.edges)] = new_route.edges
        self.traci_connect.vehicle.setRoute(vehID, new_edge_list)
        return True

    def _get_route_info(self, vehID="", routeID="", route_edges=[], route_info_list={}):
        route = []
        if len(route_edges) > 0:
            route = copy.deepcopy(route_edges)
        if len(routeID) > 0:
            if routeID in route_info_list:
                return copy.deepcopy(route_info_list[routeID])
            route = self.traci_connect.route.getEdges(routeID)
        if len(vehID) > 0:
            route = self.traci_connect.vehicle.getRoute(vehID)
        travel_time, length = 0.0, 0.0
        num = len(route)
        for i, edgeID in enumerate(route):
            length += self._network.get_road_length(edgeID)
            # travel_time = length / max_speed on the laneID(traci.lane.getMaxSpeed())
            travel_time += self._network.get_road_travel_time(edgeID)
            if i < num - 1:
                next_edgeID = route[i + 1]
                lane_num = self._network.get_lane_number(edgeID)
                for i in range(lane_num):
                    via_laneID = self._network.get_via_laneID(edgeID, next_edgeID, i)
                    if via_laneID != "":
                        break
                length += self._network.get_road_length(laneID=via_laneID)
                travel_time += self._network.get_road_travel_time(laneID=via_laneID)
        del route
        route_info = {"length": length, "travel_time": travel_time, "cost": travel_time}
        return route_info

    def _is_exist(self, pos, r, target):
        posx, posy = pos

        def distance(targetpos):
            x, y = targetpos
            return np.sqrt((posx - x) ** 2 + (posy - y) ** 2)

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
