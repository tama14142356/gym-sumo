import sumolib


class SumoGraph:
    """load a .net.xml file and store network info class
    The following named options are supported on kwargs:

    "net" : initialize data structurs with an existing net object (default Net())
    "withPrograms" : import all traffic light programs (default False)
    "withLatestPrograms" : import only the last program for each traffic light.
                            This is the program that would be active in sumo by default.
                            (default False)
    "withConnections" : import all connections (default True)
    "withFoes" : import right-of-way information (default True)
    "withInternal" : import internal edges and lanes (default False)
    "withPedestrianConnections" : import connections between sidewalks, crossings
                                  (default False)
    """

    def __init__(self, filepath, **kwargs):
        self.network = sumolib.net.readNet(filepath, **kwargs)

    def get_all_edge_dict(self, with_internal=True):
        return self.network.getEdges(withInternal=with_internal)

    def get_all_edge_list(self, with_internal=True):
        return list(self.get_all_edge_dict(with_internal))

    def get_all_edgeID_list(self, with_internal=True):
        all_edge_list = self.get_all_edge_list(with_internal)
        return list(map(lambda edge: edge.getID(), all_edge_list))

    def get_edge_obj(self, edgeID):
        if not self.network.hasEdge(edgeID):
            return None
        return self.network.getEdge(edgeID)

    def get_edgeID(self, edge_index, is_normal=True):
        all_edgeID_list = self.get_all_edgeID_list((not is_normal))
        edge_num = len(all_edgeID_list)
        edgeID = "" if edge_index >= edge_num else all_edgeID_list[edge_index]
        return edgeID

    def get_edge_index(self, edgeID, is_normal=True):
        all_edgeID_list = self.get_all_edgeID_list((not is_normal))
        index = -1 if edgeID not in all_edgeID_list else all_edgeID_list.index(edgeID)
        return index

    def get_laneID(self, edgeID, lane_index):
        if not self.network.hasEdge(edgeID):
            return ""
        edge_obj = self.network.getEdge(edgeID)
        return edge_obj.getLane(lane_index).getID()

    def is_internal_edgeID(self, edgeID):
        if not self.network.hasEdge(edgeID):
            return False
        edge_obj = self.network.getEdge(edgeID)
        # is_special = edge_obj.isSpecial()
        is_internal = edge_obj.getFunction() == "internal"
        return is_internal

    def get_all_node_dict(self):
        return self.network.getNodes()

    def get_all_node_list(self):
        return list(self.get_all_node_dict())

    def get_all_nodeID_list(self):
        all_node_list = self.get_all_node_list()
        return list(map(lambda node: node.getID(), all_node_list))

    def get_node_obj(self, nodeID):
        if not self.network.hasNode(nodeID):
            return None
        return self.network.getNode(nodeID)

    def get_nodeID(self, node_index):
        all_nodeID_list = self.get_all_nodeID_list()
        node_num = len(all_nodeID_list)
        nodeID = "" if node_index >= node_num else all_nodeID_list[node_index]
        return nodeID

    def get_to_nodeID(self, edgeID):
        if not self.network.hasEdge(edgeID):
            return ""
        edge_obj = self.network.getEdge(edgeID)
        return edge_obj.getToNode().getID()

    def get_from_nodeID(self, edgeID):
        if not self.network.hasEdge(edgeID):
            return ""
        edge_obj = self.network.getEdge(edgeID)
        return edge_obj.getFromNode().getID()

    def get_node_index(self, nodeID):
        all_nodeID_list = self.get_all_nodeID_list()
        index = -1 if nodeID not in all_nodeID_list else all_nodeID_list.index(nodeID)
        return index

    def _get_next_edge_dict(self, cur_edgeID):
        if not self.network.hasEdge(cur_edgeID):
            return {}
        cur_edge_obj = self.network.getEdge(cur_edgeID)
        return cur_edge_obj.getOutgoing()

    def get_next_edge_dict(self, cur_edgeID, cur_lane_index=0):
        _next_edge_dict = self._get_next_edge_dict(cur_edgeID)
        next_edge_dict = {}
        for next_edge_obj, connections in _next_edge_dict.items():
            tmp = [
                c for c in connections if c.getFromLane().getIndex() == cur_lane_index
            ]
            if len(tmp) > 0:
                next_edge_dict[next_edge_obj] = tmp
        return next_edge_dict

    def get_next_edgeIDs(self, cur_edgeID, cur_lane_index=0):
        next_edge_dict = self.get_next_edge_dict(cur_edgeID, cur_lane_index)
        return list(map(lambda element: element.getID(), next_edge_dict))

    def get_next_directions(self, cur_edgeID, cur_lane_index=0):
        next_edge_dict = self.get_next_edge_dict(cur_edgeID, cur_lane_index)
        return list(
            map(lambda key: next_edge_dict[key][0].getDirection(), next_edge_dict)
        )

    def get_next_edgeID_info(self, cur_edgeID, next_edgeID, cur_lane_index=-1):
        net = self.network
        if not net.hasEdge(cur_edgeID) or not net.hasEdge(next_edgeID):
            return []
        cur_edge_obj = net.getEdge(cur_edgeID)
        next_edge_obj = net.getEdge(next_edgeID)
        connections = cur_edge_obj.getConnections(next_edge_obj)
        if cur_lane_index < 0:
            return connections
        return [c for c in connections if c.getFromLane().getIndex() == cur_lane_index]

    def get_next_edgeID_direction(self, cur_edgeID, next_edgeID, cur_lane_index=0):
        conn_info = self.get_next_edgeID_info(cur_edgeID, next_edgeID, cur_lane_index)
        if len(conn_info) > 0:
            return conn_info[0].getDirection()
        return ""

    def get_next_edgeID_to_lane(self, cur_edgeID, next_edgeID, cur_lane_index=0):
        conn_info = self.get_next_edgeID_info(cur_edgeID, next_edgeID, cur_lane_index)
        if len(conn_info) > 0:
            return conn_info[0].getToLane().getIndex()
        return -1

    def get_next_edgeID(self, cur_edgeID, next_direction, cur_lane_index=0):
        next_edge_dict = self.get_next_edge_dict(cur_edgeID, cur_lane_index)
        for next_edge_obj, connection_list in next_edge_dict.items():
            for connection in connection_list:
                direction = connection.getDirection()
                if next_direction == direction:
                    return next_edge_obj.getID()
        return ""

    def _get_from_edge_dict(self, cur_edgeID):
        if not self.network.hasEdge(cur_edgeID):
            return {}
        cur_edge_obj = self.network.getEdge(cur_edgeID)
        return cur_edge_obj.getIncoming()

    def get_from_edge_dict(self, cur_edgeID, cur_lane_index=0):
        _from_edge_dict = self._get_from_edge_dict(cur_edgeID)
        from_edge_dict = {}
        for from_edge_obj, connections in _from_edge_dict.items():
            tmp = [c for c in connections if c.getToLane().getIndex() == cur_lane_index]
            if len(tmp) > 0:
                from_edge_dict[from_edge_obj] = tmp
        return from_edge_dict

    def get_from_edgeIDs(self, cur_edgeID, cur_lane_index=0):
        from_edge_dict = self.get_from_edge_dict(cur_edgeID, cur_lane_index)
        return list(map(lambda element: element.getID(), from_edge_dict))

    def get_from_directions(self, cur_edgeID, cur_lane_index=0):
        from_edge_dict = self.get_from_edge_dict(cur_edgeID, cur_lane_index)
        return list(
            map(lambda key: from_edge_dict[key][0].getDirection(), from_edge_dict)
        )

    def get_from_edgeID_info(self, cur_edgeID, from_edgeID, cur_lane_index=-1):
        net = self.network
        if not net.hasEdge(cur_edgeID) or not net.hasEdge(from_edgeID):
            return []
        cur_edge_obj = net.getEdge(cur_edgeID)
        from_edge_obj = net.getEdge(from_edgeID)
        connections = from_edge_obj.getConnections(cur_edge_obj)
        if cur_lane_index < 0:
            return connections
        return [c for c in connections if c.getToLane().getIndex() == cur_lane_index]

    def get_from_edgeID_direction(self, cur_edgeID, from_edgeID, cur_lane_index=0):
        conn_info = self.get_from_edgeID_info(cur_edgeID, from_edgeID, cur_lane_index)
        if len(conn_info) > 0:
            return conn_info[0].getDirection()
        return ""

    def get_from_edgeID_from_lane(self, cur_edgeID, from_edgeID, cur_lane_index=0):
        conn_info = self.get_from_edgeID_info(cur_edgeID, from_edgeID, cur_lane_index)
        if len(conn_info) > 0:
            return conn_info[0].getFromLane().getIndex()
        return -1

    def get_from_edgeID(self, cur_edgeID, from_direction, cur_lane_index=0):
        from_edge_dict = self.get_from_edge_dict(cur_edgeID, cur_lane_index)
        for from_edge_obj, connection_list in from_edge_dict.items():
            for connection in connection_list:
                direction = connection.getDirection()
                if from_direction == direction:
                    return from_edge_obj.getID()
        return ""


if __name__ == "__main__":
    import os

    sumoBinary = os.path.join(
        os.path.dirname(__file__), "sumo_configs/nishiwaseda/osm.net.xml"
    )
    kwargs = {"withInternal": True}
    sumo_network = SumoGraph(sumoBinary, **kwargs)
    # edgeID = "98726975"
    edgeID = "415221603#10"
    all_edgeID_list = sumo_network.get_all_edgeID_list()
    # edgeID = all_edgeID_list[0]
    next_edgeID_list = sumo_network.get_next_edgeIDs(edgeID)
    # print(next_edgeID_list, type(next_edgeID_list[0]))
    next_edgeID_list_lane_1 = sumo_network.get_next_edgeIDs(edgeID, 1)
    # print(next_edgeID_list_lane_1)
    next_direction_list = sumo_network.get_next_directions(edgeID, 1)
    # print(next_direction_list, type(next_direction_list[0]))
    next_edgeID_dict = sumo_network.get_next_edge_dict(edgeID, 0)
    # print(next_edgeID_dict)
    # for edge, conn in next_edgeID_dict.items():
    #     print(list(map(str, conn)), edge)
    next_edgeID = next_edgeID_list_lane_1[0]
    next_edgeID_info = sumo_network.get_next_edgeID_info(edgeID, next_edgeID, 1)
    # # print(next_edgeID_info, list(map(str, next_edgeID_info)))
    from_edge_dict = sumo_network.get_from_edge_dict(edgeID, 1)
    # print(from_edge_dict)
    # for edge, conn in from_edge_dict.items():
    #     print(list(map(str, conn)), edge)
    from_direction_list = sumo_network.get_from_directions(edgeID)
    # print(from_direction_list)
    from_edgeID = sumo_network.get_from_edgeID(edgeID, "s", 0)
    # print(from_edgeID)
