import numpy as np
import copy
import networkx as nx
from matplotlib import pyplot as plt
from collections import defaultdict

import torch
from torch_geometric.data import Data
from torch_geometric.utils import to_networkx

from gym_sumo.envs._util import flatten_list, calc_distance
from gym_sumo.envs.sumo_graph import SumoGraph

# category variable(one hot encoding)
# for node
NODE_CATEGORY = [[0, 0], [0, 1], [1, 0], [1, 1]]
JUNCTION_CATEGORY = 0
VEHICLE_CATEGORY = 1
NONE_VEHICLE_JUNCTION_CATEGORY = 2
# for edge
EDGE_CATEGORY = [0, 1]
ROAD_CATEGORY = 0
NONE_ROAD_CATEGORY = 1

NUM_KEY = ["edge_normal", "junction", "lane", "edge", "node"]
EDGE_NORMAL_NUM = 0
JUNCTION_NUM = 1
LANE_NUM = 2
EDGE_NUM = 3
NODE_NUM = 4


class Graph:
    def __init__(self, filepath, is_node_attr=True, is_edge_attr=True):
        self._filepath = filepath
        self.sumo_network = SumoGraph(filepath, withInternal=True)
        self._num = defaultdict(int)
        self._num[NUM_KEY[EDGE_NORMAL_NUM]] = len(
            self.sumo_network.get_all_edgeID_list(False)
        )
        self._num[NUM_KEY[LANE_NUM]] = self._num[NUM_KEY[EDGE_NORMAL_NUM]]
        self._num[NUM_KEY[JUNCTION_NUM]] = len(self.sumo_network.get_all_nodeID_list())
        self._num[NUM_KEY[EDGE_NUM]] = len(self.sumo_network.get_all_edgeID_list(True))
        self._node_list = []
        self._is_edge_attr = is_edge_attr
        self._is_node_attr = is_node_attr
        self._is_direction = True
        self._graph = Data()
        self._ini_graph = Data()
        # create graph
        self._define_graph()

    def get_graph(self):
        return self._graph

    def set_graph(self, graph):
        self._graph = graph

    def reset_graph(self):
        self._graph = copy.deepcopy(self._ini_graph)

    def get_num(self, key):
        """get the number of the key

        Args:
            key (string): the key of the number

        Returns:
            integer: number
        """
        tmp = self._num
        value = -1 if key not in tmp else tmp[key]
        return value

    def get_vehicle_index_in_node_list(self, vehID):
        if vehID not in self._node_list:
            return -1
        num_node = len(self.sumo_network.get_all_nodeID_list())
        return num_node + self._node_list.index(vehID)

    def _add_vehID_in_node_list(self, vehID):
        if vehID not in self._node_list:
            self._node_list.append(vehID)
            self._num[NUM_KEY[NODE_NUM]] += 1

    def get_graph_info_from_vehinfo(self, vehinfo):
        is_exist = vehinfo.get("exist", False)
        vehID = vehinfo.get("ID", "veh0")
        from_node_index = self.get_vehicle_index_in_node_list(vehID)
        if is_exist:
            # add edge to graph
            cur_edgeID = vehinfo.get("cur_edgeID", "")
            cur_edge_index = self.sumo_network.get_edge_index(cur_edgeID)
            next_edgeID = vehinfo.get("next_edgeID", "")
            to_nodeID = self.sumo_network.get_to_nodeID(cur_edgeID)
            to_node_index = self.sumo_network.get_node_index(to_nodeID)
            if cur_edge_index == -1:
                original_node_index = to_node_index
                to_nodeID = self.sumo_network.get_to_nodeID(next_edgeID)
                to_node_index = self.sumo_network.get_node_index(to_nodeID)
                cur_edge_index = self.sumo_network.get_edge_index(next_edgeID)
                if to_node_index == -1:
                    to_node_index = original_node_index
        else:
            to_node_index = from_node_index
            cur_edge_index = -1
        edge_index_info = [from_node_index, to_node_index]
        pos_info = list(vehinfo.get("pos", [0.0, 0.0])) if is_exist else [0.0, 0.0]
        x_info, edge_attr_info = [], []
        if self._is_node_attr:
            category_val = (
                VEHICLE_CATEGORY if is_exist else NONE_VEHICLE_JUNCTION_CATEGORY
            )
            category = NODE_CATEGORY[category_val]
            speed = vehinfo.get("speed", 0.0) if is_exist else 0.0
            x_info = flatten_list([speed, category])
        tmp = list(map(list, self._graph.edge_attr.numpy()))
        if self._is_edge_attr:
            speed = 0.0 if cur_edge_index < 0 else tmp[cur_edge_index][0]
            # calculation edge(veh->tonode) length
            to_pos = self._graph.pos.numpy()[to_node_index]
            length = calc_distance(np.array(pos_info), to_pos)
            category_val = NONE_ROAD_CATEGORY if cur_edge_index < 0 else ROAD_CATEGORY
            category = EDGE_CATEGORY[category_val]
            edge_attr_info = flatten_list([speed, length, category])
        return edge_index_info, pos_info, x_info, edge_attr_info

    def update_vehicle_info_in_node(self, vehinfo):
        graph_info = self.get_graph_info_from_vehinfo(vehinfo)
        edge_index_info, pos_info, x_info, edge_attr_info = graph_info
        from_node_index, to_node_index = edge_index_info
        self._graph.edge_index[1][from_node_index] = to_node_index
        self._graph.pos[from_node_index] = torch.tensor(pos_info, dtype=torch.float)
        if len(x_info) > 0:
            self._graph.x[from_node_index] = torch.tensor(x_info, dtype=torch.float)
        # add feature of edge(veh-> tonode)
        if len(edge_attr_info) > 0:
            self._graph.edge_attr[from_node_index] = torch.tensor(
                edge_attr_info, dtype=torch.float
            )

    # add vehicle as node
    def add_vehicle_in_node(self, vehinfo):
        vehID = vehinfo.get("ID", "veh0")
        # add vehicleID as nodeID
        self._add_vehID_in_node_list(vehID)
        graph_info = self.get_graph_info_from_vehinfo(vehinfo)
        edge_index_info, pos_info, x_info, edge_attr_info = graph_info
        from_node_index, to_node_index = edge_index_info
        tmp_edge_index_list = list(map(list, self._graph.edge_index.numpy()))
        tmp_edge_index_list[0].append(from_node_index)
        tmp_edge_index_list[1].append(to_node_index)
        self._graph.edge_index = torch.tensor(tmp_edge_index_list, dtype=torch.long)
        # add position of vehicle as node position
        tmp_pos_list = list(map(list, self._graph.pos.numpy()))
        tmp_pos_list.append(pos_info)
        self._graph.pos = torch.tensor(tmp_pos_list, dtype=torch.float)
        # add feature of vehicle as node
        if len(x_info) > 0:
            tmp_x_list = list(map(list, self._graph.x.numpy()))
            tmp_x_list.append(x_info)
            self._graph.x = torch.tensor(tmp_x_list, dtype=torch.float)
        # add feature of edge(veh-> tonode)
        tmp_edge_attr_list = list(map(list, self._graph.edge_attr.numpy()))
        if len(edge_attr_info) > 0:
            tmp_edge_attr_list.append(edge_attr_info)
            self._graph.edge_attr = torch.tensor(tmp_edge_attr_list, dtype=torch.float)

    def _get_feature_node(self, nodeID):
        speed = 0.0
        category = NODE_CATEGORY[JUNCTION_CATEGORY]
        return flatten_list([speed, category])

    def _get_pos_node(self, nodeID):
        node_obj = self.sumo_network.get_node_obj(nodeID)
        if node_obj is None:
            return []
        return list(node_obj.getCoord())

    def _generate_node(self):
        pos_list, x_list = [], []
        all_nodeID_list = self.sumo_network.get_all_nodeID_list()
        for nodeID in all_nodeID_list:
            pos_list.append(self._get_pos_node(nodeID))
            x_list.append(self._get_feature_node(nodeID))
        x = torch.tensor(x_list, dtype=torch.float)
        pos = torch.tensor(pos_list, dtype=torch.float)
        return pos, x

    def _get_feature_edge(self, edgeID):
        edge_obj = self.sumo_network.get_edge_obj(edgeID)
        if edge_obj is None:
            return []
        speed = float(edge_obj.getSpeed())
        length = float(edge_obj.getLength())
        category = float(EDGE_CATEGORY[ROAD_CATEGORY])
        return flatten_list([speed, length, category])

    def _convert_nodeID_int(self, edgeID):
        """conversion nodeID's type from string to int

        Args:
            edgeID (str): target edge id to get network of node

        Returns:
            int, int: hash code of nodeID
        """
        # get nodeID(string)
        from_nodeID = self.sumo_network.get_from_nodeID(edgeID)
        to_nodeID = self.sumo_network.get_to_nodeID(edgeID)

        # nodeID(string) -> node_index(int)
        from_node_index = self.sumo_network.get_node_index(from_nodeID)
        to_node_index = self.sumo_network.get_node_index(to_nodeID)
        return from_node_index, to_node_index

    def _generate_edge(self):
        from_node_list, to_node_list, x_list = [], [], []
        normal_edgeID_list = self.sumo_network.get_all_edgeID_list(False)
        for edgeID in normal_edgeID_list:
            # generate edge_index
            from_node_index, to_node_index = self._convert_nodeID_int(edgeID)
            from_node_list.append(from_node_index)
            to_node_list.append(to_node_index)
            # get feature of edge([speed, length, category])
            x_list.append(self._get_feature_edge(edgeID))
        edge_attr = torch.tensor(x_list, dtype=torch.float)
        edge_index = torch.tensor([from_node_list, to_node_list], dtype=torch.long)
        return edge_index, edge_attr

    def _define_graph(self):
        # create graph
        edge_index, edge_attr = self._generate_edge()
        pos, x = self._generate_node()
        self._graph = Data(pos=pos, edge_index=edge_index)
        self._graph.x = x if self._is_node_attr else None
        self._graph.edge_attr = edge_attr if self._is_edge_attr else None
        self._ini_graph = copy.deepcopy(self._graph)

    def get_position_tuple(self, data):
        if data.pos is not None:
            node_position_tuple = {}
            for i, element in enumerate(data.pos):
                node_position_tuple[i] = tuple(element.numpy())
        return node_position_tuple

    def show_graph(self, data, is_pos=True, title="test", is_label=False):
        # networkxのグラフに変換
        nxg = to_networkx(data)
        # print(list(nxg.edges))
        # 可視化のためのページランク計算
        pr = nx.pagerank(nxg)
        pr_max = np.array(list(pr.values())).max()

        # 可視化する際のノード位置
        pos = self.get_position_tuple(data)
        draw_pos = nx.spring_layout(nxg, pos=pos, seed=0)

        cmap = plt.get_cmap("tab10")
        labels = np.zeros(data.num_nodes)
        if is_label:
            # ノードの色設定
            labels = data.y.numpy()
            print(labels)
        colors = [cmap(label) for label in labels]

        # 図のサイズ
        plt.figure(figsize=(100, 100))

        # 描画
        if is_pos:
            nx.draw(nxg, pos, with_labels=True)
        else:
            nx.draw_networkx_nodes(
                nxg,
                draw_pos,
                node_size=[v / pr_max * 1000 for v in pr.values()],
                node_color=colors,
                alpha=0.5,
            )
            nx.draw_networkx_edges(nxg, draw_pos, arrowstyle="-", alpha=0.2)
            nx.draw_networkx_labels(nxg, draw_pos, font_size=10)

        plt.title(title)
        plt.show()

    def check_graph(self, data):
        """グラフ情報を表示"""
        print("グラフ構造:", data)
        print("グラフのキー: ", data.keys)
        print("ノード数:", data.num_nodes)
        print("エッジ数:", data.num_edges)
        print("ノードの特徴量数:", data.num_node_features)
        print("孤立したノードの有無:", data.contains_isolated_nodes())
        print("自己ループの有無:", data.contains_self_loops())
        print("====== ノードの特徴量:x ======")
        print(data["x"])
        print("====== ノードのクラス:y ======")
        print(data["y"])
        print("====== ノードの座標:pos ======")
        print(data["pos"])
        print("========= エッジ形状:edge_index =========")
        print(data["edge_index"])
        print("========= エッジの特徴量:edge_attr =========")
        print(data["edge_attr"])


if __name__ == "__main__":
    import os
    import sumolib

    # from IPython import embed
    import xml.etree.ElementTree as ET

    sumoBinary = os.path.join(
        os.path.dirname(__file__), "sumo_configs/nishiwaseda/osm.net.xml"
    )

    def test_func(test_str):
        return test_str, [] if test_str == "" else [test_str]

    test = [""]
    test2 = list(map(test_func, test))

    test_edge_list = ["98726975", "415221603#10"]
    test_sub = test_edge_list[: len(test_edge_list) - 1]
    tree = ET.parse(sumoBinary)
    root = tree.getroot()
    print(root, type(root))
    # for child in root:
    #     if child.tag == "edge":
    #         print(child.attrib, type(child.attrib))
    # type(child.attrib): <class 'dict'>
    # print(root)
    g = Graph(sumoBinary)
    graph = g.get_graph()
    # print(graph.edge_attr.numpy())
    # print(graph.pos.numpy())
    print(len(g.sumo_network.get_all_edgeID_list(False)))

    kwargs = {"withInternal": True}
    net = sumolib.net.readNet(sumoBinary, **kwargs)
    # netreader = sumolib.net.NetReader(**kwargs)
    # try:
    #     parse(gzip.open(sumoBinary), netreader)
    # except IOError:
    #     parse(sumoBinary, netreader)
    edge_objs = net.getEdges()
    edge_obj = edge_objs[0]
    edge_obj_last = edge_objs[len(edge_objs) - 1]
    print(edge_obj, type(edge_obj))
    edgeID = edge_obj.getID()
    index = g.sumo_network.get_edge_index(edgeID)
    connections = edge_obj.getConnections(edge_obj_last)
    outgoing = edge_obj.getOutgoing()
    outgoing_last = edge_obj_last.getOutgoing()
    connections_last = edge_obj_last.getConnections(list(outgoing_last)[0])
    print(outgoing, connections_last)
    last_edgeID = edge_obj_last.getID()
    edge2 = net.getEdge(edgeID)

    g.check_graph(graph)
    print(g.get_num("edge_normal"))
    next_edgeID_list = g.sumo_network.get_next_edgeIDs(edgeID)
    print(next_edgeID_list)
    # g.show_graph(graph, title="sumo", is_pos=False, is_label=False)
    # g.show_graph(graph, title="sumo", is_pos=True, is_label=False)
    v_info = {}
    v_info["ID"] = "veh0"
    v_info["pos"] = [1.0, 10.0]
    v_info["speed"] = [2.6]
    v_info["exist"] = True
    v_info["cur_edgeID"] = edgeID
    v_info["next_edgeID"] = next_edgeID_list[0]
    g.add_vehicle_in_node(v_info)
    graph = g.get_graph()
    g.show_graph(graph, title="sumo_add", is_pos=True, is_label=False)
    v_info["pos"] = [3.6, 10.0]
    g.update_vehicle_info_in_node(v_info)
    graph = g.get_graph()
    g.show_graph(graph, title="sumo_update", is_pos=True, is_label=False)
