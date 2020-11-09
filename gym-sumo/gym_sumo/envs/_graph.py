import xml.etree.ElementTree as ET
import torch
from torch_geometric.data import Data
from collections import defaultdict
import numpy as np

import networkx as nx
from torch_geometric.utils import to_networkx
from matplotlib import pyplot as plt
from gym_sumo.envs._util import flatten_list

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
    def __init__(self, filepath, isNode_attr=True, isEdge_attr=True):
        self.__filepath = filepath
        self.__nodeIDList = []  # list of node id
        # dictionary: {'edgeID':(fromnodeindex, tonodeindex)}
        self.__edgeIDList = []  # list of edgeID including normal, internal
        # self.nodeIDdict = {}  # dictionary: {'nodeID': nodeindex}
        # self.normalEdgeIDdict = {}  # dictionary: {'edgeID': edgeindex}
        self.__edges = None
        self.__num = defaultdict(int)
        self.__isEdge_attr = isEdge_attr
        self.__isNode_attr = isNode_attr
        self.__isDirection = True
        self.__graph = Data()
        # create graph
        self._defineGraph()

    def getGraph(self):
        return self.__graph

    def setGraph(self, graph):
        self.__graph = graph

    def getNum(self, key):
        """get the number of the key

        Args:
            key (string): the key of the number

        Returns:
            integer: number
        """
        tmp = self.__num
        value = -1 if key not in tmp else tmp[key]
        return value

    def getEdgeIndex(self, edgeID, isNormal=True):
        index = -1
        tmpList = self.__edgeIDList
        edge_num = self.getNum(NUM_KEY[EDGE_NORMAL_NUM])
        if edgeID in tmpList:
            index = tmpList.index(edgeID)
        if isNormal:
            if index >= edge_num:
                index = -1
        return index

    def getEdgeID(self, edgeIndex, isNormal=True):
        tmpList = self.__edgeIDList
        ID = None
        edge_num = self.getNum(NUM_KEY[EDGE_NORMAL_NUM])
        if edgeIndex < len(tmpList) and edgeIndex >= 0:
            ID = tmpList[edgeIndex]
        if isNormal:
            if edgeIndex >= edge_num:
                ID = None
        return ID

    def getNodeID(self, nodeIndex, internalEdgeID=None):
        nodeID = None
        tmpList = self.__nodeIDList
        edgeID = internalEdgeID
        if nodeIndex < len(tmpList) and nodeIndex >= 0:
            nodeID = tmpList[nodeIndex]
        if edgeID is not None:
            index = edgeID.rfind("_")
            nodeID = edgeID[1:index]
        return nodeID

    def getNodeIndex(self, nodeID, normalEdgeID=None, isTo=True):
        """get node index

        Args:
            nodeID (string): node ID as registered junction on sumo
            normalEdgeID (string, optional): edge ID as registered road for
                                             getting connected junction ID(node ID).
                                             Defaults to None.
            isTo (bool, optional): whether target node or not (if not true, from node).
                                   Defaults to True.

        Returns:
            string: node index
        """
        index = -1
        tmp = self.__nodeIDList
        if normalEdgeID is not None:
            nodeTuple = self.getNodeTuple(normalEdgeID)
            index = nodeTuple[0]
            if isTo:
                index = nodeTuple[1]
        if nodeID is not None and nodeID in tmp:
            # return self.nodeIDdict[nodeID]
            index = tmp.index(nodeID)
        return index

    def getNodeTuple(self, normalEdgeID):
        nodeTuple = (-1, -1)
        edgeIndex = self.getEdgeIndex(normalEdgeID)
        if edgeIndex != -1:
            tmp = self.__graph.edge_index.numpy()
            fromNodeIndex = tmp[0][edgeIndex]
            toNodeIndex = tmp[1][edgeIndex]
            nodeTuple = (fromNodeIndex, toNodeIndex)
        return nodeTuple

    def getEdges(self):
        if self.__edges is not None:
            return self.__edges
        nxg = to_networkx(self.__graph)
        tmpList = list(nxg.edges)
        self.__edges = []
        for i, tmp in enumerate(tmpList):
            edge = [list(self.__graph.edge_attr[i].numpy()), list(tmp)]
            self.__edges.append(edge)
        return self.__edges

    def getConnection(self):
        return self.__normalEdgeConnection

    def _getProcessInfo(self, curEdgeIndex, toEdgeID):
        tmp = self.__normalEdgeConnection
        ans = [toEdgeID, None, None, None]
        isFinished = False

        # index error or not supported
        if curEdgeIndex >= len(tmp) or curEdgeIndex < 0 or not self.__isDirection:
            isFinished = True

        for lane in tmp[curEdgeIndex]:
            if isFinished:
                break
            for direct in tmp[curEdgeIndex][lane]:
                if isFinished:
                    break
                for edges in tmp[curEdgeIndex][lane][direct]:
                    if edges["to"] == toEdgeID:
                        ans[1] = edges["via"]
                        ans[2] = direct
                        ans[3] = lane
                        isFinished = True
        return ans

    # 基本としてはcurEdgeIndexから接続されているEdgeの情報を返す
    # ただし、現在の車線(curLaneIndex)、行きたい方向(nextDirection)
    # がNoneでないもの全てと一致するものを返す。
    # nextDirectionだけを考慮したい場合は（接続元がわかっていない場合）curLaneIndexをNoneにすれば
    # あくまでedge単位でその方向に道があるか確認できる
    # 指定した車線から行けるかどうかも考慮に入れるのであれば、curLaneIndexを指定する。
    # nextDirectionがNoneになることは想定していない。つまり、この車線から行ける全てまたは一部のedgeと方向を返すことは想定していない
    def _getNextInfo(self, curEdgeIndex, nextDirection, curLaneIndex=None):
        ans = [None, None, nextDirection, curLaneIndex]
        tmp = self.__normalEdgeConnection
        # index error or not supported
        if curEdgeIndex >= len(tmp) or curEdgeIndex < 0 or not self.__isDirection:
            return ans

        if curLaneIndex in tmp[curEdgeIndex]:
            if nextDirection in tmp[curEdgeIndex][curLaneIndex]:
                edges = tmp[curEdgeIndex][curLaneIndex][nextDirection][0]
                ans[0] = edges["to"]
                ans[1] = edges["via"]
                return ans
            else:
                if curLaneIndex is not None:
                    return ans

        for lane in tmp[curEdgeIndex]:
            if nextDirection in tmp[curEdgeIndex][lane]:
                edges = tmp[curEdgeIndex][lane][nextDirection][0]
                ans[0] = edges["to"]
                ans[1] = edges["via"]
                ans[3] = lane
        return ans

    def getNextInfoAll(
        self, curEdgeID, curLaneIndex=None, nextDirection=None, toEdgeID=None
    ):
        curEdgeIndex = self.getEdgeIndex(curEdgeID)
        # get process info in the road from curEdge to toEdge
        if toEdgeID is not None:
            tmp = self._getProcessInfo(curEdgeIndex, toEdgeID)
        else:
            tmp = self._getNextInfo(curEdgeIndex, nextDirection, curLaneIndex)
        return tmp[0], tmp[1], tmp[2], tmp[3]

    def getNextInfoFrom(self, curEdgeID, nextDirection=None, toEdgeID=None):
        ans = self.getNextInfoAll(curEdgeID, None, nextDirection, toEdgeID)
        return ans[3]

    def getNextInfoDirect(self, curEdgeID, toEdgeID, curLaneIndex=None):
        ans = self.getNextInfoAll(curEdgeID, curLaneIndex, None, toEdgeID)
        return ans[2]

    def getNextInfoVia(
        self, curEdgeID, curLaneIndex=None, nextDirection=None, toEdgeID=None
    ):
        ans = self.getNextInfoAll(curEdgeID, curLaneIndex, nextDirection, toEdgeID)
        return ans[1]

    def getNextInfoTo(self, curEdgeID, nextDirection, curLaneIndex=None):
        ans = self.getNextInfoAll(curEdgeID, curLaneIndex, nextDirection, None)
        return ans[0]

    def setNormalEdgeConnection(self, root=None):
        """
        this function generate connection list
        root : xmlparse root
        root is None:
            self.__normalEdgeConnection :
                list of edgesList which connects edgeindex
                (fromedgeindex : list of toedgeindex)
        root is not None :
            self.__normalEdgeConnection:
                dictionary list of edgeList which connects edgeindex
                    [fromEdgeIndex] = {'fromLane': {'direction':[
                        {'to':toEdgeIndex, 'via': viaLaneID}]}}
        """
        if root is not None:
            num_edges = self.getNum(NUM_KEY[EDGE_NORMAL_NUM])
            tmpConnectionList = [{} for i in range(num_edges)]
            attribList = self._getEdgeConnection(root)
            for attrib in attribList:
                fromEdgeIndex = self.getEdgeIndex(attrib["from"])
                fromLaneIndex = attrib["fromLane"]
                toEdgeID = attrib["to"]
                # tolane = attrib['toLane']
                direction = attrib["dir"]
                viaEdgeID = attrib["via"]
                tmp = {}
                tmpList = []
                if fromLaneIndex in tmpConnectionList[fromEdgeIndex]:
                    tmp = tmpConnectionList[fromEdgeIndex][fromLaneIndex]
                    if direction in tmp:
                        tmpList = tmp[direction]
                toEdge = {"to": toEdgeID, "via": viaEdgeID}
                tmpList.append(toEdge)
                tmp[direction] = tmpList
                tmpConnectionList[fromEdgeIndex][fromLaneIndex] = tmp
        else:
            self.__isDirection = False
            num_edges = self.__graph.num_edges
            tmpConnectionList = [[] * 1 for i in range(num_edges)]
            edgeList = self.__graph.edge_index.numpy()
            for i in range(num_edges):
                fromEdgeIndex = edgeList[0][i]
                toEdge = edgeList[1][i]
                if toEdge not in tmpConnectionList[fromEdgeIndex]:
                    tmpConnectionList[fromEdgeIndex].append(toEdge)
        self.__normalEdgeConnection = tmpConnectionList

    def updateNode(self, vehinfo):
        isExist = vehinfo["exist"]
        vehID = vehinfo["ID"]
        fromNodeIndex = self.getNodeIndex(vehID)
        if isExist:
            # add edge to graph
            curEdgeID = vehinfo["curEdgeID"]
            edgeIndex = self.getEdgeIndex(curEdgeID)
            nextEdgeID = vehinfo["nextEdgeID"]
            toNodeIndex = self.getNodeIndex(None, curEdgeID, True)
            if toNodeIndex == -1:
                curNodeID = self.getNodeID(-1, curEdgeID)
                nodeIndex = self.getNodeIndex(curNodeID)
                toNodeIndex = self.getNodeIndex(None, nextEdgeID, True)
                edgeIndex = self.getEdgeIndex(nextEdgeID)
                if nextEdgeID is None:
                    toNodeIndex = nodeIndex
        else:
            toNodeIndex = fromNodeIndex
            edgeIndex = -1
        self.__graph.edge_index[1][fromNodeIndex] = toNodeIndex
        # add position of vehicle as node position
        veh_pos = vehinfo["pos"] if isExist else [0.0, 0.0]
        self.__graph.pos[fromNodeIndex] = torch.tensor(veh_pos, dtype=torch.float)
        # add feature of vehicle as node
        if self.__isNode_attr:
            category_val = (
                VEHICLE_CATEGORY if isExist else NONE_VEHICLE_JUNCTION_CATEGORY
            )
            category = NODE_CATEGORY[category_val]
            speed = vehinfo["speed"] if isExist else 0.0
            tmp_feature = [speed, category]
            feature = flatten_list(tmp_feature)
            self.__graph.x[fromNodeIndex] = torch.tensor(feature, dtype=torch.float)
        # add feature of edge(veh-> tonode)
        if self.__isEdge_attr:
            speed = (
                0.0 if edgeIndex < 0 else self.__graph.edge_attr.numpy()[edgeIndex][0]
            )
            # calculation edge(veh->tonode) length
            toPos = self.__graph.pos.numpy()[toNodeIndex]
            length = self._calcDistance(vehinfo["pos"], toPos)
            category_val = NONE_ROAD_CATEGORY if edgeIndex < 0 else ROAD_CATEGORY
            category = EDGE_CATEGORY[category_val]
            tmp_attrib = [speed, length, category]
            attrib = flatten_list(tmp_attrib)
            self.__graph.edge_attr[fromNodeIndex] = torch.tensor(
                attrib, dtype=torch.float
            )

    # add vehicle as node
    def addNode(self, vehinfo):
        isExist = vehinfo["exist"]
        vehID = vehinfo["ID"]
        # add vehicleID as nodeID
        self._addNodeID(vehID)
        fromNodeIndex = self.getNodeIndex(vehID)
        if isExist:
            # add edge to graph
            curEdgeID = vehinfo["curEdgeID"]
            edgeIndex = self.getEdgeIndex(curEdgeID)
            nextEdgeID = vehinfo["nextEdgeID"]
            toNodeIndex = self.getNodeIndex(None, curEdgeID, True)
            if toNodeIndex == -1:
                curNodeID = self.getNodeID(-1, curEdgeID)
                nodeIndex = self.getNodeIndex(curNodeID)
                toNodeIndex = self.getNodeIndex(None, nextEdgeID, True)
                edgeIndex = self.getEdgeIndex(nextEdgeID)
                if nextEdgeID is None:
                    toNodeIndex = nodeIndex
        else:
            toNodeIndex = fromNodeIndex
            edgeIndex = -1
        tmp = self.__graph.edge_index.numpy()
        src = np.append(tmp[0], fromNodeIndex)
        dst = np.append(tmp[1], toNodeIndex)
        tmp = np.array([src, dst])
        edge_index = torch.tensor(tmp, dtype=torch.long)
        self.__graph.edge_index = edge_index
        # add position of vehicle as node position
        veh_pos = vehinfo["pos"] if isExist else [0.0, 0.0]
        tmp = np.append(self.__graph.pos.numpy(), [veh_pos], axis=0)
        pos = torch.tensor(tmp, dtype=torch.float)
        self.__graph.pos = pos
        # add feature of vehicle as node
        if self.__isNode_attr:
            category_val = (
                VEHICLE_CATEGORY if isExist else NONE_VEHICLE_JUNCTION_CATEGORY
            )
            category = NODE_CATEGORY[category_val]
            speed = vehinfo["speed"] if isExist else 0.0
            tmp_feature = [speed, category]
            feature = flatten_list(tmp_feature)
            tmp = np.append(self.__graph.x.numpy(), [feature], axis=0)
            x = torch.tensor(tmp, dtype=torch.float)
            self.__graph.x = x
        # add feature of edge(veh-> tonode)
        if self.__isEdge_attr:
            speed = (
                0.0 if edgeIndex < 0 else self.__graph.edge_attr.numpy()[edgeIndex][0]
            )
            # calculation edge(veh->tonode) length
            toPos = self.__graph.pos.numpy()[toNodeIndex]
            length = self._calcDistance(vehinfo["pos"], toPos)
            category_val = NONE_ROAD_CATEGORY if edgeIndex < 0 else ROAD_CATEGORY
            category = EDGE_CATEGORY[category_val]
            tmp_attrib = [speed, length, category]
            attrib = flatten_list(tmp_attrib)
            tmp = np.append(self.__graph.edge_attr.numpy(), [attrib], axis=0)
            edge_attr = torch.tensor(tmp, dtype=torch.float)
            self.__graph.edge_attr = edge_attr

    def _calcDistance(self, fromPos, toPos):
        return float(
            np.sqrt((toPos[0] - fromPos[0]) ** 2 + (toPos[1] - fromPos[1]) ** 2)
        )

    def _addNodeID(self, nodeID):
        if nodeID not in self.__nodeIDList:
            self.__nodeIDList.append(nodeID)
            self.__num[NUM_KEY[NODE_NUM]] = len(self.__nodeIDList)

    def _getFeatureNode(self, attrib):
        speed = 0.0
        category = NODE_CATEGORY[JUNCTION_CATEGORY]
        tmp = [speed, category]
        feature = flatten_list(tmp)
        return feature

    def _getPosNode(self, attrib):
        self._addNodeID(attrib["id"])
        pos = [float(attrib["x"]), float(attrib["y"])]
        return pos

    def _generateNode(self, root):
        posList = []
        xList = []
        for child in root:
            if child.tag == "junction":
                self.__num[NUM_KEY[JUNCTION_NUM]] += 1
                if child.attrib["type"] is None or child.attrib["type"] != "internal":
                    posList.append(self._getPosNode(child.attrib))
                    xList.append(self._getFeatureNode(child.attrib))
        x = torch.tensor(xList, dtype=torch.float)
        pos = torch.tensor(posList, dtype=torch.float)
        return pos, x

    def _getFeatureEdge(self, attrib):
        for child in attrib:
            if child.tag == "lane":
                self.__num[NUM_KEY[LANE_NUM]] += 1
                speed = float(child.attrib["speed"])
                length = float(child.attrib["length"])
                category = float(EDGE_CATEGORY[ROAD_CATEGORY])
                tmp = [speed, length, category]
                feature = flatten_list(tmp)
                return feature
        return None

    def _convNodeLong(self, attrib):
        """
        purpose : conversion nodeID's type from string to torch.long

        attrib : read the xml and get the attribution from a tag
                 (assume tag is "edge")
        return : long, long (nodeID hash code)
        """
        # get nodeID(string)
        fromNodeID = attrib["from"]
        toNodeID = attrib["to"]

        # nodeID(string) -> torch.long
        self._addNodeID(fromNodeID)
        self._addNodeID(toNodeID)

        # add edge, node to graph
        fromNodeIndex = self.getNodeIndex(fromNodeID)
        toNodeIndex = self.getNodeIndex(toNodeID)
        return fromNodeIndex, toNodeIndex

    def _generateEdge(self, root):
        fromList = []
        toList = []
        xList = []
        edgeIDList = []
        normalEdgeIDList = []
        for child in root:
            if child.tag == "edge":
                self.__num[NUM_KEY[EDGE_NUM]] += 1
                edgeID = child.attrib["id"]
                # avoid internel edges
                if "function" in child.attrib:
                    if child.attrib["function"] == "internal":
                        if edgeID not in edgeIDList:
                            edgeIDList.append(edgeID)
                        continue
                if "from" in child.attrib:
                    if edgeID in normalEdgeIDList:
                        continue
                    normalEdgeIDList.append(edgeID)
                    self.__num[NUM_KEY[EDGE_NORMAL_NUM]] += 1
                    # generate edge_index
                    fromNodeIndex, toNodeIndex = self._convNodeLong(child.attrib)
                    fromList.append(fromNodeIndex)
                    toList.append(toNodeIndex)
                    # get feature of edge(length)
                    xList.append(self._getFeatureEdge(child))
        normalEdgeIDList[len(normalEdgeIDList) : len(edgeIDList)] = edgeIDList
        self.__edgeIDList = normalEdgeIDList
        edge_attr = torch.tensor(xList, dtype=torch.float)
        edge_index = torch.tensor([fromList, toList], dtype=torch.long)
        return edge_index, edge_attr

    def _xmlParse(self):
        tree = ET.parse(self.__filepath)
        root = tree.getroot()
        # print(root)
        return root

    def _defineGraph(self):
        root = self._xmlParse()
        # create graph
        edge_index, edge_attr = self._generateEdge(root)
        pos, x = self._generateNode(root)
        tmp = None
        if self.__isEdge_attr:
            if self.__isNode_attr:
                tmp = Data(x=x, pos=pos, edge_index=edge_index, edge_attr=edge_attr)
            else:
                tmp = Data(pos=pos, edge_index=edge_index, edge_attr=edge_attr)
        else:
            if self.__isNode_attr:
                tmp = Data(x=x, pos=pos, edge_index=edge_index)
            else:
                tmp = Data(pos=pos, edge_index=edge_index)
        self.__graph = tmp
        # generate connection list
        self.setNormalEdgeConnection(root)

    def _getEdgeConnection(self, root, isnormal=True):
        """
        get list of connection attrib dictionary
        (element is {'from': fromedgeID, 'to': toedgeID,
                     'fromLane': fromLaneNumber on fromedge,
                     'toLane': toLaneNumber on toedge,
                     'via': via leanIndex,
                     'dir': driection from fromedge to toedge,
                     'state': state of connection, "tl": traffic light id,
                     'linkIndex': index of signal responsible for
                                 the connection with traffic light})

        *dir dictionary:
        'l': left
        'L': partially Left
        'r': right
        'R': partially Right
        'T' or 't': turn(back)
        's': straight
        'invalid': no direction
        """
        edgeConnectionList = []
        for child in root:
            if child.tag == "connection":
                if "from" in child.attrib:
                    if isnormal:
                        if child.attrib["from"][0] != ":":
                            edgeConnectionList.append(child.attrib)
                    else:
                        edgeConnectionList.append(child.attrib)
        return edgeConnectionList

    def getPositionTuple(self, data):
        if data.pos is not None:
            nodePositionTuple = {}
            for i, element in enumerate(data.pos):
                nodePositionTuple[i] = tuple(element.numpy())
        return nodePositionTuple

    def showgraph(self, data, ispos=True, title="test", isLabel=False):
        # networkxのグラフに変換
        nxg = to_networkx(data)
        # print(list(nxg.edges))
        # 可視化のためのページランク計算
        pr = nx.pagerank(nxg)
        pr_max = np.array(list(pr.values())).max()

        # 可視化する際のノード位置
        pos = self.getPositionTuple(data)
        draw_pos = nx.spring_layout(nxg, pos=pos, seed=0)

        cmap = plt.get_cmap("tab10")
        labels = np.zeros(data.num_nodes)
        if isLabel:
            # ノードの色設定
            labels = data.y.numpy()
            print(labels)
        colors = [cmap(label) for label in labels]

        # 図のサイズ
        plt.figure(figsize=(100, 100))

        # 描画
        if ispos:
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


# sumoBinary = os.environ['SUMO_HOME']
# sumoBinary += "/mytest/shinjuku/testmap/osm.net.xml"
# g = Graph(sumoBinary, edge_attr=True)
# print(g.edge_attr.numpy())
# print(g.graph.pos.numpy())
# print(g.getEdges())

# # g.check_graph(g.graph)
# # print(g.num)
# # print(g.nodedict)
# g.showgraph(g.graph, title='sumo', ispos=False, isLabel=False)
