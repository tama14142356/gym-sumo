import xml.etree.ElementTree as ET
import torch
from torch_geometric.data import Data
from collections import defaultdict
import numpy as np

import networkx as nx
from torch_geometric.utils import to_networkx
from matplotlib import pyplot as plt


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
        self.__defineGraph()

    def getGraph(self):
        graph = self.__graph
        if self.__graph.edge_index is None:
            graph = None
        return graph

    def setGraph(self, graph):
        # if type(graph) is Data:
        self.__graph = graph

    def getNum(self, key):
        value = -1
        tmp = self.__num
        if key in tmp:
            value = tmp[key]
        return value

    def getEdgeIndex(self, edgeID, isNormal=True):
        index = -1
        tmpList = self.__edgeIDList
        edge_num = self.getNum('edge_normal')
        if edgeID in tmpList:
            index = tmpList.index(edgeID)
        if isNormal:
            if index >= edge_num:
                index = -1
        return index

    def getEdgeID(self, edgeIndex, isNormal=True):
        tmpList = self.__edgeIDList
        ID = None
        edge_num = self.getNum('edge_normal')
        if edgeIndex < len(tmpList) and edgeIndex >= 0:
            ID = tmpList[edgeIndex]
        if isNormal:
            if edgeIndex >= edge_num:
                ID = None
        return ID

    def getNodeID(self, nodeIndex=-1, internalEdgeID=None):
        nodeID = None
        tmpList = self.__nodeIDList
        edgeID = internalEdgeID
        if nodeIndex < len(tmpList) and nodeIndex >= 0:
            nodeID = tmpList[nodeIndex]
        if edgeID is not None:
            index = edgeID.rfind('_')
            nodeID = edgeID[1:index]
        return nodeID

    def getNodeIndex(self, nodeID=None, normalEdgeID=None, isTo=True):
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
            tmp = self.__graph.edge_index
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
    
    def __getProcessInfo(self, curEdgeIndex, toEdgeID):
        tmp = self.__normalEdgeConnection
        ans = [toEdgeID, None, None, None]
        isFinished = False
        
        # index error or not supported
        if (curEdgeIndex >= len(tmp)
                or curEdgeIndex < 0
                or not self.__isDirection):
            isFinished = True
        
        for lane in tmp[curEdgeIndex]:
            if isFinished:
                break
            for direct in tmp[curEdgeIndex][lane]:
                if isFinished:
                    break
                for edges in tmp[curEdgeIndex][lane][direct]:
                    if edges['to'] == toEdgeID:
                        ans[1] = edges['via']
                        ans[2] = direct
                        ans[3] = lane
                        isFinished = True
        return ans
    
    # 基本としてはcurEdgeIndexから接続されているEdgeの情報を返す
    # ただし、現在の車線(curLaneIndex)、行きたい方向(nextDirection)、接続先(toEdgeID)
    # のいずれかと一致するもののうち一つを返す。
    # 基本優先度は以下：
    # 1. toEdgeID
    # 2. nextDirection
    # 3. curLaneIndex
    # 優先順位の意味はNoneでない限り、優先順位の一番高いもの以外は基本的に無視するということ。
    # toEdgeIDがNoneでないなら、とにかくnextDirectionと一致していようがいまいが、
    # 関係なくtoEdgeIDと一致するedgeの中の一つを返す。
    # (ただし、curEdgeに対するtoEdgeは基本的に1対１であるとの想定)
    # nextDirectionを優先したい場合（接続先がわかっていない場合）toEdgeをNoneにすればその方向に道があるか確認できる
    # ただし、それが指定した車線から行けるかどうかはわからない。行けるのであれば、優先的にそれを返すが、
    # ない場合は、違う車線からその方向に道があるのであれば、それを返す。
    # よって、どうしても車線を一致させたい場合は確認する必要がある。
    def __getNextInfo(self, curEdgeIndex, curLaneIndex=None,
                      nextDirection=None, toEdgeID=None):
        # get process info in the road from curEdge to toEdge
        if toEdgeID is not None:
            return self.__getProcessInfo(curEdgeIndex, toEdgeID)
        
        ans = [None, None, nextDirection, curLaneIndex]
        tmp = self.__normalEdgeConnection
        # index error or not supported
        if (curEdgeIndex >= len(tmp)
                or curEdgeIndex < 0
                or not self.__isDirection):
            return ans
        
        for lane in tmp[curEdgeIndex]:
            if nextDirection in tmp[curEdgeIndex][lane]:
                edges = tmp[curEdgeIndex][lane][nextDirection][0]
                ans[0] = edges['to']
                ans[1] = edges['via']
                ans[3] = lane
                if lane == curLaneIndex:
                    break
        return ans

    def getNextInfoAll(self, curEdgeID, curLaneIndex=None,
                       nextDirection='no', toEdgeID=None):
        curEdgeIndex = self.getEdgeIndex(curEdgeID)
        tmp = self.__getNextInfo(curEdgeIndex, curLaneIndex,
                                 nextDirection, toEdgeID)
        return tmp[0], tmp[1], tmp[2], tmp[3]

    def getNextInfoFrom(self, curEdgeID, curLaneIndex=None,
                        nextDirection='no', toEdgeID=None):
        _, _, _, ans = self.getNextInfoAll(curEdgeID, curLaneIndex,
                                           nextDirection, toEdgeID)
        return ans

    def getNextInfoDirect(self, curEdgeID, curLaneIndex=None,
                          nextDirection='no', toEdgeID=None):
        _, _, ans, _ = self.getNextInfoAll(curEdgeID, curLaneIndex,
                                           nextDirection, toEdgeID)
        return ans

    def getNextInfoVia(self, curEdgeID, curLaneIndex=None,
                       nextDirection='no', toEdgeID=None):
        _, ans, _, _ = self.getNextInfoAll(curEdgeID, curLaneIndex,
                                           nextDirection, toEdgeID)
        return ans

    def getNextInfoTo(self, curEdgeID, curLaneIndex=None,
                      nextDirection='no', toEdgeID=None):
        ans, _, _, _ = self.getNextInfoAll(curEdgeID, curLaneIndex,
                                           nextDirection, toEdgeID)
        return ans

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
            num_edges = self.__num['edge_normal']
            tmpConnectionList = [{} for i in range(num_edges)]
            attribList = self.__getEdgeConnection(root)
            for attrib in attribList:
                fromEdgeIndex = self.getEdgeIndex(attrib['from'])
                fromLaneIndex = attrib['fromLane']
                toEdgeID = attrib['to']
                # tolane = attrib['toLane']
                direction = attrib['dir']
                viaEdgeID = attrib['via']
                tmp = {}
                tmpList = []
                if fromLaneIndex in tmpConnectionList[fromEdgeIndex]:
                    tmp = tmpConnectionList[fromEdgeIndex][fromLaneIndex]
                    if direction in tmp:
                        tmpList = tmp[direction]
                toEdge = {'to': toEdgeID, 'via': viaEdgeID}
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

    # add vehicle as node
    def addNode(self, vehID, curedgeID, vehinfo):
        isNode = False
        # add edge to graph
        tmp = self.__graph.edge_index.numpy()
        toNodeIndex = self.getNodeIndex(None, curedgeID, True)
        if toNodeIndex == -1:
            isNode = True
            curNodeID = self.getNodeID(-1, curedgeID)
            nodeIndex = self.getNodeIndex(curNodeID)
        else:
            # add vehicleID as nodeID
            self.__addNodeID(vehID)
            src = np.append(tmp[0], self.getNodeIndex(vehID))
            dst = np.append(tmp[1], toNodeIndex)
            tmp = np.array([src, dst])
        edge_index = torch.tensor(tmp, dtype=torch.long)
        # add position of vehicle as node position
        tmp = self.__graph.pos.numpy()
        if not isNode:
            # calculation edge(veh->tonode) length
            distance = self.__calcDistance(vehinfo['pos'], tmp[toNodeIndex])
            tmp = np.append(tmp, [vehinfo['pos']], axis=0)
        pos = torch.tensor(tmp, dtype=torch.float)
        # add feature of vehicle as node
        if self.__isNode_attr:
            tmp = self.__graph.x.numpy()
        else:
            tmp = np.zeros(self.__graph.num_nodes, dtype=float)
        if isNode:
            tmp[nodeIndex] = vehinfo['speed']
        else:
            tmp = np.append(tmp, [vehinfo['speed']], axis=0)
        x = torch.tensor(tmp, dtype=torch.float)
        # add feature of edge(veh-> tonode)
        if self.__isEdge_attr:
            tmp = self.__graph.edge_attr.numpy()
            if not isNode:
                edgeIndex = self.getEdgeIndex(curedgeID)
                attrib = [tmp[edgeIndex][0], distance]
                tmp = np.append(tmp, [attrib], axis=0)
            edge_attr = torch.tensor(tmp, dtype=torch.float)
            self.__graph = Data(x=x, edge_index=edge_index,
                                edge_attr=edge_attr, pos=pos)
        else:
            self.__graph = Data(x=x, edge_index=edge_index, pos=pos)

    def __calcDistance(self, fromPos, toPos):
        return float(
            np.sqrt((toPos[0] - fromPos[0])**2 + (toPos[1] - fromPos[1])**2))

    def __addNodeID(self, nodeID):
        if nodeID not in self.__nodeIDList:
            # value = len(self.__nodeIDList)
            # self.nodeIDdict[nodeID] = value
            self.__nodeIDList.append(nodeID)

    def __getFeatureNode(self, attrib):
        speed = 0.0
        return [speed]

    def __getPosNode(self, attrib):
        self.__addNodeID(attrib['id'])
        pos = [float(attrib['x']), float(attrib['y'])]
        return pos

    def __generateNode(self, root):
        posList = []
        xList = []
        for child in root:
            if child.tag == 'junction':
                self.__num['junction'] += 1
                if (child.attrib['type'] is None
                        or child.attrib['type'] != 'internal'):
                    posList.append(self.__getPosNode(child.attrib))
                    xList.append(self.__getFeatureNode(child.attrib))
        x = torch.tensor(xList, dtype=torch.float)
        pos = torch.tensor(posList, dtype=torch.float)
        return pos, x

    def __getFeatureEdge(self, attrib):
        for child in attrib:
            if child.tag == 'lane':
                self.__num['lane'] += 1
                speed = child.attrib['speed']
                length = child.attrib['length']
                return float(speed), float(length)
        return None

    def __convNodeLong(self, attrib):
        """
        purpose : conversion nodeID's type from string to torch.long

        attrib : read the xml and get the attribution from a tag
                 (assume tag is "edge")
        return : long, long (nodeID hash code)
        """
        # get nodeID(string)
        fromNodeID = attrib['from']
        toNodeID = attrib['to']

        # nodeID(string) -> torch.long
        self.__addNodeID(fromNodeID)
        self.__addNodeID(toNodeID)

        # add edge, node to graph
        fromNodeIndex = self.getNodeIndex(fromNodeID)
        toNodeIndex = self.getNodeIndex(toNodeID)
        return fromNodeIndex, toNodeIndex

    def __generateEdge(self, root):
        fromList = []
        toList = []
        xList = []
        edgeIDList = []
        normalEdgeIDList = []
        for child in root:
            if child.tag == 'edge':
                self.__num['edge'] += 1
                edgeID = child.attrib['id']
                # avoid internel edges
                if 'function' in child.attrib:
                    if child.attrib['function'] == 'internal':
                        if edgeID not in edgeIDList:
                            edgeIDList.append(edgeID)
                        continue
                if 'from' in child.attrib:
                    if edgeID in normalEdgeIDList:
                        continue
                    normalEdgeIDList.append(edgeID)
                    # self.normalEdgeIDdict[edgeID] \
                    #     = self.__num['edge_normal']
                    self.__num['edge_normal'] += 1
                    # generate edge_index
                    fromNodeIndex, toNodeIndex \
                        = self.__convNodeLong(child.attrib)
                    fromList.append(fromNodeIndex)
                    toList.append(toNodeIndex)
                    # get feature of edge(length)
                    edgespeed, edgelength = self.__getFeatureEdge(child)
                    x = [edgespeed, edgelength]
                    xList.append(x)
        normalEdgeIDList[len(normalEdgeIDList):len(edgeIDList)] = edgeIDList
        self.__edgeIDList = normalEdgeIDList
        edge_attr = torch.tensor(xList, dtype=torch.float)
        edge_index = torch.tensor([fromList, toList], dtype=torch.long)
        return edge_index, edge_attr

    def __xmlParse(self):
        tree = ET.parse(self.__filepath)
        root = tree.getroot()
        # print(root)
        return root

    def __defineGraph(self):
        root = self.__xmlParse()
        # create graph
        edge_index, edge_attr = self.__generateEdge(root)
        pos, x = self.__generateNode(root)
        tmp = None
        if self.__isEdge_attr:
            if self.__isNode_attr:
                tmp = Data(x=x, pos=pos, edge_index=edge_index,
                           edge_attr=edge_attr)
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

    def __getEdgeConnection(self, root, isnormal=True):
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
            if child.tag == 'connection':
                if 'from' in child.attrib:
                    if isnormal:
                        if child.attrib['from'][0] != ':':
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

    def showgraph(self, data, ispos=True, title='test', isLabel=False):
        # networkxのグラフに変換
        nxg = to_networkx(data)
        # print(list(nxg.edges))
        # 可視化のためのページランク計算
        pr = nx.pagerank(nxg)
        pr_max = np.array(list(pr.values())).max()

        # 可視化する際のノード位置
        pos = self.getPositionTuple(data)
        draw_pos = nx.spring_layout(nxg, pos=pos, seed=0)

        cmap = plt.get_cmap('tab10')
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
                nxg, draw_pos,
                node_size=[v / pr_max * 1000 for v in pr.values()],
                node_color=colors, alpha=0.5)
            nx.draw_networkx_edges(nxg, draw_pos, arrowstyle='-', alpha=0.2)
            nx.draw_networkx_labels(nxg, draw_pos, font_size=10)

        plt.title(title)
        plt.show()

    def check_graph(self, data):
        '''グラフ情報を表示'''
        print("グラフ構造:", data)
        print("グラフのキー: ", data.keys)
        print("ノード数:", data.num_nodes)
        print("エッジ数:", data.num_edges)
        print("ノードの特徴量数:", data.num_node_features)
        print("孤立したノードの有無:", data.contains_isolated_nodes())
        print("自己ループの有無:", data.contains_self_loops())
        print("====== ノードの特徴量:x ======")
        print(data['x'])
        print("====== ノードのクラス:y ======")
        print(data['y'])
        print("========= エッジ形状 =========")
        print(data['edge_index'])


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
