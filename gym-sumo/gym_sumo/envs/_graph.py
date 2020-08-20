import xml.etree.ElementTree as ET
import torch
from torch_geometric.data import Data
from collections import defaultdict
import numpy as np
import random
import os

import networkx as nx
from torch_geometric.utils import to_networkx
from matplotlib import pyplot as plt

class Graph:
    def __init__(self, filepath, isNode_attr=True, isEdge_attr=True):
        self.filepath = filepath
        self.nodeIDList = [] # list of node id
        self.EdgeNodedict = {} # dictionary: {'edgeID':(fromnodeindex, tonodeindex)}
        self.edgeIDList = [] # list of edgeID including normal, internal
        # self.nodeIDdict = {} # dictionary: {'nodeID': nodeindex}
        # self.normalEdgeIDdict = {} # dictionary: {'edgeID': edgeindex}
        self.edges = None
        self.num = defaultdict(int)
        self.isEdge_attr = isEdge_attr
        self.isNode_attr = isNode_attr
        self.isDirection = True
        # create graph
        self.defineGraph()
        self.gettuplePos(self.graph)
    
    def getEdgeIndex(self, edgeID, isNormal=True):
        index = -1
        if edgeID in self.edgeIDList:
            index = self.edgeIDList.index(edgeID)
        if isNormal:
            if index >= self.num['edge_normal']:
                index = -1
        return index
    
    def getEdgeID(self, edgeIndex, isNormal=True):
        if edgeIndex >= len(self.edgeIDList):
            return None
        if isNormal:
            if edgeIndex >= self.num['edge_normal']:
                return None
        return self.edgeIDList[edgeIndex]    

    def getNodeTuple(self, normalEdgeID):
        if normalEdgeID in self.EdgeNodedict:
            return self.EdgeNodedict[normalEdgeID]
        return (-1, -1)

    def getFromNodeIndex(self, normalEdgeID):
        tempNodeTuple = self.getNodeTuple(normalEdgeID)
        return tempNodeTuple[0]

    def getToNodeIndex(self, normalEdgeID):
        tempNodeTuple = self.getNodeTuple(normalEdgeID)
        return tempNodeTuple[1]

    def getFromNodeID(self, normalEdgeID):
        

    def getToNodeID(self, normalEdgeID):
        nodeIndex = self.getToNodeIndex(normalEdgeID)
        return self.getNodeID(nodeIndex)

    def getNodeID(self, nodeIndex=None, normalEdgeID=None, isFrom=True):
        nodeID = None
        if nodeIndex is None:
            if isFrom:
                nodeIndex = self.getFromNodeIndex(normalEdgeID)
                nodeID = self.getNodeID(nodeIndex)
        else:
            if nodeIndex < len(self.nodeIDList):
                nodeID = self.nodeIDList[nodeNum]
        
        return nodeID

    def getNodeIndex(self, nodeID):
        if nodeID in self.nodeIDList:
            # return self.nodeIDdict[nodeID]
            return self.nodeIDList.index(nodeID)
        return -1

    def calcDistance(self, fromPos, toPos):
        return float(np.sqrt((toPos[0] - fromPos[0])**2 + (toPos[1] - fromPos[1])**2))
    
    def getEdges(self):
        if self.edges is not None:
            return self.edges
        self.nxg = to_networkx(self.graph)
        tmpList = list(self.nxg.edges)
        self.edges = []
        for i, tmp in enumerate(tmpList):
            edge = [list(self.graph.edge_attr[i].numpy()), list(tmp)]
            self.edges.append(edge)
        return self.edges
    
    def getNodeIDfromEdge(self, edgeID):
        edge = edgeID[1:]
        length = len(edge)
        index = length - 1
        for i in range(length - 1, 0, -1):
            index = i
            if edge[i] == '_':
                break
        nodeID = edge[:index]
        return nodeID
    
    def getEdgeIDfromNode(self, nodeID, fromLane):
        edgeID = ':' + nodeID + '_{}'.format(fromLane)
        return edgeID

    def getNextEdges(self, curedgeID, curLaneIndex, direction):
        tmp = self.normalEdgeConnection
        nextEdges = []
        if self.isDirection:
            fromEdgeIndex = self.getEdgeIndex(curedgeID)
            if fromEdgeIndex < len(tmp):
                if curLaneIndex in tmp[fromEdgeIndex]:
                    if direction in tmp[fromEdgeIndex][curLaneIndex]:
                        nextEdges = tmp[fromEdgeIndex][curLaneIndex][direction][0]
        return nextEdges

    def getNextEdgeID(self, curedgeID, curLaneIndex, direction, isNormal=True):
        nextEdges = self.getNextEdges(curedgeID, curLaneIndex, direction)
        edgeID = None
        if len(nextEdges) > 0:
            if isNormal:
                edgeID = nextEdges['to']
            else:
                edgeID = nextEdges['via']
        return edgeID    

    def setNormalEdgeConnection(self, root=None):
        """
        this function generate connection list
        root : xmlparse root
        root is None: 
            self.normalEdgeConnection : 
                list of edgesList which connects edgeindex (fromedgeindex : list of toedgeindex)
        root is not None :
            self.normalEdgeConnection:
                dictionary list of edgeList which connects edgeindex
                    [fromEdgeIndex] = {'fromLane': {'direction':[{'to':toEdgeIndex, 'via': viaEdgeID}]}}
        """
        if root is not None:
            num_edges = self.num['edge_normal']
            self.normalEdgeConnection = [{} for i in range(num_edges)]
            connectionList = self.getEdgeConnection(root)
            for connect in connectionList:
                fromEdgeIndex = self.getEdgeIndex(connect['from'])
                fromLaneIndex = connect['fromLane']
                toEdgeID = connect['to']
                # tolane = connect['toLane']
                direction = connect['dir']
                viaEdgeID = connect['via']
                tmp = {}
                tmpList = []
                if fromLaneIndex in self.normalEdgeConnection[fromEdgeIndex]:
                    tmp = self.normalEdgeConnection[fromEdgeIndex][fromLaneIndex]
                    if direction in tmp:
                        tmpList = tmp[direction]
                toEdge = {'to': toEdgeID, 'via': viaEdgeID}
                tmpList.append(toEdge)
                tmp[direction] = tmpList
                self.normalEdgeConnection[fromEdgeIndex][fromLaneIndex] = tmp
        else:
            self.isDirection = False
            num_edges = self.graph.num_edges
            self.normalEdgeConnection = [[] * 1 for i in range(num_edges)]
            edgeList = self.graph.edge_index.numpy()
            for i in range(num_edges):
                fromEdgeIndex = edgeList[0][i]
                toEdge = edgeList[1][i]
                if toEdge not in self.normalEdgeConnection[fromEdgeIndex]:
                    self.normalEdgeConnection[fromEdgeIndex].append(toEdge)

    # add vehicle as node
    def addNode(self, vehID, curedgeID, vehinfo):
        isNode = False
        # add edge to graph
        tmp = self.graph.edge_index.numpy()
        if curedgeID not in self.EdgeNodedict:
            isNode = True
            curNodeID = self.getNodeIDfromEdge(curedgeID)
            nodeIndex = self.getNodeIndex(curNodeID)
        else:
            edge = self.EdgeNodedict[curedgeID]
            # add vehicleID as nodeID
            self.addNodeID(vehID)
            src = np.append(tmp[0], self.getNodeIndex(vehID))
            dst = np.append(tmp[1], edge[1])
            tmp = np.array([src, dst])
        edge_index = torch.tensor(tmp,dtype=torch.long)
        # add position of vehicle as node position
        tmp = self.graph.pos.numpy()
        if not isNode:
            # calculation edge(veh->tonode) length 
            distance = self.calcDistance(vehinfo['pos'], tmp[edge[1]])
            tmp = np.append(tmp, [vehinfo['pos']], axis=0)
        pos = torch.tensor(tmp, dtype=torch.float)
        # add feature of vehicle as node
        if self.isNode_attr:
            tmp = self.graph.x.numpy()
        else:
            tmp = np.zeros(self.graph.num_nodes, dtype=float)
        if isNode:
            tmp[nodeIndex] = vehinfo['speed']
        else:
            tmp = np.append(tmp, [vehinfo['speed']], axis=0)
        x = torch.tensor(tmp, dtype=torch.float)
        # add feature of edge(veh-> tonode)
        if self.isEdge_attr:
            tmp = self.graph.edge_attr.numpy()
            if not isNode:
                edgenum = self.getEdgeIndex(curedgeID)
                attrib = [tmp[edgenum][0], distance]
                tmp = np.append(tmp, [attrib], axis=0)
            edge_attr = torch.tensor(tmp, dtype=torch.float)
            self.graph = Data(x = x, edge_index=edge_index, edge_attr=edge_attr, pos=pos)
        else:
            self.graph = Data(x = x, edge_index=edge_index, pos=pos)

    def addNodeID(self, nodeID):
        if nodeID not in self.nodeIDList:
            # value = len(self.nodeIDList)
            # self.nodeIDdict[nodeID] = value
            self.nodeIDList.append(nodeID)
    
    def getFeatureNode(self, attrib):
        speed = 0.0
        return [speed]
    
    def getPosNode(self, attrib):
        self.addNodeID(attrib['id'])
        pos = [float(attrib['x']), float(attrib['y'])]
        return pos

    def generateNode(self, root):
        posList = []
        xList = []
        for child in root:
            if child.tag == 'junction':
                self.num['junction'] += 1
                if child.attrib['type'] is None or child.attrib['type'] != 'internal':
                    posList.append(self.getPosNode(child.attrib))
                    xList.append(self.getFeatureNode(child.attrib))
        x = torch.tensor(xList, dtype=torch.float)
        pos = torch.tensor(posList, dtype=torch.float)
        return pos, x

    def getFeatureEdge(self, attrib):
        for child in attrib:
            if child.tag == 'lane':
                self.num['lane'] += 1
                speed = child.attrib['speed']
                length = child.attrib['length']
                return float(speed), float(length)
        return None
    
    def convNodeLong(self, attrib):
        """ 
        purpose : conversion nodeID's type from string to torch.long 

        attrib : read the xml and get the attribution from a tag (assume tag is "edge")
        return : long, long (nodeID hash code)
        """
        # get nodeID(string)
        fromNodeID = attrib['from']
        toNodeID = attrib['to']

        # nodeID(string) -> torch.long
        self.addNodeID(fromNodeID)
        self.addNodeID(toNodeID)

        # add edge, node to graph
        fromNodeIndex = self.getNodeIndex(fromNodeID)
        toNodeIndex = self.getNodeIndex(toNodeID)
        self.EdgeNodedict[attrib['id']] = (fromNodeIndex, toNodeIndex)
        return fromNodeIndex, toNodeIndex
    
    def generateEdge(self, root):
        fromList = []
        toList = []
        xList = []
        edgeIDList = []
        normalEdgeIDList = []
        for child in root:
            if child.tag == 'edge':
                self.num['edge'] += 1
                # avoid internel edges
                if 'function' in child.attrib:
                    if child.attrib['function'] == 'internal':
                        if child.attrib['id'] not in edgeIDList:
                            edgeIDList.append(child.attrib['id'])
                        continue
                if 'from' in child.attrib:
                    if child.attrib['id'] not in normalEdgeIDList:
                        normalEdgeIDList.append(child.attrib['id'])
                        # self.normalEdgeIDdict[child.attrib['id']] = self.num['edge_normal']
                        self.num['edge_normal'] += 1
                    # generate edge_index
                    fromNodeIndex, toNodeIndex = self.convNodeLong(child.attrib)
                    fromList.append(fromNodeIndex)
                    toList.append(toNodeIndex)
                    # get feature of edge(length)
                    edgespeed, edgelength = self.getFeatureEdge(child)
                    x = [edgespeed, edgelength]
                    xList.append(x)
        normalEdgeIDList[len(normalEdgeIDList):len(edgeIDList)] = edgeIDList
        self.edgeIDList = normalEdgeIDList
        edge_attr = torch.tensor(xList, dtype=torch.float)
        edge_index = torch.tensor([fromList, toList], dtype=torch.long)
        return edge_index, edge_attr

    def xmlParse(self):
        tree = ET.parse(self.filepath)
        root = tree.getroot()
        # print(root)
        return root

    def defineGraph(self):
        root = self.xmlParse()
        # create graph
        edge_index, edge_attr = self.generateEdge(root)
        pos, x = self.generateNode(root)
        # generate connection list
        self.setNormalEdgeConnection(root)
        if self.isEdge_attr:
            if self.isNode_attr:
                self.graph = Data(x = x, pos=pos, edge_index=edge_index, edge_attr=edge_attr)
            else:
                self.graph = Data(pos=pos, edge_index=edge_index, edge_attr=edge_attr)
        else :
            if self.isNode_attr:
                self.graph = Data(x = x, pos=pos, edge_index=edge_index)
            else:
                self.graph = Data(pos=pos, edge_index=edge_index)
    
    def getEdgeConnection(self, root, isnormal=True):
        """  
        get list of connection attrib dictionary 
        (element is {'from': fromedgeID, 'to':toedgeID, 'fromLane':fromLaneNumber on fromedge, 
        'toLane':toLaneNumber on toedge, 'via': via edge(internal edge=junction), 
        'dir': driection from fromedge to toedge, 'state': state of connection, "tl":traffic light id, 
        'linkIndex':index of signal responsible for the connection with traffic light})
        
        *dir dictionary:
        'l':left
        'L':partially Left
        'r':right
        'R':partially Right
        'T' or 't' : turn(back)
        's' : straight
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
    
    def gettuplePos(self, data):
        pos = None
        self.nodepos = []
        if data.pos is not None:
            tmp = {}
            for i, element in enumerate(data.pos):
                tmp[i] = tuple(element.numpy())
                self.nodepos.append(tmp[i])
            pos = tmp   
        return pos

    def showgraph(self, data, ispos=True, title='test', label=False):
        # networkxのグラフに変換
        nxg = to_networkx(data)
        # print(list(nxg.edges))
        # 可視化のためのページランク計算
        pr = nx.pagerank(nxg)
        pr_max = np.array(list(pr.values())).max()

        # 可視化する際のノード位置
        pos = self.setPos(data)
        draw_pos = nx.spring_layout(nxg, pos=pos, seed=0) 

        cmap = plt.get_cmap('tab10')
        labels = np.zeros(data.num_nodes)
        if label :
            # ノードの色設定
            labels = data.y.numpy()
            print(labels)
        colors = [cmap(l) for l in labels]

        # 図のサイズ
        plt.figure(figsize=(100, 100))

        # 描画
        if ispos :
            nx.draw(nxg, pos, with_labels=True)
        else :
            nx.draw_networkx_nodes(nxg, 
                                draw_pos,
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
# g.showgraph(g.graph, title='sumo', ispos=False, label=False)