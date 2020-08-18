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
        self.nodeIDdict = {} # dictionary: {'nodeID': nodeindex}
        self.normalEdgeIDList = [] # list of normal edgeID 
        self.normalEdgeIDdict = {} # dictionary: {'edgeID': edgeindex}
        self.normalEdgedict = {} # dictionary: {'edgeID':(fromnodeindex, tonodeindex)}
        self.edgeIDList = [] # list of edgeID
        self.edges = None
        self.num = defaultdict(int)
        self.isEdge_attr = isEdge_attr
        self.isNode_attr = isNode_attr
        # create graph
        self.defineGraph()
        self.gettuplePos(self.graph)
    
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
                    [fromedgeindex] = {'fromlane': {'direction':[toedgelist]}}
        """
        if root is not None:
            num_edges = self.num['edge_normal']
            self.normalEdgeConnection = [{} for i in range(num_edges)]
            connectionList = self.getEdgeConnection(root)
            for connect in connectionList:
                fromedge = self.normalEdgeIDdict[connect['from']]
                fromlane = connect['fromLane']
                toedge = self.normalEdgeIDdict[connect['to']]
                # tolane = connect['toLane']
                direction = connect['dir']
                tmp = {}
                tmplist = []
                if fromlane in self.normalEdgeConnection[fromedge]:
                    tmp = self.normalEdgeConnection[fromedge][fromlane]
                    if direction in tmp:
                        tmplist = tmp[direction]
                tmplist.append(toedge)
                tmp[direction] = tmplist
                self.normalEdgeConnection[fromedge][fromlane] = tmp
        else:
            num_edges = self.graph.num_edges
            self.normalEdgeConnection = [[] * 1 for i in range(num_edges)]
            edgeList = self.graph.edge_index.numpy()
            for i in range(num_edges):
                fromedge = edgeList[0][i]
                toedge = edgeList[1][i]
                if toedge not in self.normalEdgeConnection[fromedge]:
                    self.normalEdgeConnection[fromedge].append(toedge)

    def calcDistance(self, fromPos, toPos):
        return float(np.sqrt((toPos[0] - fromPos[0])**2 + (toPos[1] - fromPos[1])**2))

    def internalEdgeToNode(self, edgeID):
        edge = edgeID[1:]
        length = len(edge)
        index = length - 1
        for i in range(length - 1, 0, -1):
            index = i
            if edge[i] == '_':
                break
        nodeID = edge[:index]
        return nodeID

    # add vehicle as node
    def addNode(self, vehID, curedgeID, vehinfo):
        isNode = False
        # add edge to graph
        tmp = self.graph.edge_index.numpy()
        if curedgeID not in self.normalEdgedict:
            isNode = True
            curnodeID = self.internalEdgeToNode(curedgeID)
            nodeindex = self.nodeIDdict[curnodeID]
        else:
            edge = self.normalEdgedict[curedgeID]
            # add vehicleID as nodeID
            self.addnodeID(vehID)
            src = np.append(tmp[0], self.nodeIDdict[vehID])
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
            tmp[nodeindex] = vehinfo['speed']
        else:
            tmp = np.append(tmp, [vehinfo['speed']], axis=0)
        x = torch.tensor(tmp, dtype=torch.float)
        # add feature of edge(veh-> tonode)
        if self.isEdge_attr:
            tmp = self.graph.edge_attr.numpy()
            if not isNode:
                edgenum = self.normalEdgeIDdict[curedgeID]
                attrib = [tmp[edgenum][0], distance]
                tmp = np.append(tmp, [attrib], axis=0)
            edge_attr = torch.tensor(tmp, dtype=torch.float)
            self.graph = Data(x = x, edge_index=edge_index, edge_attr=edge_attr, pos=pos)
        else:
            self.graph = Data(x = x, edge_index=edge_index, pos=pos)

    def getnodeID(self, nodeNum):
        return self.nodeIDList[nodeNum]

    def getnodeNum(self, nodeID):
        return self.nodeIDdict[nodeID]

    def getEdges(self):
        if self.edges is not None:
            return self.edges
        self.nxg = to_networkx(self.graph)
        tmplist = list(self.nxg.edges)
        self.edges = []
        for i, tmp in enumerate(tmplist):
            edge = [list(self.graph.edge_attr[i].numpy()), list(tmp)]
            self.edges.append(edge)
        return self.edges

    def addnodeID(self, nodeID):
        if nodeID not in self.nodeIDdict:
            value = len(self.nodeIDList)
            self.nodeIDdict[nodeID] = value
            self.nodeIDList.append(nodeID)
    
    def getFeatureNode(self, attrib):
        speed = 0.0
        return [speed]
    
    def getPosNode(self, attrib):
        self.addnodeID(attrib['id'])
        pos = [float(attrib['x']), float(attrib['y'])]
        return pos

    def generateNode(self, root):
        poslist = []
        xlist = []
        for child in root:
            if child.tag == 'junction':
                self.num['junction'] += 1
                if child.attrib['type'] is None or child.attrib['type'] != 'internal':
                    poslist.append(self.getPosNode(child.attrib))
                    xlist.append(self.getFeatureNode(child.attrib))
        x = torch.tensor(xlist, dtype=torch.float)
        pos = torch.tensor(poslist, dtype=torch.float)
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
        fromnodeID = attrib['from']
        tonodeID = attrib['to']

        # nodeID(string) -> torch.long
        self.addnodeID(fromnodeID)
        self.addnodeID(tonodeID)

        # add edge, node to graph
        fromnodeNumber = self.nodeIDdict[fromnodeID]
        tonodeNumber = self.nodeIDdict[tonodeID]
        self.normalEdgedict[attrib['id']] = (fromnodeNumber, tonodeNumber)
        return fromnodeNumber, tonodeNumber
    
    def generateEdge(self, root):
        fromlist = []
        tolist = []
        xlist = []
        for child in root:
            if child.tag == 'edge':
                self.num['edge'] += 1
                self.edgeIDList.append(child.attrib['id'])
                # avoid internel edges
                if 'from' in child.attrib:
                    if 'function' in child.attrib:
                        if child.attrib['function'] == 'internal':
                            continue
                    self.normalEdgeIDList.append(child.attrib['id'])
                    self.normalEdgeIDdict[child.attrib['id']] = self.num['edge_normal']
                    self.num['edge_normal'] += 1
                    # generate edge_index
                    fromnodeNumber, tonodeNumber = self.convNodeLong(child.attrib)
                    fromlist.append(fromnodeNumber)
                    tolist.append(tonodeNumber)
                    # get feature of edge(length)
                    edgespeed, edgelength = self.getFeatureEdge(child)
                    x = [edgespeed, edgelength]
                    xlist.append(x)
        edge_attr = torch.tensor(xlist, dtype=torch.float)
        edge_index = torch.tensor([fromlist, tolist], dtype=torch.long)
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