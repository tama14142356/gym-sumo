
# sumocfg = 'testmap/osm.sumocfg'
# sumoCmd = ['sumo-gui', '-c', sumocfg]
# traci.start(sumoCmd)
# step=0
# edgelist = traci.edge.getIDList()
# # print(dict(edgelist))

# while step < 60 * 60 * 24: # 1day
#     traci.simulationStep()
#     step += 1
# # traci.close()
# import numpy as np
# import random
# tmp = {}
# for i in range(3):
#     tmp[i] = i
# print(tmp)
# tmp = np.zeros((3, 1))
# t = [1]
# print(tmp)
# tmp = np.append(tmp, [t], axis=0)
# print(tmp)
tmp = [{} for i in range(10)]
for i in range(10):
    veh = 'test{}'.format(i)
    tmp[i][veh] = i
print(tmp)
tmp = [{} for i in range(10)]
for i in range(10):
    veh = 'test{}'.format(i)
    lan = 'lan{}'.format(0)
    t = {}
    tt = []
    if veh in tmp[i]:
        t = tmp[i][veh]
        if lan in t:
            tt = t[lan]
    tt.append(i)    
    t[lan] = tt
    tmp[i][veh] = t
print(tmp)
for i in range(10):
    veh = 'test{}'.format(i)
    lan = 'lan{}'.format(1)
    t = {}
    tt = []
    if veh in tmp[i]:
        t = tmp[i][veh]
        if lan in t: 
            tt = t[lan]
    tt.append(i)
    t[lan] = tt
    tmp[i][veh] = t
# tmp = [[]*1 for i in range(10)]
print(tmp)
# for i in range(10):
#     k = random.randint(0, 9)
#     # if i == 6:
#     #     k = 0
#     # if i == 3:
#     #     k = 0
#     for j in range(i + 1):
#         if j not in tmp[k]:
#             tmp[k].append(j)
#     # tmp[k] = tt
#     print(k, tmp)
# print(tmp, "final")
# import re
# tmp = ":::190909_00_00"
# m = re.search(r'\d+', tmp)
# print(m.group())
# len = len(tmp)
# index = len -1
# for i in range(len - 1, 0, -1):
#     index = i
#     if tmp[i] == '_':
#         break
# print(len, index, tmp[:index])

# import xml.etree.ElementTree as ET
# import torch
# from torch_geometric.data import Data
# from collections import defaultdict
# import numpy as np
# import random
# import os

# import networkx as nx
# from torch_geometric.utils import to_networkx
# from matplotlib import pyplot as plt

# src = [0, 1, 2]
# dst = [1, 2, 1]

# edge_index = torch.tensor([src, dst], dtype=torch.long)

# xlist = [[1, 2], [3, 4], [5, 6]]
# x = torch.tensor(xlist, dtype=torch.float)

# edge = [[1.0, 2.0], [2.5, 1.0], [2.0, 4.0]]
# edge_attr = torch.tensor(edge, dtype=torch.float)

# poslist = [[2.0, 5.5], [3.0, 4.5], [2.1, 9.0]]
# pos = torch.tensor(poslist, dtype=torch.float)

# data = Data(x=x, edge_attr=edge_attr, edge_index=edge_index, pos=pos)
# print(data)

# tmp = data.edge_index.numpy()
# print(data.edge_index, "add1")
# # if curedgeID not in self.normalEdgedict:
# #     isNode = True
# #     curnodeID = self.internalEdgeToNode(curedgeID)
# #     nodeindex = self.nodeIDdict[curnodeID]
# # else:
# edge = (0, 1)
# # add vehicleID as nodeID
# # self.addnodeID(vehID)
# np.append(tmp[0], 3)
# np.append(tmp[1], edge[1])
# edge_index = torch.tensor(tmp,dtype=torch.long)
# print(edge_index, isNode, "add2")
# # add position of vehicle as node position
# tmp = data.pos.numpy()
# if not isNode:
#     np.append(tmp, vehinfo['pos'])
#     # calculation edge(veh->tonode) length 
#     distance = self.calcDistance(vehinfo['pos'], tmp[edge[1]])
# pos = torch.tensor(tmp, dtype=torch.float)
# # add feature of vehicle as node
# if self.isNode_attr:
#     tmp = data.x.numpy()
# else:
#     tmp = np.zeros(data.num_nodes, dtype=float)
# if isNode:
#     tmp[nodeindex] = vehinfo['speed']
# else:
#     np.append(tmp, vehinfo['speed'])
# x = torch.tensor(tmp, dtype=torch.float)
# # add feature of edge(veh-> tonode)
# if self.isEdge_attr:
#     tmp = data.edge_attr.numpy()
#     if not isNode:
#         edgenum = self.normalEdgeIDdict[curedgeID]
#         attrib = [tmp[edgenum][0], distance]
#         np.append(tmp, attrib)
#     edge_attr = torch.tensor(tmp, dtype=torch.float)
#     data = Data(x = x, edge_index=edge_index, edge_attr=edge_attr, pos=pos)
# else:
#     data = Data(x = x, edge_index=edge_index, pos=pos)