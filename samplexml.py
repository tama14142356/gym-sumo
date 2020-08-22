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
# class myTest:
#     def __init__(self):
#         self.testtest = 1
#     def test(self):
#         if 'newv' not in dir(self):
#             self.__newv = 0
#         return self.__newv

# test = myTest()
# print(test.test())
# if '__newv' in dir(test):
#     print("yes")
# else:
#     print("no")
# test = [1, 2, 3, 4]
# test2 = [5, 3, 8]
# test4 = (-1, -2)
# test.append(test2)
# test3 = [test, test2]
# test[len(test):len(test2)] = test2
# test2[len(test2):len(test)] = test
# test3 = test2
# tmp = [{} for i in range(10)]
# for i in range(10):
#     veh = 'test{}'.format(i)
#     tmp[i][veh] = i
# print(tmp)
# tmp = [{} for i in range(10)]
# for i in range(10):
#     veh = 'test{}'.format(i)
#     lan = 'lan{}'.format(0)
#     t = {}
#     tt = []
#     if veh in tmp[i]:
#         t = tmp[i][veh]
#         if lan in t:
#             tt = t[lan]
#     tt.append(i)
#     t[lan] = tt
#     tmp[i][veh] = t
# print(tmp)
# for i in range(10):
#     veh = 'test{}'.format(i)
#     lan = 'lan{}'.format(1)
#     t = {}
#     tt = []
#     if veh in tmp[i]:
#         t = tmp[i][veh]
#         if lan in t:
#             tt = t[lan]
#     tt.append(i)
#     t[lan] = tt
#     tmp[i][veh] = t
# # tmp = [[]*1 for i in range(10)]
# print(tmp)
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
# np.append(tmp[0], 3)
# np.append(tmp[1], edge[1])
# edge_index = torch.tensor(tmp,dtype=torch.long)
# print(edge_index, isNode, "add2")
# add position of vehicle as node position
# tmp = data.pos.numpy()
# pos = torch.tensor(tmp, dtype=torch.float)
# add feature of vehicle as node
# if self.isNode_attr:
#     tmp = data.x.numpy()
# else:
#     tmp = np.zeros(data.num_nodes, dtype=float)
# if isNode:
#     tmp[nodeindex] = vehinfo['speed']
# else:
#     np.append(tmp, vehinfo['speed'])
# x = torch.tensor(tmp, dtype=torch.float)
# add feature of edge(veh-> tonode)
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
# test \
#     = 'y'
# test = ['y', 't',
#         'h', 'k']
# i = 0
# i += 1
# print(i)
# start = 2
# for i in range(start, 10):
#     print(i)

# test = [{'to':"value", 'via':1}, {'to'}]
# class myData:
#     def __init__(self):
#         self.x = 0
#         self.y = 1
#         self.listx = [12, 2, 4, 5]
#         self.tests = []
#         self.add(10)
    
#     def add(self, num):
#         for i in range(num):
#             t = TestXml()
#             self.tests.append(t)


# class TestXml(myData):
#     def __init__(self):
#         self.__member = 0
#         self.__listmenmber = [2, 3, 7, 9]

#     def getmember(self):
#         return self.__member
    
#     def getlist(self):
#         return self.__listmenmber

# mydata = myData()
# print(mydata.listx, mydata.tests)
# print(mydata.tests[0].getlist())
# test = TestXml()
# print(test.getmember(), test.getlist(), test.getData())
# testlist = test.getlist()
# for i in testlist:
#     print(i)
# for i in test.getlist():
#     print(i, "get")
# print(testlist[0])
# print(test.getlist()[0])
# testdata = test.getData()
# print(testdata.x, testdata.listx)
# print(test.getData().x, test.getData().listx)
# print(testdata.listx[0])
# print(test.getData().listx[0])

# from torch_geometric.data import Data
# graph = Data()
# if type(graph) is Data:
#     print("ys")
# else:
#     print("no")
# test = -1

# test = 0.0 if test < 0 else test
# print(test)
# test = [None, None, 'l', '0']
# print(test)
# test[0] = '12810_11'
# print(test)
# test[1] = ':kljlsa_isj'
# print(test)
# test = (-1, -1)
# print(test)
# test[0] = 1
# print(test)
# test = '131231_0_12'
# index = test.rfind('_') + 1
# print(test[index:])

# def test():
#     return 1, 2

# t = (test())
# print(t)
# test = [('aka', True, False, 'l', 'jkj')]
# test = [('aka', True, False, 'l', 'jkj'), ('aka', True, False, 's', 'kk')]
# if 'l' in test[0]:
#     print("yy")
# else:
#     print("no")

# from gym import spaces
# import numpy as np

# action = [spaces.Discrete(3),
#           spaces.Box(low=-1, high=1, shape=(1, ), dtype=np.float32),
#           spaces.Discrete(5),
#           spaces.Box(low=-1, high=1, shape=(3, ), dtype=np.float32)]
# tmp2 = spaces.Tuple(action)
# print(tmp2)
# for i in range(10):
#     tmp = spaces.Box(low=np.array([-1.0], dtype=np.float32),
#                      high=np.array([1.0], dtype=np.float32),
#                      dtype=np.float32)
#     action.append(spaces.Tuple((spaces.Discrete(5), tmp)))

# action = []
# for i in range(10):
#     tmp = -1.0 + float(i)
#     tmp2 = i % 5
#     action.append(spaces.Tuple((tmp2, tmp)))
# print(action)

# print(action)
test = {0: [1, 2, 3], 1: [0, 2, 8], 2: [2, 2, 5]}
if 0 in test:
    print("yy")
else:
    print("no")
