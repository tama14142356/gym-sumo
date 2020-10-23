# import traci

# YAMATO_TN = '31887784'
# AYASE_BT_SAG = '37038705'
# DECELERATION_RATE = 0.9
# DECELERATION_DURATION = 3


# def main(sumocfg):
#     sumoCmd = ['sumo-gui', '-c', sumocfg]

#     traci.start(sumoCmd)
#     step = 0
#     # accident = False

#     slow_down_yamato = []
#     slow_down_ayase = []
#     traci.vehicle.add('veh0', 'route0', )

#     while step < 60 * 60 * 24:  # 1day
#         traci.simulationStep()

#         # arrived vehicle
#         arrive_list = traci.simulation.getArrivedIDList()
#         for v in arrive_list:
#             if v in slow_down_ayase:
#                 slow_down_ayase.remove(v)
#             if v in slow_down_yamato:
#                 slow_down_yamato.remove(v)

#         if step % 5 == 0:
#             v_list = traci.vehicle.getIDList()
#             for v in v_list:
#                 speed = traci.vehicle.getSpeed(v)

#                 # slow down at sag
#                 current_edge = traci.vehicle.getRoadID(v)
#                 if current_edge == YAMATO_TN and v not in slow_down_yamato:
#                     traci.vehicle.slowDown(
#                         v,
#                         speed * DECELERATION_RATE,
#                         DECELERATION_DURATION)
#                     slow_down_yamato.append(v)
#                 elif (current_edge == AYASE_BT_SAG
#                       and v not in slow_down_ayase):
#                     traci.vehicle.slowDown(
#                         v,
#                         speed * DECELERATION_RATE,
#                         DECELERATION_DURATION)
#                     slow_down_ayase.append(v)
#                 print(v, speed, current_edge)
#         step += 1
#     traci.close()


# # sumocfg = 'map/data1/sample.sumo.cfg'
# # sumocfg = 'map/data2/sample.sumo.cfg'
# sumocfg = 'map/data3/osm.sumocfg'
# # sumocfg = 'map/data3/osm2.sumocfg'
# main(sumocfg)
# import gym
# # import gym_sumo
# import random

# env = gym.make('gym_sumo:sumo-v0')
# # print("env", env.graph.normalEdgeConnection)
# # env.reset()
# print(env.routeEdge, "route")
# action = []
# for i in range(100):
#     tmp = -1.0 + float(random.randint(0, i))
#     tmp2 = random.randint(0, 4)
#     if i % 10 == 3:
#         tmp2 = 6
#     action.append((tmp2, tmp))

# env.step(action)
# print(env.render())
# env.close()
# for i, edgeID in enumerate(env.graph.normalEdgeIDList):
#     edgeIndex = env.graph.normalEdgeIDdict[edgeID]
#     print(edgeID, i, edgeIndex)
#     if i != edgeIndex:
#         print(edgeID, edgeIndex, i, "failure!!")
# edgeindex = env.graph.normalEdgeIDdict['39436461#0']
# test = env.graph.normalEdgeConnection[521]
# print(test, edgeindex)
# if test is None:
#     print("yes")
# if len(test) == 0:
#     print("testyes")
# from samplexml import Test

# t = Test()
# t.test()
import traci
import traci.constants as tc
from traci.exceptions import TraCIException
from gym_sumo.envs._graph import Graph
from gym_sumo.envs._util import randomTuple
from gym.utils import seeding
# import the library
import sumolib
# parse the net
net = sumolib.net.readNet('gym-sumo/gym_sumo/envs/sumo_configs/nishiwaseda/osm.net.xml')
# retrieve the coordinate of a node based on its ID
print(net.getNode('1093469771').getCoord())
# net.getEdge()
# retrieve the successor node ID of an edge
nextNodeID = net.getEdge('-98726975').getToNode().getID()
edge = net.getEdge('-98726975')
__graph = Graph("gym-sumo/gym_sumo/envs/sumo_configs/nishiwaseda/osm.net.xml")
__vehIDList = {}
routeEdge = []
seed = None
np_random, seed = seeding.np_random(seed)


def addCar(carnum):
    for i in range(carnum):
        vehID = 'veh{}'.format(i)
        routeID = 'route{}'.format(i)
        __vehIDList[vehID] = routeID
        fromEdgeID, _ = generateRoute(routeID)
        start = __graph.getEdgeIndex(fromEdgeID)
        if start not in routeEdge:
            routeEdge.append(start)
            insertCar(vehID, routeID, fromEdgeID)


def insertCar(vehID, routeID, startEdgeID=None):
    if startEdgeID is None:
        startEdgeID = traci.route.getEdges(routeID)[0]
    # register car
    traci.vehicle.add(vehID, routeID)
    # set speed mode
    traci.vehicle.setSpeedMode(vehID, 0)
    laneID = startEdgeID + '_0'
    length = min(traci.vehicle.getLength(vehID),
                 traci.lane.getLength(laneID))
    # insert car instantly
    traci.vehicle.moveTo(vehID, laneID, length)
    traci.vehicle.setSpeed(vehID, 11.0)


def generateRoute(routeID):
    num_edge = __graph.getNum('edge_normal') - 1
    fromEdgeID = None
    toEdgeID = None
    for i in range(10):
        # print(i, "test", routeID, "route")
        edges = randomTuple(0, num_edge, 2, routeEdge, np_random)
        fromEdgeID = __graph.getEdgeID(edges[0])
        toEdgeID = __graph.getEdgeID(edges[1])
        try:
            route = traci.simulation.findRoute(fromEdgeID, toEdgeID)
            if len(route.edges) > 0:
                traci.route.add(routeID, route.edges)
                break
        except TraCIException as tr:
            print(tr)
    return fromEdgeID, toEdgeID


traci.start(["sumo-gui", "-c",
             "gym-sumo/gym_sumo/envs/sumo_configs/nishiwaseda/osm.sumocfg"])
addCar(100)
veh_list = traci.vehicle.getIDList()
vehID = veh_list[0]
traci.vehicle.subscribe(
    vehID, (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION, tc.VAR_ANGLE))
# , tc.VAR_NEIGHBORS
#  tc.VAR_POSITION, tc.VAR_PREV_SPEED,
                                # tc.VAR_REMOVED, tc.VAR_ROUTE_INDEX, tc.VAR_SCREENSHOT, tc.VAR_SHAPE, tc.VAR_SIGNALS, tc.VAR_SPEED, tc.VAR_STAGE))
# , tc.VAR_FOES, tc.VAR_FOLLOWER, tc.VAR_HAS_VIEW, tc.VAR_LANE_ID, tc.VAR_LANE_INDEX, tc.VAR_LEADER,
#                                 tc.VAR_MOVE_TO,
print(traci.vehicle.getSubscriptionResults(vehID))
for step in range(3):
    print("step", step)
    traci.simulationStep()
    print(traci.vehicle.getSubscriptionResults(vehID))
traci.close()
