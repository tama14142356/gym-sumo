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
import gym
# import gym_sumo
import random

env = gym.make('gym_sumo:sumo-v0')
# print("env", env.graph.normalEdgeConnection)
# env.reset()
print(env.routeEdge, "route")
action = []
for i in range(100):
    tmp = -1.0 + float(random.randint(0, i))
    tmp2 = random.randint(0, 4)
    if i % 10 == 3:
        tmp2 = 6
    action.append((tmp2, tmp))

env.step(action)
print(env.render())
env.close()
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