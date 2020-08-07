import traci
from gym_sumo.envs.sumo_env import Graph
import random
# YAMATO_TN = '31887784'
# AYASE_BT_SAG = '37038705'
DECELERATION_RATE = 0.9
DECELERATION_DURATION = 3
g = Graph('map/testmap/osm.net.xml')
print(g.graph)
print(len(g.normalEdgeConnection), g.normalEdgeConnection)
route = {}
def isNewRoute(fromnodeindex, tonodeindex):
    if fromnodeindex not in route:
        route[fromnodeindex] = []
        route[fromnodeindex].append(tonodeindex)
        return True
    elif tonodeindex not in route[fromnodeindex]:
        route[fromnodeindex].append(tonodeindex)
        return True
    return False

def random_int(a, b, num):
    ns = []
    while len(ns) < num:
        n = random.randrange(a, b)
        if not n in ns:
            ns.append(n)
    return ns

def generateRoute(routeID):
    i = 0
    while True:
        # print(i, "test", routeID, "route")
        ns = random_int(0, g.num['edge_normal'], 2)
        fromedge = ns[0]
        toedge = ns[1]
        if not isNewRoute(fromedge, toedge):
            continue
        print(fromedge, toedge, routeID)
        try:
            route = traci.simulation.findRoute(g.normalEdgeIDList[fromedge], g.normalEdgeIDList[toedge])
            traci.route.add(routeID, route.edges)
            break
        except traci.exceptions.TraCIException as routeErr:
            i+=1
            # print(i, route.edges)
            if i == 10:
                print(routeErr)
                break

def addCar(carnum):
    for i in range(carnum):
        vehID = 'veh{}'.format(i)
        routeID = 'route{}'.format(i)
        generateRoute(routeID)
        traci.vehicle.add(vehID, routeID)
        #set speed mode
        traci.vehicle.setSpeedMode(vehID, 0)

def main(sumocfg):
    sumoCmd = ['sumo-gui', '-c', sumocfg]

    traci.start(sumoCmd)
    cnt = 0
    for i, edge in enumerate(g.normalEdgeConnection):
        if len(edge) == 0:
            cnt+=1
            fromedge = i
            toedge = 0
            print(i, g.normalEdgeIDList[i])
            try:
                route = traci.simulation.findRoute(g.normalEdgeIDList[fromedge], g.normalEdgeIDList[toedge])
                traci.route.add(routeID, route.edges)
            except:
                print("cannot find route!!")
    print(cnt)
    # traci.close()
    addCar(100)
    step = 0
    accident = False

    slow_down_yamato = []
    slow_down_ayase = []
    

    edgelist = traci.edge.getIDList()
    # fromedge = int(random.uniform(0, g.num['edge_normal']))
    # toedge = int(random.uniform(0, g.num['edge_normal']))
    # route = traci.simulation.findRoute(g.normaledgelist[fromedge], g.normaledgelist[toedge])
    # print(route)
    # lane = traci.lane.getIDList()
    # for i, laneid in enumerate(lane):
    #     edge = traci.lane.getEdgeID(laneid)
    #     num = traci.lane.getLinkNumber(laneid)
    #     links = traci.lane.getLinks(laneid)
    #     print(i, edge, laneid, num)
    #     for link in links:
    #         print(link)

    traci.simulationStep()
    v_list = traci.vehicle.getIDList()
    for v in v_list:
        speed = traci.vehicle.getSpeed(v)
        signal = traci.vehicle.getSignals(v)
        # slow down at sag
        current_edge = traci.vehicle.getRoadID(v)
        pos = traci.vehicle.getLanePosition(v)
        dis = traci.vehicle.getDistance(v)
        param = traci.edge.getParameter(current_edge, "from")
        print(v, "speed", speed, "edge", current_edge, "siganl", signal, "pos", pos, "dis", dis, step)
    while step < 60 * 60 * 24: # 1day
        traci.simulationStep()

        # arrived vehicle
        # arrive_list = traci.simulation.getArrivedIDList()
        # for v in arrive_list:
        #     if v in slow_down_ayase:
        #         slow_down_ayase.remove(v)
        #     if v in slow_down_yamato:
        #         slow_down_yamato.remove(v)
        v_list = traci.vehicle.getIDList()
        if len(v_list) == 0:
            traci.close()
            break
        for v in v_list:
            if step == 0:
                traci.vehicle.setSpeed(v, 10)
            if step == 5:
                traci.vehicle.setSpeed(v, 15)
            # if step >= 6:
            #     traci.vehicle.setSignals(v, 1)
            speed = traci.vehicle.getSpeed(v)
            signal = traci.vehicle.getSignals(v)
            # slow down at sag
            current_edge = traci.vehicle.getRoadID(v)
            pos = traci.vehicle.getLanePosition(v)
            dis = traci.vehicle.getDistance(v)
            param = traci.edge.getParameter(current_edge, "from")
            print(v, "speed", speed, "edge", current_edge, "siganl", signal, "pos", pos, "dis", dis, step)
        step += 1
    traci.close()


# sumocfg = 'map/data1/sample.sumo.cfg'
# sumocfg = 'map/data2/sample.sumo.cfg'
# sumocfg = 'map/data3/osm.sumocfg'
sumocfg = 'map/testmap/osm.sumocfg'
main(sumocfg)