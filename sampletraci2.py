import traci
import random

from gym_sumo.envs.sumo_env import Graph

g = Graph('map/testmap/osm.net.xml')
graph = g.getGraph()
# print(g.graph)
# print(len(g.normalEdgeConnection), g.normalEdgeConnection)
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
        if n not in ns:
            ns.append(n)
    return ns


def random_double(a, b, num):
    ns = []
    while len(ns) < num:
        n = random.uniform(a, b)
        if n not in ns:
            ns.append(n)
    return ns


def generateRoute(routeID):
    i = 0
    while True:
        # print(i, "test", routeID, "route")
        ns = random_int(0, g.getNum('edge_normal'), 2)
        fromedge = ns[0]
        toedge = ns[1]
        if not isNewRoute(fromedge, toedge):
            continue
        # print(fromedge, toedge, routeID)
        try:
            route = traci.simulation.findRoute(g.getEdgeID(fromedge),
                                               g.getEdgeID(toedge))
            traci.route.add(routeID, route.edges)
            return g.getEdgeID(fromedge), g.getEdgeID(toedge)
        except traci.exceptions.TraCIException as routeErr:
            i += 1
            # print(i, route.edges)
            if i == 10:
                print(routeErr)
                break
    return None, None


def addCar(carnum):
    for i in range(carnum):
        vehID = 'veh{}'.format(i)
        routeID = 'route{}'.format(i)
        fromedge, _ = generateRoute(routeID)
        traci.vehicle.add(vehID, routeID)
        laneID = fromedge + '_{}'.format(0)
        # set speed mode
        traci.vehicle.setSpeedMode(vehID, 0)
        length = traci.vehicle.getLength(vehID)
        length = min(length, traci.lane.getLength(laneID))
        start = g.getEdgeIndex(fromedge)
        if len(route[start]) <= 1:
            traci.vehicle.moveTo(vehID, laneID, length)
            traci.vehicle.setSpeed(vehID, 0.0)
# def isStreet(vehID):


def turn(v, direction):
    # if v == 'veh99':
    current_edge = traci.vehicle.getRoadID(v)
    curLaneIndex = traci.vehicle.getLaneIndex(v)
    print(v)
    nextEdgeID = g.getNextInfoTo(current_edge, str(curLaneIndex), direction)
    laneIndex = g.getNextInfoFrom(current_edge, str(curLaneIndex), direction)
    if nextEdgeID is None:
        if v == 'veh0':
            print(v, "eeeee")
        return False
    else:
        if laneIndex != str(curLaneIndex):
            return False
        if v == 'veh0':
            print("yyy")
        nextLaneID = nextEdgeID + '_{}'.format(0)
        # nodeIndex = g.getNodeIndex(None, nextEdgeID, False)
        # nodeID = g.getNodeID(nodeIndex)
        # angle = traci.vehicle.getAngle(v)
        # if nodeID is not None:
        #     x, y = traci.junction.getPosition(nodeID)
        #     traci.vehicle.moveToXY(v, nextEdgeID, 0, x, y, angle=angle,
        #                            keepRoute=4)
        pos = min(traci.vehicle.getLength(v),
                  traci.lane.getLength(nextLaneID))
        # traci.vehicle.moveTo(v, nextLaneID, pos)
        # if v == 'veh0':
        #     print(v, "edgeID", nextEdgeID, "laneID", nextLaneID,
        #           "nodeindex", nodeIndex, "nodeID", nodeID)
        routeList = traci.vehicle.getRoute(v)
        newroute = traci.simulation.findRoute(nextEdgeID,
                                              routeList[len(routeList) - 1])
        # print(newroute, "newroute")
        newedgeList = [current_edge]
        newedgeList[len(newedgeList):len(newroute.edges)] = newroute.edges
        if len(newroute.edges) == 0:
            print(v, "empty")
            newedgeList.append(nextEdgeID)
        traci.vehicle.setRoute(v, newedgeList)
        traci.vehicle.moveTo(v, nextLaneID, pos)

    return True


def showInfo(v):
    step = traci.simulation.getTime()
    speed = traci.vehicle.getSpeed(v)
    signal = traci.vehicle.getSignals(v)
    # edge & lane
    current_edge = traci.vehicle.getRoadID(v)
    laneID = traci.vehicle.getLaneID(v)
    laneindex = traci.vehicle.getLaneIndex(v)
    lanelen = traci.lane.getLength(laneID)
    pos = traci.vehicle.getLanePosition(v)
    v_x, v_y = traci.vehicle.getPosition(v)
    angle = traci.vehicle.getAngle(v)
    dis = traci.vehicle.getDistance(v)
    acc = traci.vehicle.getAccel(v)
    acc2 = traci.vehicle.getAcceleration(v)
    decel = traci.vehicle.getDecel(v)
    pos2 = traci.simulation.convert2D(current_edge, pos, laneindex)
    links = traci.lane.getLinks(laneID)
    shape = traci.lane.getShape(laneID)
    route = traci.vehicle.getRoute(v)
    routeIndex = traci.vehicle.getRouteIndex(v) + 1
    foesList = []
    if routeIndex < len(route):
        nextEdgeID = route[routeIndex]
        laneNum = traci.edge.getLaneNumber(nextEdgeID)
        for lane in range(laneNum):
            nextLaneID = nextEdgeID + '_{}'.format(lane)
            foes = traci.lane.getFoes(laneID, nextLaneID)
            foesList.append(foes)
    connection = g.getConnection()
    connect = []
    edgeIndex = g.getEdgeIndex(current_edge)
    if edgeIndex != -1:
        connect = connection[edgeIndex]
    print(v, "speed", speed, "siganl", signal, "edge", current_edge,
          "laneID", laneID, "laneindex", laneindex, "len", lanelen,
          "pos", pos, "v_x", v_x, "v_y", v_y, "angle", angle, "dis", dis,
          "max_accel", acc, "cur_accel", acc2, "max_decel", decel,
          "position", pos2, "link", links, "connet", connect, "foes", foesList,
          "shape", shape, "step", step)


def takeAction():
    v_list = traci.vehicle.getIDList()
    step = traci.simulation.getTime()
    for v in v_list:
        index = int(v[3:])
        speed = traci.vehicle.getSpeed(v)
        # signal = traci.vehicle.getSignals(v)
        # edge & lane
        # current_edge = traci.vehicle.getRoadID(v)
        laneID = traci.vehicle.getLaneID(v)
        # laneindex = traci.vehicle.getLaneIndex(v)
        lanelen = traci.lane.getLength(laneID)
        pos = traci.vehicle.getLanePosition(v)
        v_x, v_y = traci.vehicle.getPosition(v)
        # angle = traci.vehicle.getAngle(v)
        # dis = traci.vehicle.getDistance(v)
        # acc = traci.vehicle.getAccel(v)
        acc2 = traci.vehicle.getAcceleration(v)
        decel = traci.vehicle.getDecel(v)
        # traci.vehicle.moveToXY(v, current_edge, laneindex, v_x,
        #                        v_y, angle=angle, keepRoute=4)
        # traci.vehicle.setPreviousSpeed(v, speed)
        # print(v, "speed", speed, "edge", current_edge, "siganl", signal,
        #       "pos", pos, "len", lanelen, "dis", dis, step)
        time = step
        if step == 3:
            turn(v, 'l')
        # print(v, "speed", speed, "edge", current_edge, "siganl", signal, 
        #       "pos", pos, "dis", dis, "len", lanelen, "acc2", acc2, step)
        time = preaccel[index]
        vnext = speed + time
        under = max(-4.5, -vnext)
        futureaccel[index] = random.uniform(under, 2.6)
        preaccel[index] = futureaccel[index]
        pos2 = pos + vnext
        traci.vehicle.setSpeed(v, vnext)
        # traci.vehicle.moveTo(v, laneID, pos2)
        if v == v_list[0]:
            print(v, index, decel, "pos2", pos2, "pos", pos,
                  "lanelen", lanelen, "vnext", vnext, "time", time,
                  "future", futureaccel[index], "pre", preaccel[index],
                  "speed", speed, "accel", acc2, "testest")


def changeSpeed(v, accel):
    index = int(v[3:])
    preaccel[index] = accel
    print("pre", preaccel[index], "future", accel)


def main(sumocfg):
    step_length = 1
    sumoCmd = ['sumo-gui', '-c', sumocfg, '--step-length', str(step_length)]

    traci.start(sumoCmd)
    addCar(100)
    step = 0

    # lane = traci.lane.getIDList()
    # for i, laneid in enumerate(lane):
    #     edge = traci.lane.getEdgeID(laneid)
    #     num = traci.lane.getLinkNumber(laneid)
    #     links = traci.lane.getLinks(laneid)
    #     print(i, edge, laneid, num)
    #     for link in links:
    #         print(link)

    print("testetset222")
    # traci.simulationStep()
    print("testetset")
    v_list = traci.vehicle.getIDList()
    # if len(v_list) > 0:
    #     showInfo(v_list[0])
    print("testetset3333")
    step = traci.simulation.getTime()
    print(step)
    traci.simulationStep()
    step = traci.simulation.getTime()
    print(step, "step2")
    # if len(v_list) > 0:
    #     showInfo(v_list[0])
    while step < 60 * 60 * 24:  # 1day
        print("testetset4444")
        v_list = traci.vehicle.getIDList()
        if len(v_list) > 0:
            showInfo(v_list[0])
        takeAction()
        print("take action")
        # if len(v_list) > 0:
        #     showInfo(v_list[0])
        if step == 6:
            turn(v_list[0], 'r')
        # changeSpeed(v_list[0], 1)
        # print("take action")
        # if len(v_list) > 0:
            # showInfo(v_list[0])
        traci.simulationStep()
        step = traci.simulation.getTime()
        v_list = traci.vehicle.getIDList()
        print("simulation step")
        if len(v_list) == 0:
            traci.close()
            break
        # showInfo(v_list[0])
        # takeAction()
        # if step == 6:
        #     turn(v_list[0], 'r')
        # changeSpeed(v_list[0], 1)
        # print("take action")
        # if len(v_list) > 0:
        #     showInfo(v_list[0])
            
    traci.close()


# sumocfg = 'map/data1/sample.sumo.cfg'
# sumocfg = 'map/data2/sample.sumo.cfg'
# sumocfg = 'map/data3/osm.sumocfg'
sumocfg = 'map/testmap/osm.sumocfg'
preaccel = random_double(0.0, 2.6, 100)
futureaccel = [0.0] * 100
main(sumocfg)