import traci
import random
import sys
import numpy as np

from traci.exceptions import TraCIException
import traci.constants as tc
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
            # traci.route.add(routeID, [g.getEdgeID(fromedge), g.getEdgeID(toedge)])
            return g.getEdgeID(fromedge), g.getEdgeID(toedge)
        except TraCIException as routeErr:
            i += 1
            print(i, route.edges)
            if i == 10:
                print(routeErr)
                break
        except Exception as e:
            a, b, c = sys.exc_info()
            print(e, a, b, c)
    return None, None


def addCar(carnum):
    for i in range(carnum):
        v_list = traci.vehicle.getIDList()
        print(v_list)
        vehID = 'veh{}'.format(i)
        routeID = 'route{}'.format(i)
        fromedge, _ = generateRoute(routeID)
        traci.vehicle.add(vehID, routeID, typeID="reroutingType")
        laneID = fromedge + '_{}'.format(0)
        # set speed mode
        traci.vehicle.setSpeedMode(vehID, 0)
        length = traci.vehicle.getLength(vehID)
        length = min(length, traci.lane.getLength(laneID))
        start = g.getEdgeIndex(fromedge)
        if len(route[start]) <= 1:
            traci.vehicle.moveTo(vehID, laneID, length)
            x, y = traci.vehicle.getPosition(vehID)
            if vehID == 'veh0':
                traci.vehicle.moveToXY(vehID, fromedge, 0, x, y, -90.0, 2)
            elif vehID == 'veh1':
                traci.vehicle.moveToXY(vehID, fromedge, 0, x, y, 90.0, 2)
            elif vehID == 'veh2':
                traci.vehicle.moveToXY(vehID, fromedge, 0, x, y, 0.0, 2)
            elif vehID == 'veh3':
                traci.vehicle.moveToXY(vehID, fromedge, 0, x, y, 360.0, 2)
            elif vehID == 'veh4':
                traci.vehicle.moveToXY(vehID, fromedge, 0, x, y, 450.0, 2)
            elif vehID == 'veh5':
                traci.vehicle.moveToXY(vehID, fromedge, 0, x, y, -450.0, 2)
            traci.vehicle.setSpeed(vehID, 11.11)
            vlist = traci.lane.getLastStepVehicleIDs(laneID)
            # showInfo(vlist[0])
            print(vlist)
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
    pos = traci.vehicle.getLanePosition(v)
    v_x, v_y = traci.vehicle.getPosition(v)
    angle = traci.vehicle.getAngle(v)
    dis = traci.vehicle.getDistance(v)
    acc = traci.vehicle.getAccel(v)
    acc2 = traci.vehicle.getAcceleration(v)
    decel = traci.vehicle.getDecel(v)
    accumulate = traci.vehicle.getAccumulatedWaitingTime(v)
    action_stp = traci.vehicle.getActionStepLength(v)
    travel = traci.vehicle.getAdaptedTraveltime(v, step, current_edge)
    allsub = traci.vehicle.getAllContextSubscriptionResults()
    allow = traci.vehicle.getAllowedSpeed(v)
    subresult = traci.vehicle.getAllSubscriptionResults()
    decel2 = traci.vehicle.getApparentDecel(v)
    bestLane = traci.vehicle.getBestLanes(v)
    # drive = traci.vehicle.getDrivingDistance(v, current_edge, pos, laneindex)
    # drive2 = traci.vehicle.getDrivingDistance2D(v, v_x, v_y)
    emergDecel = traci.vehicle.getEmergencyDecel(v)
    follow = traci.vehicle.getFollower(v)
    laneChange = traci.vehicle.getLaneChangeState(v, 0)
    laneChangePretty = traci.vehicle.getLaneChangeStatePretty(v, 0)
    action_time = traci.vehicle.getLastActionTime(v)
    lateralLanePos = traci.vehicle.getLateralLanePosition(v)
    lateralSpeed = traci.vehicle.getLateralSpeed(v)
    leader = traci.vehicle.getLeader(v)
    left = traci.vehicle.getLeftFollowers(v)
    leftleader = traci.vehicle.getLeftLeaders(v)
    line = traci.vehicle.getLine(v)
    maxSpeed = traci.vehicle.getMaxSpeed(v)
    maxLatSpeed = traci.vehicle.getMaxSpeedLat(v)
    gap = traci.vehicle.getMinGap(v)
    latGap = traci.vehicle.getMinGapLat(v)
    neighborLeftAhead = traci.vehicle.getNeighbors(v, 0)
    neighborRightAhead = traci.vehicle.getNeighbors(v, 1)
    neighborLeftBehind = traci.vehicle.getNeighbors(v, 2)
    neighborRightBehind = traci.vehicle.getNeighbors(v, 3)
    rightfollow = traci.vehicle.getRightFollowers(v)
    rightleader = traci.vehicle.getRightLeaders(v)
    slope = traci.vehicle.getSlope(v)
    deviat = traci.vehicle.getSpeedDeviation(v)
    factor = traci.vehicle.getSpeedFactor(v)
    mode = traci.vehicle.getSpeedMode(v)
    speed_without = traci.vehicle.getSpeedWithoutTraCI(v)
    delay = traci.vehicle.getStopDelay(v)
    stopSpeed = traci.vehicle.getStopSpeed(v, speed, gap)
    state = traci.vehicle.getStopState(v)
    tau = traci.vehicle.getTau(v)
    typeID = traci.vehicle.getTypeID(v)
    vclass = traci.vehicle.getVehicleClass(v)
    via = traci.vehicle.getVia(v)
    wait = traci.vehicle.getWaitingTime(v)
    # pos2 = traci.simulation.convert2D(current_edge, pos, laneindex)
    route = traci.vehicle.getRoute(v)
    routeIndex = traci.vehicle.getRouteIndex(v) + 1
    foesList = []
    if routeIndex < len(route):
        nextEdgeID = route[routeIndex]
        laneNum = traci.edge.getLaneNumber(nextEdgeID)
        for lane in range(laneNum):
            nextLaneID = nextEdgeID + '_{}'.format(lane)
            try:
                foes = traci.lane.getFoes(laneID, nextLaneID)
                foesList.append(foes)
            except traci.exceptions.TraCIException as tr:
                print(tr)
    # links = traci.lane.getLinks(laneID)
    # shape = traci.lane.getShape(laneID)
    # lanelen = traci.lane.getLength(laneID)
    connection = g.getConnection()
    connect = []
    edgeIndex = g.getEdgeIndex(current_edge)
    if edgeIndex != -1:
        connect = connection[edgeIndex]
    print(v, "speed", speed, "siganl", signal, "edge", current_edge,
          "laneID", laneID,
          #   "laneindex", laneindex, "len", lanelen,
          "pos", pos, "v_x", v_x, "v_y", v_y, "angle", angle, "dis", dis,
          "max_accel", acc, "cur_accel", acc2, "max_decel", decel,
          #   "position", pos2, "link", links, "connet", connect, "foes", foesList,
          #   "link", links,
          "connet", connect, "foes", foesList,
          #   "shape", shape,
          "step", step, "accumlate", accumulate, "action_stp",
          action_stp, "action_time", action_time, "travel", travel, "allsub",
          allsub, "allow", allow, "subresult", subresult, "decel2", decel2,
          #   "bestlane", bestLane, "drive", drive, "drive2", drive2,
          "bestlane", bestLane,
          #   "drive", drive,
          "emergeDecel", emergDecel, "follow", follow, "lanechange",
          laneChange, "lanechangepretty", laneChangePretty, "lateratLanepos",
          lateralLanePos, "lateralSpeed", lateralSpeed, "leader", leader,
          "left", left, "leftleader", leftleader, "line", line, "maxspeed",
          maxSpeed, "maxLatSpeed", maxLatSpeed, "gap", gap, "latGap", latGap,
          "neighbor leftahead", neighborLeftAhead, "neighbor rightahead",
          neighborRightAhead, "neighbor leftbehind", neighborLeftBehind,
          "neighbor rightbehind", neighborRightBehind, "rightfollow",
          rightfollow, "rightleader", rightleader, "slope", slope, "deviat",
          deviat, "factor", factor, "speedmode", mode, "speed_without",
          speed_without, "delay", delay, "stopspeed", stopSpeed, "state",
          state, "tau", tau, "typeID", typeID, "vclass", vclass, "via", via,
          "wait", wait)


def takeAction():
    v_list = traci.vehicle.getIDList()
    # step = traci.simulation.getTime()
    for v in v_list:
        # index = int(v[3:])
        try:
            speed = traci.vehicle.getSpeed(v)
            # signal = traci.vehicle.getSignals(v)
            # edge & lane
            # current_edge = traci.vehicle.getRoadID(v)
            # laneID = traci.vehicle.getLaneID(v)
            laneindex = traci.vehicle.getLaneIndex(v)
            # lanelen = traci.lane.getLength(laneID)
            # laneshape = traci.lane.getShape(laneID)
            # junction_list = traci.junction.getIDList()
            # junctionshape = traci.junction.getShape(junction_list[0])
            # pos = traci.vehicle.getLanePosition(v)
            v_x, v_y = traci.vehicle.getPosition(v)
            # angle = traci.vehicle.getAngle(v)
            # dis = traci.vehicle.getDistance(v)
            # acc = traci.vehicle.getAccel(v)
            # acc2 = traci.vehicle.getAcceleration(v)
            # decel = traci.vehicle.getDecel(v)
            # traci.vehicle.moveToXY(v, current_edge, laneindex, v_x,
            #                        v_y, angle=angle, keepRoute=4)
            # traci.vehicle.setPreviousSpeed(v, speed)
            # print(v, "speed", speed, "edge", current_edge, "siganl", signal,
            #       "pos", pos, "len", lanelen, "dis", dis, step)
            # time = step
            # if isJunction(v):
            #     routeIndex = traci.vehicle.getRouteIndex(v)
            #     routes = traci.vehicle.getRoute(v)
            #     current_edge = traci.vehicle.getRoadID(v)
            #     print(current_edge, routeIndex, routes,
            #           routes[routeIndex], "routeindex")
            # if step == 3:
            #     turn(v, 'l')
            # print(v, "speed", speed, "edge", current_edge, "siganl", signal,
            #       "pos", pos, "dis", dis, "len", lanelen, "acc2", acc2, step)
            # 現在の速度を計算してから未来の速度を決める方法
            # time = preaccel[index]
            # vnext = speed + time
            # under = max(-4.5, -vnext)
            # futureaccel[index] = random.uniform(under, 2.6)
            # preaccel[index] = futureaccel[index]
            # pos2 = pos + vnext
            # 普通の方法
            # under = max(-4.5, -speed)
            # futureAccel = random.uniform(-decel, acc)
            # vnext = speed + futureAccel
            # traci.vehicle.setSpeed(v, vnext)
            # if vnext < 0:
            #     # back action
            #     time = traci.simulation.getDeltaT()
            #     postmp = pos + vnext * time
            #     traci.vehicle.moveTo(v, laneID, postmp)
            if v == 'veh0':
                print("take action ", v)
                delta = traci.simulation.getDeltaT()
                angle = traci.vehicle.getAngle(v)
                next_x, next_y = vector_decomposition(speed * delta, angle)
                v_next_x, v_next_y = v_x + next_x, v_y + next_y
                traci.vehicle.moveToXY(
                    v, '', laneindex, v_next_x, v_next_y, angle, 2)
                showInfo(v)
            # if v == v_list[0]:
            #     print(v, index, decel, "maxaccel", acc, "pos", pos,
            #           "lanelen", lanelen, "vnext ", "time", time,
            #           "future", futureaccel[index], "pre", preaccel[index],
            #           "speed", speed, "accel", acc2, "testest")
        except traci.exceptions.TraCIException as tr:
            print(tr)
            print(v, "error traci")
        except Exception as ex:
            print(ex)
            print(v, "error exception")


def get_base_angle(angle):
    abs_angle = abs(angle)
    n = int(abs_angle // 360)
    abs_base_angle = abs_angle - float(360 * n)
    base_angle = (abs_base_angle if angle >= 0.0 else abs_base_angle + 360.0)
    return base_angle


def vector_decomposition(vector_length, angle):
    base_angle = get_base_angle(angle)
    base_radian = np.deg2rad(base_angle)
    v_y = vector_length * np.cos(base_radian)
    v_x = vector_length * np.sin(base_radian)
    return v_x, v_y


def changeSpeed(v, accel):
    index = int(v[3:])
    preaccel[index] = accel
    print("pre", preaccel[index], "future", accel)


def isJunction(vehID):
    ID = traci.vehicle.getRoadID(vehID)
    if g.getEdgeIndex(ID) == -1:
        return True
    return False


def main(sumocfg):
    step_length = 1
    sumoCmd = ['sumo-gui', '-c', sumocfg, '--step-length', str(step_length),
               '--tls.all-off', str(True)]

    traci.start(sumoCmd)
    addCar(100)
    # step = 0

    traci.vehicle.subscribe(
        'veh0', (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION, tc.VAR_ANGLE, tc.VAR_NEIGHBORS))
    print(traci.vehicle.getSubscriptionResults('veh0'))
    # lane = traci.lane.getIDList()
    # for i, laneid in enumerate(lane):
    #     edge = traci.lane.getEdgeID(laneid)
    #     num = traci.lane.getLinkNumber(laneid)
    #     links = traci.lane.getLinks(laneid)
    #     print(i, edge, laneid, num)
    #     for link in links:
    #         print(link)

    # traci.simulationStep()
    v_list = traci.vehicle.getIDList()
    # if len(v_list) > 0:
    #     showInfo(v_list[0])
    # step = traci.simulation.getTime()
    traci.simulationStep()
    # step = traci.simulation.getTime()
    # if len(v_list) > 0:
    #     showInfo(v_list[0])
    while True:  # 1day
        v_list = traci.vehicle.getIDList()
        if len(v_list) <= 0:
            break
        # for v in v_list:
        # showInfo('veh0')
        takeAction()
        result = traci.vehicle.getSubscriptionResults('veh0')
        cllidlist = traci.simulation.getCollidingVehiclesIDList()
        arraival = traci.simulation.getArrivedIDList()
        stop = traci.simulation.getStopStartingVehiclesIDList()
        stop2 = traci.simulation.getStopEndingVehiclesIDList()
        teleport = traci.simulation.getStartingTeleportIDList()
        poilist = traci.poi.getIDList()
        for poi in poilist:
            poi_pos = traci.poi.getPosition(poi)
        polygonlist = traci.polygon.getIDList()
        for pol in polygonlist:
            pol_pos = traci.polygon.getShape(pol)
        # if len(v_list) > 0:
        #     showInfo(v_list[0])
        # if step == 6:
        #     turn(v_list[0], 'r')
        # changeSpeed(v_list[0], 1)
        # print("take action")
        # if len(v_list) > 0:
        #     showInfo(v_list[0])
        traci.simulationStep()
        cllidlist = traci.simulation.getCollidingVehiclesIDList()
        arraival = traci.simulation.getArrivedIDList()
        stop = traci.simulation.getStopStartingVehiclesIDList()
        stop2 = traci.simulation.getStopEndingVehiclesIDList()
        teleport = traci.simulation.getStartingTeleportIDList()
        result = traci.vehicle.getSubscriptionResults('veh0')
        # step = traci.simulation.getTime()
        v_list = traci.vehicle.getIDList()
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
            # showInfo(v_list[0])

    traci.close()


# sumocfg = 'map/data1/sample.sumo.cfg'
# sumocfg = 'map/data2/sample.sumo.cfg'
# sumocfg = 'map/data3/osm.sumocfg'
sumocfg = 'map/testmap/osm.sumocfg'
preaccel = random_double(0.0, 2.6, 100)
futureaccel = [0.0] * 100
main(sumocfg)
