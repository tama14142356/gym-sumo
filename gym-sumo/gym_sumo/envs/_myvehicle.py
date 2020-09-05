import traci

# action
STRAIGHT = 0
UTARN = 1
LEFT = 2
PARLEFT = 3
RIGHT = 4
PARRIGHT = 5
STOP = 6

# standard length for adding car
SPOS = 10.5

DIRECTION = ['s', 'T', 'l', 'L', 'r', 'R']

VEH_SIGNALS = {
    0: "VEH_SIGNAL_BLINKER_RIGHT",
    1: "VEH_SIGNAL_BLINKER_LEFT",
    2: "VEH_SIGNAL_BLINKER_EMERGENCY",
    3: "VEH_SIGNAL_BRAKELIGHT",
    4: "VEH_SIGNAL_FRONTLIGHT",
    5: "VEH_SIGNAL_FOGLIGHT",
    6: "VEH_SIGNAL_HIGHBEAM",
    7: "VEH_SIGNAL_BACKDRIVE",
    8: "VEH_SIGNAL_WIPER",
    9: "VEH_SIGNAL_DOOR_OPEN_LEFT",
    10: "VEH_SIGNAL_DOOR_OPEN_RIGHT",
    11: "VEH_SIGNAL_EMERGENCY_BLUE",
    12: "VEH_SIGNAL_EMERGENCY_RED",
    13: "VEH_SIGNAL_EMERGENCY_YELLOW",
}


class CurVehicle:
    def __init__(self, vehIDList, step_length, graph):
        self.__graph = graph
        self.__curVehInfo = []
        for vehID in vehIDList:
            tmp = {}
            tmp['ID'] = vehID
            tmp['exist'] = False
            tmp['speed'] = -1.0
            tmp['lanePos'] = -1.0
            tmp['laneID'] = ''
            tmp['edgeID'] = ''
            tmp['pos'] = (-1.0, -1.0)
            vehList = traci.vehicle.getIDList()
            if vehID in vehList:
                tmp['exist'] = True
                tmp['speed'] = traci.vehicle.getSpeed(vehID)
                tmp['lanePos'] = traci.vehicle.getLanePosition(vehID)
                tmp['laneID'] = traci.vehicle.getLaneID(vehID)
                tmp['edgeID'] = traci.vehicle.getRoadID(vehID)
                tmp['pos'] = traci.vehicle.getPosition(vehID)
            tmp['routeIndex'] = 0
            tmp['accel'] = 0.0
            tmp['routeLen'] = 0
            tmp['routeID'] = vehIDList[vehID]
            route = traci.route.getEdges(vehIDList[vehID])
            index = len(route) - 1
            tmp['start'] = route[0]
            tmp['target'] = route[index]
            self.__curVehInfo.append(tmp)
        self.isValid = False
        self.__stepLength = step_length
        self.__removeVehIDList = []
        self.__noExistList = {}
        self.updateExist()

    def getCurInfoAll(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]

    def setInValid(self):
        self.isValid = False

    def isExist(self, vehID=None, vehIndex=-1):
        if vehIndex < 0:
            if vehID is None:
                return False
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['exist']

    def setExist(self, vehID=None, vehIndex=-1):
        if vehIndex < 0:
            if vehID is not None:
                vehIndex = self.getVehIndex(vehID)
        if vehIndex > 0:
            self.__curVehInfo[vehIndex]['exist'] = True

    def updateExist(self):
        tmpList = {}
        vehList = self.__curVehInfo
        for vehInfo in vehList:
            vehID = vehInfo['ID']
            isExist = vehInfo['exist']
            if not isExist:
                start = vehInfo['start']
                tmp = {}
                if start in tmpList:
                    tmp = tmpList[start]
                    tmp['ID'].append(vehID)
                else:
                    tmp['pos'] = -1.0
                    tmp['ID'] = [vehID]
                    tmpList[start] = tmp

        for vehInfo in vehList:
            vehID = vehInfo['ID']
            isExist = vehInfo['exist']
            if isExist:
                curEdge = vehInfo['edgeID']
                if curEdge in tmpList:
                    curLanePos = vehInfo['lanePos']
                    tmp = tmpList[curEdge]
                    pos = tmp['pos']
                    if pos > 0:
                        curLanePos = min(pos, curLanePos)
                    tmp['pos'] = curLanePos
                    tmpList[curEdge] = tmp

        tmpListCopy = tmpList.copy()

        for edgeID in tmpListCopy:
            edgeInfo = tmpList[edgeID]
            pos = edgeInfo['pos']
            if pos >= SPOS:
                vehID = edgeInfo['ID'].pop(0)
                vehIndex = self.getVehIndex(vehID)
                routeID = self.__curVehInfo[vehIndex]['routeID']
                laneID = edgeID + '_0'
                length = min(traci.vehicle.getLength(vehID),
                             traci.lane.getLength(laneID))
                # register car
                traci.vehicle.add(vehID, routeID)
                # set speed mode
                traci.vehicle.setSpeedMode(vehID, 0)
                # insert car instantly
                traci.vehicle.moveTo(vehID, laneID, length)
                traci.vehicle.setSpeed(vehID, 0.0)
                self.setExist(vehID=vehID)
                # if len(edgeInfo['ID']) <= 0:
                #     tmpList.pop(edgeID)
            # update list
            # if edgeID in tmpList:
            #     tmpList[edgeID] = edgeInfo

    def getVehIndex(self, vehID):
        index = -1
        for i, vehInfo in enumerate(self.__curVehInfo):
            if vehID == vehInfo['ID']:
                index = i
        if index < 0:
            raise AttributeError("not exist the vehicle id: {}".format(vehID))
        return index

    def getVehID(self, vehIndex):
        return self.__curVehInfo[vehIndex]['ID']

    def getRemoveList(self):
        return self.__removeVehIDList

    def removeVeh(self, vehID):
        if vehID not in self.__removeVehIDList:
            self.__removeVehIDList.append(vehID)

    def getTarget(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['target']

    def getStart(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehIndo[vehIndex]['start']

    def setCurAccel(self, vehID, accel, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        self.__curVehInfo[vehIndex]['accel'] = accel

    def getCurAccel(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['accel']

    def setCurSpeed(self, vehID, speed, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        self.__curVehInfo[vehIndex]['speed'] = speed

    def getCurSpeed(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['speed']

    def setCurRouteIndex(self, vehID, routeIndex, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        self.__curVehInfo[vehIndex]['routeIndex'] = routeIndex

    def getCurRouteIndex(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['routeIndex']

    def setCurPos(self, vehID, pos, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        self.__curVehInfo[vehIndex]['pos'] = pos

    def getCurPos(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        pos = self.__curVehInfo[vehIndex]['pos']
        return pos[0], pos[1]

    def setCurLane(self, vehID, laneID, lanePos, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        self.__curVehInfo[vehIndex]['lanePos'] = lanePos
        self.__curVehInfo[vehIndex]['laneID'] = laneID
        edgeID = ''
        if laneID is not None and len(laneID) > 0:
            edgeID = traci.lane.getEdgeID(laneID)
        self.__curVehInfo[vehIndex]['edgeID'] = edgeID

    def getCurEdge(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['edgeID']

    def getCurLaneID(self, vehID, vehIndex=-1):
        lane = (self.getCurLane(vehID, vehIndex=vehIndex))
        return lane[1]

    def getCurLanePos(self, vehID, vehIndex=-1):
        lane = (self.getCurLane(vehID, vehIndex=vehIndex))
        return lane[0]

    def getCurLane(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        curLanePos = self.__curVehInfo[vehIndex]['lanePos']
        curLaneID = self.__curVehInfo[vehIndex]['laneID']
        return curLanePos, curLaneID

    def setRouteLength(self, vehID, length, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        self.__curVehInfo[vehIndex]['routeLen'] = length

    def getRouteLength(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        return self.__curVehInfo[vehIndex]['routeLen']

    def calcCurInfo(self):
        if not self.isValid:
            self.isValid = True
            for index, vehInfo in enumerate(self.__curVehInfo):
                vehID = vehInfo['ID']
                self.calcCurSpeed(vehID, vehIndex=index)
                self.calcCurLanePos(vehID, vehIndex=index)
                self.calcCurPosition(vehID, vehIndex=index)
                self.calcRouteLength(vehID, vehIndex=index)
        self.updateExist()

    def calcRouteLength(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        if self.isValid:
            return self.getRouteLength(vehID, vehIndex=vehIndex)
        route = traci.vehicle.getRoute(vehID)
        startEdgeID = route[0]
        laneID = startEdgeID + '_{}'.format(0)
        length = traci.lane.getLength(laneID)
        edge_num = len(route)
        for i in range(1, edge_num):
            viaLaneID = self.__graph.getNextInfoVia(route[i - 1],
                                                    None, None, route[i])
            length += traci.lane.getLength(viaLaneID)
            laneID = route[i] + '_{}'.format(0)
            length += traci.lane.getLength(laneID)
        self.setRouteLength(vehID, length, vehIndex=vehIndex)
        return length

    def calcCurPosition(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        if self.isValid:
            return (self.getCurPos(vehID, vehIndex=vehIndex))
        curLanePos, curLaneID = self.getCurLane(vehID)
        index = curLaneID.rfind('_') + 1
        laneIndex = int(curLaneID[index:])
        curEdgeID = self.getCurEdge(vehID)
        pos = traci.simulation.convert2D(curEdgeID, curLanePos,
                                         laneIndex)
        self.setCurPos(vehID, pos, vehIndex=vehIndex)
        return pos

    def calcCurSpeed(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        if self.isValid:
            return self.getCurSpeed(vehID, vehIndex=vehIndex)
        preSpeed = traci.vehicle.getSpeed(vehID)
        curSpeed = preSpeed + self.getCurAccel(vehID, vehIndex=vehIndex)
        self.__curVehInfo[vehIndex]['speed'] = curSpeed
        return curSpeed

    # In current step, return lane position of vehID
    def calcCurLanePos(self, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        if self.isValid:
            return self.getCurLane(vehID, vehIndex=vehIndex)

        # get infomation of vehicle
        preRouteIndex = traci.vehicle.getRouteIndex(vehID)
        if preRouteIndex < 0:
            self.setCurLane(vehID, None, -1, vehIndex=vehIndex)
            self.setCurRouteIndex(vehID, preRouteIndex, vehIndex=vehIndex)
            return -1, None

        preLaneID = traci.vehicle.getLaneID(vehID)
        roadLength = traci.lane.getLength(preLaneID)
        preLanePos = traci.vehicle.getLanePosition(vehID)
        curSpeed = self.getCurSpeed(vehID, vehIndex=vehIndex)
        curLanePosition = preLanePos + (curSpeed * self.__stepLength)

        if curLanePosition <= roadLength:
            self.setCurLane(vehID, preLaneID, curLanePosition,
                            vehIndex=vehIndex)
            self.setCurRouteIndex(vehID, preRouteIndex, vehIndex=vehIndex)
            return curLanePosition, preLaneID

        curLaneID = None
        preRouteIndex += 1
        tmpRouteIndex = preRouteIndex - 1
        tempPos = curLanePosition - roadLength
        route = traci.vehicle.getRoute(vehID)
        edge_num = len(route)

        for i in range(preRouteIndex, edge_num):
            preEdgeID = route[i - 1]
            edgeID = route[i]
            laneID = edgeID + '_{}'.format(0)
            roadLength = traci.lane.getLength(laneID)
            viaLaneID = self.__graph.getNextInfoVia(preEdgeID, None,
                                                    None, edgeID)
            nodeLength = traci.lane.getLength(viaLaneID)
            if tempPos < nodeLength:
                curLaneID = viaLaneID
                break
            tmpRouteIndex = i
            tempPos -= roadLength
            if tempPos <= roadLength:
                curLaneID = laneID
                break
            tempPos -= nodeLength

        curLanePosition = tempPos
        if curLanePosition > roadLength:
            curLanePosition = -1

        self.setCurLane(vehID, curLaneID, curLanePosition, vehIndex=vehIndex)
        self.setCurRouteIndex(vehID, tmpRouteIndex, vehIndex=vehIndex)
        return curLanePosition, curLaneID

    def changeSpeed(self, vehID, accelRate, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)
        # calculate future accel
        accel = traci.vehicle.getAccel(vehID) * accelRate
        decel = traci.vehicle.getDecel(vehID) * accelRate
        futureAccel = accel if accelRate >= 0 else decel

        # calculate future speed
        curSpeed = self.getCurSpeed(vehID, vehIndex=vehIndex)
        futureSpeed = curSpeed + futureAccel
        futureAccel = -curSpeed if futureSpeed < 0 else futureAccel
        futureSpeed = 0.0 if futureSpeed < 0 else futureSpeed

        # set future accel
        self.setCurAccel(vehID, futureAccel, vehIndex=vehIndex)

        return futureSpeed

    def couldTurn(self, futureSpeed, vehID, vehIndex=-1):
        if vehIndex < 0:
            vehIndex = self.getVehIndex(vehID)

        curEdgeID = self.getCurEdge(vehID, vehIndex=vehIndex)
        curLanePos, curLaneID = self.getCurLane(vehID, vehIndex=vehIndex)
        roadLength = traci.lane.getLength(curLaneID)

        if self.isJunction(vehID, vehIndex, curEdgeID):
            totalLen = self.getRouteLength(vehID, vehIndex=vehIndex)
            route = traci.vehicle.getRoute(vehID)
            nodeLength = traci.lane.getLength(curLaneID)
            if totalLen > nodeLength:
                return False
            tmpLaneID = route[0] + '_0'
            roadLength += traci.lane.getLength(tmpLaneID)

        futureLanePos = curLanePos + (futureSpeed * self.__stepLength)
        if futureLanePos > roadLength:
            return True
        return False

    # In the present step, whether the car is on junction or not
    def isJunction(self, vehID, vehIndex=-1, curEdgeID=None):
        if curEdgeID is None:
            if vehIndex < 0:
                vehIndex = self.getVehIndex(vehID)
            curEdgeID = self.getCurEdge(vehID, vehIndex=vehIndex)
        if self.__graph.getEdgeIndex(curEdgeID) == -1:
            return True
        return False
