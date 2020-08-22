import traci


class CurVehicle:
    def __init__(self, vehIDList, step_length, graph):
        self.__graph = graph
        self.curVehInfo = []
        for i, vehID in enumerate(vehIDList):
            tmp = {}
            tmp['ID'] = vehID
            tmp['speed'] = traci.vehicle.getSpeed(vehID)
            tmp['lanePos'] = traci.vehicle.getLanePosition(vehID)
            tmp['laneID'] = traci.vehicle.getLaneID(vehID)
            tmp['edgeID'] = traci.vehicle.getRoadID(vehID)
            tmp['accel'] = 0.0
            tmp['pos'] = (traci.vehicle.getPosition(vehID))
            self.curVehInfo.append(tmp)
        self.isValid = False
        self.__stepLength = step_length
    
    def setInValid(self):
        self.isValid = False

    def getVehIndex(self, vehID):
        index = -1
        for i, vehInfo in enumerate(self.curVehInfo):
            if vehID == vehInfo['ID']:
                index = i
        return index
    
    def getVehID(self, vehIndex):
        return self.curVehInfo[vehIndex]['ID']

    def setAccel(self, vehID, accel):
        index = self.getVehIndex(vehID)
        self.curVehInfo[index]['accel'] = accel
    
    def getAccel(self, vehID):
        index = self.getVehIndex(vehID)
        return self.curVehInfo[index]['accel']

    def setCurSpeed(self, vehID, speed):
        index = self.getVehIndex(vehID)
        self.curVehInfo[index]['speed'] = speed
    
    def getCurSpeed(self, vehID):
        index = self.getVehIndex(vehID)
        if not self.isValid:
            self.calcCurSpeed(vehID)
        return self.curVehInfo[index]['speed']
    
    def getPosition(self, vehID):
        index = self.getVehIndex(vehID)
        pos = self.curVehInfo[index]['pos']
        return pos[0], pos[1]
    
    def getCurEdge(self, vehID):
        index = self.getVehIndex(vehID)
        return self.curVehInfo[index]['edgeID']

    def getCurLane(self, vehID):
        index = self.getVehIndex(vehID)
        curLanePos = self.curVehInfo[index]['lanePos']
        curLaneID = self.curVehInfo[index]['laneID']
        return curLanePos, curLaneID

    def setCurLane(self, vehID, laneID, lanePos):
        index = self.getVehIndex(vehID)
        self.curVehInfo[index]['lanePos'] = lanePos
        self.curVehInfo[index]['laneID'] = laneID
        edgeID = traci.lane.getEdgeID(laneID)
        self.curVehInfo[index]['edgeID'] = edgeID
    
    def calcCurInfo(self):
        if not self.isValid:
            for vehInfo in self.curVehInfo:
                vehID = vehInfo['ID']
                self.calcCurSpeed(vehID)
                self.calcCurLanePos(vehID)
                self.calcCurPosition(vehID)
                self.isValid = True

    def calcCurPosition(self, vehID):
        if self.isValid:
            return (self.getPosition(vehID))
        curLanePos, curLaneID = self.getCurLane(vehID)
        index = curLaneID.rfind('_') + 1
        laneIndex = int(curLaneID[index:])
        index = self.getVehIndex(vehID)
        curEdgeID = self.getCurEdge(vehID)
        pos = traci.simulation.convert2D(curEdgeID, curLanePos,
                                         laneIndex)
        self.curVehInfo[index]['pos'] = pos
        return pos
    
    def calcCurSpeed(self, vehID):
        if self.isValid:
            return self.getCurSpeed(vehID)
        index = self.getVehIndex(vehID)
        preSpeed = traci.vehicle.getSpeed(vehID)
        curSpeed = preSpeed + self.curVehInfo[index]['accel']
        self.setCurSpeed(vehID, curSpeed)
        return curSpeed

    # In current step, return lane position of vehID
    def calcCurLanePos(self, vehID):
        if self.isValid:
            return self.getCurLane(vehID)
        
        # get infomation of vehicle
        preLaneID = traci.vehicle.getLaneID(vehID)
        roadLength = traci.lane.getLength(preLaneID)
        preLanePos = traci.vehicle.getLanePosition(vehID)
        curSpeed = self.getCurSpeed(vehID)
        curLanePosition = preLanePos + (curSpeed * self.__stepLength)
        
        if curLanePosition <= roadLength:
            self.setCurLane(vehID, preLaneID, curLanePosition)
            return curLanePosition, preLaneID
        
        curLaneID = None
        preRouteIndex = traci.vehicle.getRouteIndex(vehID) + 1
        tempPos = curLanePosition - roadLength
        route = traci.vehicle.getRoute(vehID)
        edge_num = len(route)
        
        if preRouteIndex >= edge_num:
            self.setCurLane(vehID, None, -1)
            return -1, None
        
        for i in range(preRouteIndex, edge_num):
            preEdgeID = route[i - 1]
            edgeID = route[i]
            laneID = edgeID + '_{}'.format(0)
            roadLength = traci.lane.getLength(laneID)
            viaLaneID = self.__graph.getNextInfoVia(preEdgeID, '0',
                                                    'no', edgeID)
            nodeLength = traci.lane.getLength(viaLaneID)
            if tempPos < nodeLength:
                curLaneID = viaLaneID
                break
            tempPos -= roadLength
            if tempPos <= roadLength:
                curLaneID = laneID
                break
            tempPos -= nodeLength

        curLanePosition = tempPos
        if curLanePosition > roadLength:
            curLanePosition = -1
        
        self.setCurLane(vehID, curLaneID, curLanePosition)
        return curLanePosition, curLaneID

    def changeSpeed(self, vehID, accelRate):
        # calculate future accel
        accel = traci.vehicle.getAccel(vehID) * accelRate
        decel = traci.vehicle.getDecel(vehID) * accelRate
        futureAccel = accel if accelRate >= 0 else decel
        
        # calculate future speed
        curSpeed = self.getCurSpeed(vehID)
        futureSpeed = curSpeed + futureAccel
        futureAccel = -curSpeed if futureSpeed < 0 else futureAccel
        futureSpeed = 0.0 if futureSpeed < 0 else futureSpeed
        
        # set future accel
        self.setAccel(vehID, futureAccel)
        
        return futureSpeed

    def couldTurn(self, futureSpeed, vehID):
        if self.isJunction(vehID):
            return False
        curLanePos, curLaneID = self.getCurLane(vehID)
        futureLanePos = curLanePos + (futureSpeed * self.__stepLength)
        roadLength = traci.lane.getLength(curLaneID)
        if futureLanePos > roadLength:
            return True
        return False
    
    # In the present step, whether the car is on junction or not
    def isJunction(self, vehID):
        ID = self.getCurEdge(vehID)
        if self.__graph.getEdgeIndex(ID) == -1:
            return True
        return False
