import random


def random_int(a, b, num):
    ns = []
    while len(ns) < num:
        n = random.randrange(a, b)
        if n not in ns:
            ns.append(n)
    return ns


def randomTuple(a, b, num, targetList):
    fromIndex = -1
    toIndex = -1
    while True:
        ns = random_int(a, b, num)
        fromIndex = ns[0]
        toIndex = ns[1]
        if isNewTuple(targetList, fromIndex, toIndex):
            break
    return fromIndex, toIndex


def isNewTuple(targetList, a, b):
    if a not in targetList:
        targetList[a] = []
        targetList[a].append(b)
        return True
    elif b not in targetList[a]:
        targetList[a].append(b)
        return True
    return False
