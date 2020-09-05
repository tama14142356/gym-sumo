import random


def random_int(a, b, num, exclude=[]):
    """num random integer without duplication in the range from a to b

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer): the number of random integer
        exclude (list): list of integer which exclude

    Returns:
        ns (list): random integer list
    """
    ns = []
    while len(ns) < num:
        n = random.randrange(a, b)
        if n not in ns and n not in exclude:
            ns.append(n)
    return ns


def randomTuple(a, b, num, targetList):
    """create random tuple which has num element
       but, first element is chosen from
            integers which isn't inlcluded in targetList
            until you can't choose integer

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer): the number of element of tuple
        targetList (list): list of integer

    Returns:
        IndexTuple (tuple): num element random tuple
    """
    listNum = len(targetList)
    limit = b - a + 1
    IndexTuple = ()
    if listNum > limit:
        ns = random_int(a, b, num)
    else:
        ns = random_int(a, b, 1, targetList)
        tmp = random_int(a, b, num - 1, ns)
        ns[len(ns):len(tmp)] = tmp
    for nstmp in ns:
        IndexTuple = IndexTuple + (nstmp, )
    return IndexTuple


def isNewTuple(targetList, a, b):
    """function which judge whether new tuple

    Args:
        targetList (dict): dictionary of tuple :
                           key-> first element of tuple,
                           value-> list of second element of tuple
        a (integer): first element of tuple
        b (integer): second element of tuple

    Returns:
        bool: whether new tuple or not
    """
    if a not in targetList:
        targetList[a] = []
        targetList[a].append(b)
        return True
    elif b not in targetList[a]:
        targetList[a].append(b)
        return True
    return False
