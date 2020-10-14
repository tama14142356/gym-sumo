import numpy as np

DEFAULT_RANDOM_STATE = np.random.RandomState(None)


def random_int(a, b, num, exclude=[], random_state=DEFAULT_RANDOM_STATE):
    """num random integer without duplication in the range from a to b

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer): the number of random integer
        exclude (list, optional): list of integer which exclude. Defaults to [].
        random_state (numpy object, optional): numpy random_state.
                                               Defaults to np.random.RandomState(None).

    Returns:
        ns (list): random integer list
    """
    ns = []
    while len(ns) < num:
        n = random_state.randint(a, b + 1)
        if n not in ns and n not in exclude:
            ns.append(n)
    return ns


def randomTuple(a, b, num, targetList, random_state=DEFAULT_RANDOM_STATE):
    """create random tuple which has num element
       but, first element is chosen from
            integers which isn't inlcluded in targetList
            until you can't choose integer

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer): the number of element of tuple
        targetList (list): list of integer
        random_state (numpy object, optional): numpy random_state.
                                               Defaults to np.random.RandomState(None).

    Returns:
        IndexTuple (tuple): num element random tuple
    """
    listNum = len(targetList)
    limit = b - a + 1
    IndexTuple = ()
    if listNum >= limit:
        ns = random_int(a, b, num, random_state=random_state)
    else:
        ns = random_int(a, b, 1, targetList, random_state=random_state)
        tmp = random_int(a, b, num - 1, ns, random_state=random_state)
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


def _flatten_list(targetList):
    """flatten the list

    Args:
        targetList (list): target list to flatten

    Yields:
        any type(scalar): scalar element of target list
    """
    for element in targetList:
        if isinstance(element, list):
            yield from flatten_list(element)
        else:
            yield element


def flatten_list(targetList):
    """flatten the target list

    Args:
        targetList (list): target list to flatten

    Returns:
        list: list which flatten the target list
    """
    return list(_flatten_list(targetList))


def get_base_angle(angle):
    """convert value of the angle to the angle whose value is in [0, 360)

    Args:
        angle (float): row angle value

    Returns:
        float: base angle whose value is in [0, 360)
    """
    abs_angle = abs(angle)
    n = int(abs_angle // 360)
    abs_base_angle = abs_angle - float(360 * n)
    base_angle = (abs_base_angle if angle >= 0.0 else abs_base_angle + 360.0)
    return base_angle


def get_negative_base_angle(angle):
    """convert the value of angle to the angle whose value is in [-360, 0)

    Args:
        angle (float): row angle value

    Returns:
        float: negative base angle whose value is in [-360, 0)
    """
    base_angle = get_base_angle(angle)
    negative_base_angle = base_angle - 360.0
    return negative_base_angle


def vector_decomposition(vector_length, angle):
    """decompose vector to x, y

    Args:
        vector_length (float): vector length
        angle (float): row angle value of vector

    Returns:
        float, float: x coordinate, y coordinate which the vector has

    Conditions:
        0 degree line: the positive y-axis direction
        direction of declinate: clockwise
    """
    base_angle = get_base_angle(angle)
    base_radian = np.deg2rad(base_angle)
    v_y = vector_length * np.cos(base_radian)
    v_x = vector_length * np.sin(base_radian)
    return v_x, v_y


def get_degree(start_pos, target_pos):
    """get declinate of vector which is start_pos to target_pos

    Args:
        start_pos (numpy array or list or tuple): starting point of that vector
        target_pos (numpy array or list or tuple): ending point of that vector

    Returns:
        float: declinate of vector

    Conditions:
        0 degree line: the positive y-axis direction
        direction of declinate: clockwise
    """
    start_pos = np.array(start_pos)
    target_pos = np.array(target_pos)
    vector = target_pos - start_pos
    complex_number = complex(vector[0], vector[1])
    declinate = np.angle(complex_number, deg=True)
    negative_base_declinate = get_negative_base_angle(declinate)
    vector_degree = abs(negative_base_declinate) + 90.0
    return vector_degree


# def get_line_equation(start_pos, target_pos):