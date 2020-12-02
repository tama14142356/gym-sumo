import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import copy

DEFAULT_RANDOM_STATE = np.random.RandomState(None)


def random_int_set(a, b, num, exclude=[], random_state=DEFAULT_RANDOM_STATE):
    """num random integer without duplication in the range from a to b

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer): the number of random integer
        exclude (list, optional): list of integer which exclude. Defaults to [].
        random_state (numpy.random.mtrand.RandomState,
            optional): numpy random_state. Defaults to np.random.RandomState(None).

    Returns:
        list: random integer list without duplication
              but, if exclude_num >= b - a + 1 - num,
                   the number of element is less than num
    """
    ns = random_int(a, b, num, exclude, random_state)
    ns_duplicate = list(set(ns))
    return ns_duplicate


def random_int(a, b, num, exclude=[], random_state=DEFAULT_RANDOM_STATE):
    """num random integer without duplication in the range from a to b

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer): the number of random integer
        exclude (list, optional): list of integer which exclude. Defaults to [].
        random_state (numpy.random.mtrand.RandomState,
            optional): numpy random_state. Defaults to np.random.RandomState(None).

    Returns:
        list: random integer list without duplication whose has num element
              but, if exclude_num >= b - a + 1 - num, some element is duplication
    """
    ns = []
    tmp_exclude_set = set(copy.deepcopy(exclude))
    limit = b - a + 1 - num
    exclude_num = len(tmp_exclude_set)
    if exclude_num >= limit:
        exclude_list = []
    else:
        exclude_list = list(tmp_exclude_set)
    num_element = 0
    while num_element < num:
        n = random_state.randint(a, b + 1)
        if (n not in exclude_list) or (limit <= num_element - num + 1):
            ns.append(n)
            num_element += 1
            exclude_list.append(n)

    return ns


def _add_exclude_list(source_list, add_exclude_list, index):
    """add all exclude number which is chosen from add_exclude_list to source_list

    Args:
        source_list (list): list of base
        add_exclude_list (list): list of exclude
        index (integer): index of exclude list

    Returns:
        list: exclude list
    """
    exclude_list = copy.deepcopy(source_list)
    if len(add_exclude_list) <= 0:
        return exclude_list
    tmp_add_exclude = np.array(copy.deepcopy(add_exclude_list))
    add_exclude = tmp_add_exclude[:, index].tolist()
    exclude_list.append(add_exclude)
    exclude_flat_list = flatten_list(exclude_list)
    exclude_set_list = list(set(exclude_flat_list))
    return exclude_set_list


def random_tuple(
    a, b, num=2, start_exclude=[], exclude=[], random_state=DEFAULT_RANDOM_STATE
):
    """create random tuple which has num element
       but, first element is chosen from
            integers which isn't inlcluded in start_exclude
            until you can't choose integer

    Args:
        a (integer): start integer of closed section
        b (integer): end interger of closed section
        num (integer, optional): the number of element of tuple. Defaults to 2.
        start_exclude (list, optional): list of integer. Defaults to [].
        exclude (list, optional): list of integer which exclude. Defaults to [].
        random_state (numpy.random.mtrand.RandomState,
            optional): numpy random_state. Defaults to np.random.RandomState(None).

    Returns:
        tuple: tuple with "num" random numbers in the elements
    """
    index_tuple = ()
    ns = []
    if num >= 1:
        start_exclude_list = _add_exclude_list(start_exclude, exclude, 0)
        start = random_int(a, b, 1, start_exclude_list, random_state)
        end_exclude_list = _add_exclude_list(ns, exclude, 1)
        end = random_int(a, b, num - 1, end_exclude_list, random_state)
        ns = flatten_list([start, end])
    for nstmp in ns:
        index_tuple = index_tuple + (nstmp,)
    return index_tuple


def is_new_tuple(target_list, a, b):
    """function which judge whether new tuple

    Args:
        target_list (dict): dictionary of tuple :
                           key-> first element of tuple,
                           value-> list of second element of tuple
        a (integer): first element of tuple
        b (integer): second element of tuple

    Returns:
        bool: whether new tuple or not
    """
    if a not in target_list:
        target_list[a] = []
        target_list[a].append(b)
        return True
    elif b not in target_list[a]:
        target_list[a].append(b)
        return True
    return False


def _flatten_list(target_list):
    """flatten the list

    Args:
        target_list (list): target list to flatten

    Yields:
        any type(scalar): scalar element of target list
    """
    for element in target_list:
        if isinstance(element, list):
            yield from flatten_list(element)
        else:
            yield element


def flatten_list(target_list):
    """flatten the target list

    Args:
        target_list (list): target list to flatten

    Returns:
        list: list which flatten the target list
    """
    return list(_flatten_list(target_list))


def _get_vector(start_pos, target_pos, is_complex=False):
    """create vector from start_pos to target_pos

    Args:
        start_pos (numpy.ndarray or list or tuple): start vector
        target_pos (numpy.ndarray or list or tuple): end vector
        is_complex (bool, optional): type of vector is wether complex is or not.
                                     Defaults to False.

    Returns:
        numpy.ndarray or complex: vector from start_pos to target_pos
    """
    start = np.array(start_pos, dtype=np.float)
    target = np.array(target_pos, dtype=np.float)
    vector = target - start
    if is_complex:
        return complex(vector[0], vector[1])
    return vector


def get_base_vector(start_pos, target_pos, norm=1.0, is_complex=False):
    """create vector from start_pos to target_pos whose norm is norm

    Args:
        start_pos (numpy.ndarray or list or tuple): start vector
        target_pos (numpy.ndarray or list or tuple): end vector
        norm (float, optional): vector's norm. Defaults to 1.0.
        is_complex (bool, optional): type of vector is wether complex is or not.
                                     Defaults to False.

    Returns:
        numpy.ndarray or complex: vector from start_pos to target_pos whose norm is norm
    """
    vector = _get_vector(start_pos, target_pos, is_complex)
    vector_norm = abs(vector) if is_complex else np.linalg.norm(vector, ord=2)
    base_vector = vector / vector_norm
    ans_vector = base_vector * np.sqrt(norm)
    return ans_vector


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
    base_angle = abs_base_angle if angle >= 0.0 else 360.0 - abs_base_angle
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
    normal_angle = get_normal_angle(angle, [0, 1], -1)
    base_radian = np.deg2rad(normal_angle)
    v_x = vector_length * np.cos(base_radian)
    v_y = vector_length * np.sin(base_radian)
    return v_x, v_y


def get_normal_angle(angle, pre_std, pre_std_direct, new_std=[1, 0], new_std_direct=1):
    """get angle after change starndar direction, direction of declinate

    Args:
        angle (float): target angle to change standard
        pre_std (list of tuple or numpy.ndarray): previous standard direction
        pre_std_direct (int): previous direction of declinate
        new_std (list or tuple or numpy.ndarray, optional): new standard direction.
                                                            Defaults to [1, 0].
        new_std_direct (int, optional): new standard direction of declinate.
                                        Defaults to 1.
                                        1 -> counterclockwise
                                        other -> clockwise
    Returns:
        float: angle after change standard in [0, 360)
    """
    base_angle = get_base_angle(angle)
    new_std_complex = get_base_vector([0, 0], new_std, is_complex=True)
    pre_std_complex = get_base_vector([0, 0], pre_std, is_complex=True)
    if pre_std_complex == new_std_complex and pre_std_direct == new_std_direct:
        return base_angle
    if pre_std_direct != new_std_direct:
        base_angle = get_base_angle(-base_angle)
    new_std_angle = np.angle(new_std_complex, deg=True)
    pre_std_angle = np.angle(pre_std_complex, deg=True)
    angle_new_to_pre = pre_std_angle - new_std_angle
    if new_std_direct == 1:
        return get_base_angle(angle_new_to_pre + base_angle)
    return get_base_angle(360.0 - angle_new_to_pre + base_angle)


def get_degree(start_pos, target_pos):
    """get declinate of vector which is start_pos to target_pos

    Args:
        start_pos (numpy.ndarray or list or tuple): starting point of that vector
        target_pos (numpy.ndarray or list or tuple): ending point of that vector

    Returns:
        float: declinate of vector

    Conditions:
        0 degree line: the positive y-axis direction
        direction of declinate: clockwise
    """
    complex_number = _get_vector(start_pos, target_pos, is_complex=True)
    declinate = np.angle(complex_number, deg=True)
    negative_base_declinate = get_negative_base_angle(declinate)
    vector_degree = abs(negative_base_declinate) + 90.0
    return vector_degree


def get_rectangle_positions(start_pos, target_pos, width):
    """get rectangle position

    Args:
        start_pos (list or tuple or numpy.ndarray): central position of rectangle edge
        target_pos (list or tuple or numpy.ndarray): central position of opposite side
                                                   to the above
        width (float): length of tha above edge

    Returns:
        list: positions of rectangular
    """
    complex_number = _get_vector(start_pos, target_pos, is_complex=True)
    length = abs(complex_number)
    width_len = width / (2.0 * length)
    theta = np.deg2rad(90)
    rotate_vertical = np.cos(theta) + 1j * np.sin(theta)
    vertical_vector = rotate_vertical * complex_number * width_len
    rectangle = []
    vertical_vector_ndarray = np.array([vertical_vector.real, vertical_vector.imag])
    pos = start_pos + vertical_vector_ndarray
    rectangle.append(pos)
    pos = start_pos - vertical_vector_ndarray
    rectangle.append(pos)
    pos = target_pos - vertical_vector_ndarray
    rectangle.append(pos)
    pos = target_pos + vertical_vector_ndarray
    rectangle.append(pos)
    return rectangle


def in_rect(rect, target):
    """whether target position is in rect

    Args:
        rect (list or numpy.ndarray): rectangular positions
        target (list or tuple or numpy.ndarray): target position

    Returns:
        bool: whether target position is in rect
    """
    vector_a = np.array(rect[0])
    vector_b = np.array(rect[1])
    vector_c = np.array(rect[2])
    vector_d = np.array(rect[3])
    vector_e = np.array(target)

    vector_ab = vector_b - vector_a
    vector_ae = vector_e - vector_a
    vector_bc = vector_c - vector_b
    vector_be = vector_e - vector_b
    vector_cd = vector_d - vector_c
    vector_ce = vector_e - vector_c
    vector_da = vector_a - vector_d
    vector_de = vector_e - vector_d

    vector_cross_ab_ae = np.cross(vector_ab, vector_ae)
    vector_cross_bc_be = np.cross(vector_bc, vector_be)
    vector_cross_cd_ce = np.cross(vector_cd, vector_ce)
    vector_cross_da_de = np.cross(vector_da, vector_de)

    return (
        vector_cross_ab_ae < 0
        and vector_cross_bc_be < 0
        and vector_cross_cd_ce < 0
        and vector_cross_da_de < 0
    )


def in_many_shape(rect, target):
    point = Point(target)
    polygon = Polygon(rect)
    return polygon.contains(point)
