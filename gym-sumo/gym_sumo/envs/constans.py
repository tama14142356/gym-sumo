AREA = ["nishiwaseda", "waseda_university"]
# max length for route length
MAX_LENGTH = 1000.0
# road number of route
MAX_ROAD_NUM = 20
MIN_ROAD_NUM = 2
# standard vehicle length(m)
VEH_LEN = 5.0
# vehicle direction
STRAIGHT = 0
UTURN = 1
LEFT = 2
PAR_LEFT = 3
RIGHT = 4
PAR_RIGHT = 5
DIRECTION = ["s", "T", "l", "L", "r", "R"]

# accel
POS_LARGE = 6
POS_SMALL = 7
NEG_LARGE = 8
NEG_SMALL = 9
ACCEL = [1.0, 0.2, -1.0, -0.2]

# flag
DIRECT_FLAG = max(STRAIGHT, UTURN, LEFT, PAR_LEFT, RIGHT, PAR_RIGHT)


DIRECTION_TEXT = ["STRAIGHT", "UTURN", "LEFT", "PAR_LEFT", "RIGHT", "PAR_RIGHT"]

ACCEL_TEXT = ["LARGE ACCEL", "SMALL ACCEL", "LARGE DECEL", "SMALL DECEL"]

DEFAULT_KWARGS_FIXED = {
    "isgraph": False,
    "area": 0,
    "carnum": 100,
    "mode": "gui",
    "step_length": 0.01,
    "simulation_end": 200,
    "seed": None,
    "label": "default",
    "debug_view": False,
    "is_random_route": True,
    "max_length": MAX_LENGTH,
    "is_abs_length": False,
    "is_length": False,
    "is_road_num": False,
    "is_end": True,
    "is_start": False,
    "road_freq": 0,
    "road_ratio": 1.0,
    "is_auto": False,
    "route_length": [1000.0, 2000.0],
    "data_num": 1000,
    "max_data": 100000,
    "fixed_veh_index": 0,
}

DEFAULT_KWARGS_LIGHT = {
    "isgraph": False,
    "area": 0,
    "carnum": 100,
    "mode": "gui",
    "step_length": 0.01,
    "simulation_end": 200,
    "seed": None,
    "label": "default",
    "debug_view": False,
    "is_random_route": True,
    "max_length": MAX_LENGTH,
    "is_abs_length": False,
    "is_length": False,
    "is_road_num": False,
    "is_end": True,
    "is_start": False,
    "road_freq": 0,
    "road_ratio": 1.0,
    "is_auto": False,
}

DEFAULT_KWARGS_BASE = {
    "isgraph": False,
    "area": 0,
    "carnum": 100,
    "mode": "gui",
    "step_length": 0.01,
    "simulation_end": 200,
    "seed": None,
    "label": "default",
    "debug_view": False,
    "is_random_route": True,
    "max_length": MAX_LENGTH,
    "is_abs_length": False,
    "is_length": False,
    "is_road_num": False,
    "is_end": True,
    "is_start": False,
}

INFO = {
    "is_take": False,
    "is_removed": False,
    "cur_step": -1.0,
    "cur_sm_step": -1.0,
    "is_arrived": False,
    "needs_reset": False,
    "goal": {},
    "cur_lane": "",
    "cur_lane_pos": -1.0,
    "lane_len": -1.0,
    "speed": -1.0,
    "pos": (-1.0, -1.0),
    "driving_len": -1.0,
    "road_num": MIN_ROAD_NUM,
}

# vehicle signal number
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
