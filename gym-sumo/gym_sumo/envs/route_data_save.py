import numpy as np
import pathlib
import pickle
import pprint

from gym import error
from gym_sumo.envs import constans as gc
from gym_sumo.envs.sumo_graph import SumoGraph

from tqdm import tqdm

MESSAGE_HINT = (
    "(HINT: you can install sumo or set the path for sumo library by reading README.md)"
)
try:
    import traci
except ImportError as e:
    raise error.DependencyNotInstalled("{}.".format(e) + MESSAGE_HINT)


AREA = gc.AREA
CONFIG_RELATIVE_PATH = "sumo_configs/"
DEFAULT_FILE_NAME = "route_data"
PICKLE_EXTEND = ".binaryfile"
TEXT_EXTEND = ".txt"
SUMO_NET_EXTEND = ".net.xml"
SUMO_CONFIG_EXTEND = ".sumocfg"

CWD_PATH = pathlib.os.path.dirname(__file__)
CONFIG_DEFAULT_AREA = CONFIG_RELATIVE_PATH + AREA[0]
CONFIG_DEFAULT_ABS_PATH = pathlib.Path(
    pathlib.os.path.join(CWD_PATH, CONFIG_DEFAULT_AREA)
).resolve()
ROUTE_DEFAULT_DIR = CONFIG_DEFAULT_ABS_PATH / "route"

MAX_DEFAULT_DATA = 100000
MIN_DEFAULT_LENGTH = 500.0
MAX_DEFAULT_LENGTH = 1000.0
LOAD_DEFAULT_DIR = (
    ROUTE_DEFAULT_DIR
    / "{}_data".format(MAX_DEFAULT_DATA)
    / "end-end"
    / "{}-{}_length".format(MIN_DEFAULT_LENGTH, MAX_DEFAULT_LENGTH)
)


def get_dir_path(max_data, def_len, length):
    base_dir = ROUTE_DEFAULT_DIR
    data_dir = format_data_num_dir(max_data)
    def_dir = format_definition_length_dir(def_len)
    len_dir = format_length_dir(length)
    return str(base_dir / data_dir / def_dir / len_dir)


def get_def_len(is_end, is_start):
    return [is_end, is_start]


def format_length_dir(length):
    min_length, max_length = length
    return "{}-{}_length".format(min_length, max_length)


def format_data_num_dir(data_num):
    return "{}_data".format(data_num)


def format_definition_length_dir(def_len):
    is_end, is_start = def_len
    start = "start" if is_start else "end"
    end = "end" if is_end else "start"
    return start + "-" + end


def route_data_save(
    route_data, file_path=str(LOAD_DEFAULT_DIR), file_name=DEFAULT_FILE_NAME
):
    tmp_path = pathlib.Path(file_path)
    route_data_file_name = file_name + PICKLE_EXTEND
    route_data_text_file_name = file_name + TEXT_EXTEND
    output_path = tmp_path / route_data_file_name
    with open(output_path, "wb") as f:
        pickle.dump(route_data, f)
    output_path = tmp_path / route_data_text_file_name
    with open(output_path, "w") as f:
        pprint.pprint(route_data, stream=f, compact=True)


def route_data_load(file_path=str(LOAD_DEFAULT_DIR), in_length=False, is_all=False):
    tmp_path = pathlib.Path(file_path)
    route_data_file_name = DEFAULT_FILE_NAME + PICKLE_EXTEND
    output_path = tmp_path / route_data_file_name
    with open(output_path, "rb") as f:
        data = pickle.load(f)
    if is_all:
        return data
    if in_length:
        start_end_list = [
            {"length": d.get("abs_length"), "start_end": [d.get("start"), d.get("end")]}
            for d in data
            if d.get("abs_length") and d.get("start") and d.get("end")
        ]
        return start_end_list
    return get_start_end_list(data)


def get_start_end_list(data):
    start_end_list = [
        [d.get("start"), d.get("end")] for d in data if d.get("start") and d.get("end")
    ]
    return start_end_list


def init_simulator(sumocfg, mode="cui", routing_alg="dijkstra"):
    sumo_command = mode
    if mode == "gui":
        sumo_command = "sumo-gui"
    elif mode == "cui":
        sumo_command = "sumo"
    else:
        raise AttributeError("not supported mode!!")
    sumo_cmd = [sumo_command, "-c", sumocfg, "--routing-algorithm", routing_alg]
    traci.start(sumo_cmd, numRetries=100, label="route")
    return traci.getConnection("route")


def get_pos_edgeID(traci_conn, network, edgeID, lane_index=0):
    laneID = network.get_laneID(edgeID, lane_index)
    lane_pos_list = traci_conn.lane.getShape(laneID)
    shape_num = len(lane_pos_list)
    end_pos = lane_pos_list[shape_num - 1]
    start_pos = lane_pos_list[0]
    return start_pos, end_pos


def convert_pos(start_pos, end_pos, def_len):
    is_end, is_start = def_len
    start = start_pos if is_start else end_pos
    end = end_pos if is_end else start_pos
    return start, end


def calc_distance(from_pos, to_pos):
    return float(
        np.sqrt((to_pos[0] - from_pos[0]) ** 2 + (to_pos[1] - from_pos[1]) ** 2)
    )


def main(area, file_name, length, max_data, is_end=True, is_start=False):
    sumo_config_abs_path = CONFIG_DEFAULT_ABS_PATH
    sumocfg_file_name = file_name + SUMO_CONFIG_EXTEND
    net_file_name = file_name + SUMO_NET_EXTEND
    sumocfg_path = sumo_config_abs_path / sumocfg_file_name
    traci_conn = init_simulator(str(sumocfg_path))
    net_path = sumo_config_abs_path / net_file_name
    network = SumoGraph(str(net_path))

    def_len = get_def_len(is_end, is_start)
    route_dir_path = get_dir_path(max_data, def_len, length)
    pathlib.os.makedirs(route_dir_path, exist_ok=True)

    edgeID_list = network.get_all_edgeID_list(False)
    edge_info_list = {}
    for edgeID in edgeID_list:
        start_pos, end_pos = get_pos_edgeID(traci_conn, network, edgeID)
        start, end = convert_pos(start_pos, end_pos, def_len)
        edge_info = {"start": list(start), "end": list(end)}
        edge_info_list[edgeID] = edge_info

    route_data_list = []
    route_data_num = 0
    min_length, max_length = length

    progress_bar = tqdm(total=max_data)

    for i, start_edgeID in enumerate(edgeID_list):
        start_pos = edge_info_list[start_edgeID]["start"]
        for j, target_edgeID in enumerate(edgeID_list):
            end_pos = edge_info_list[target_edgeID]["end"]
            if i == j:
                continue
            cur_length = calc_distance(start_pos, end_pos)
            if cur_length >= min_length and cur_length <= max_length:
                route = traci_conn.simulation.findRoute(start_edgeID, target_edgeID)
                edge_num = len(route.edges)
                if edge_num > 0:
                    route_info = {
                        "start": start_edgeID,
                        "end": target_edgeID,
                        "abs_length": cur_length,
                        "route_length": route.length,
                        "cost": route.cost,
                        "travel_time": route.travelTime,
                        "edge_list": route.edges,
                        "edge_num": edge_num,
                    }
                    route_data_num += 1
                    route_data_list.append(route_info)
                    progress_bar.update(1)
                    if route_data_num >= max_data:
                        break
        if route_data_num >= max_data:
            break
    route_data_list_sorted = sorted(route_data_list, key=lambda x: x["abs_length"])
    print("saving...")
    route_data_save(route_data_list_sorted, route_dir_path)
    try:
        traci_conn.close()
    except traci_conn.exceptions.FatalTraCIError as ci:
        print(ci)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("--area", type=int, default=0)
    parser.add_argument("--file-name", type=str, default="osm")
    parser.add_argument("--min-length", type=float, default=MIN_DEFAULT_LENGTH)
    parser.add_argument("--max-length", type=float, default=MAX_DEFAULT_LENGTH)
    parser.add_argument("--only-more", action="store_true", default=False)
    parser.add_argument("--only-less", action="store_true", default=False)
    parser.add_argument("--no-only-end", action="store_true", default=False)
    parser.add_argument("--only-start", action="store_true", default=False)
    parser.add_argument("--max-data", type=int, default=MAX_DEFAULT_DATA)

    args = parser.parse_args()

    length = [args.min_length, args.max_length]

    if args.only_more:
        length[1] = np.inf

    if args.only_less:
        length[0] = 0.0

    main(
        args.area,
        args.file_name,
        length,
        args.max_data,
        (not args.no_only_end),
        args.only_start,
    )
