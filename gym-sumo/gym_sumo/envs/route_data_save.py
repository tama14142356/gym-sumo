import numpy as np
import pathlib
import pickle

from gym import error
import gym_sumo.envs.sumo_base_env as base
from gym_sumo.envs.sumo_graph import SumoGraph

MESSAGE_HINT = (
    "(HINT: you can install sumo or set the path for sumo library by reading README.md)"
)
try:
    import traci
except ImportError as e:
    raise error.DependencyNotInstalled("{}.".format(e) + MESSAGE_HINT)


AREA = base.AREA
CONFIG_RELATIVE_PATH = "sumo_configs/"
OUTPUT_FILE_NAME = "route_data.binaryfile"
TEXT_FILE_NAME = "route_data.txt"

CWD_PATH = pathlib.os.path.dirname(__file__)
CONFIG_DEFAULT_AREA = CONFIG_RELATIVE_PATH + AREA[0]
CONFIG_DEFAULT_ABS_PATH = pathlib.Path(
    pathlib.os.path.join(CWD_PATH, CONFIG_DEFAULT_AREA)
).resolve()
ROUTE_DEFAULT_DIR = CONFIG_DEFAULT_ABS_PATH / "route"


def route_data_save(start_end_list, file_path=str(ROUTE_DEFAULT_DIR)):
    tmp_path = pathlib.Path(file_path)
    route_data_file_name = OUTPUT_FILE_NAME
    route_data_text_file_name = TEXT_FILE_NAME
    output_path = tmp_path / route_data_file_name
    with open(output_path, "wb") as f:
        pickle.dump(start_end_list, f)
    output_path = tmp_path / route_data_text_file_name
    with open(output_path, "w") as f:
        print(start_end_list, file=f)


def route_data_load(file_path=str(ROUTE_DEFAULT_DIR)):
    tmp_path = pathlib.Path(file_path)
    route_data_file_name = OUTPUT_FILE_NAME
    output_path = tmp_path / route_data_file_name
    with open(output_path, "rb") as f:
        start_end_list = pickle.load(f)
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
    return end_pos


def calc_distance(from_pos, to_pos):
    return float(
        np.sqrt((to_pos[0] - from_pos[0]) ** 2 + (to_pos[1] - from_pos[1]) ** 2)
    )


def main(area, net_file_name, sumocfg_file_name, length, max_data):
    sumo_config_abs_path = CONFIG_DEFAULT_ABS_PATH
    sumocfg_path = sumo_config_abs_path / sumocfg_file_name
    traci_conn = init_simulator(str(sumocfg_path))
    net_path = sumo_config_abs_path / net_file_name
    network = SumoGraph(str(net_path))

    route_dir_path = ROUTE_DEFAULT_DIR
    pathlib.os.makedirs(str(route_dir_path), exist_ok=True)

    edgeID_list = network.get_all_edgeID_list(False)
    edge_info_list = {}
    for edgeID in edgeID_list:
        pos = get_pos_edgeID(traci_conn, network, edgeID)
        edge_info = {"pos": list(pos)}
        edge_info_list[edgeID] = edge_info

    route_data_list = []
    route_data_num = 0

    for i, start_edgeID in enumerate(edgeID_list):
        start_pos = edge_info_list[start_edgeID]["pos"]
        for j, target_edgeID in enumerate(edgeID_list):
            end_pos = edge_info_list[target_edgeID]["pos"]
            if i == j:
                continue
            cur_length = calc_distance(start_pos, end_pos)
            if cur_length >= length:
                route = traci_conn.simulation.findRoute(start_edgeID, target_edgeID)
                if len(route.edges) > 0:
                    route_data_num += 1
                    route_data_list.append([start_edgeID, target_edgeID])
                    if route_data_num >= max_data:
                        break
        if route_data_num >= max_data:
            break

    route_data_save(route_data_list, str(route_dir_path))
    try:
        traci_conn.close()
    except traci_conn.exceptions.FatalTraCIError as ci:
        print(ci)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("--area", type=int, default=0)
    parser.add_argument("--net-file-name", type=str, default="osm.net.xml")
    parser.add_argument("--sumocfg-file-name", type=str, default="osm.sumocfg")
    parser.add_argument("--length", type=float, default=1000.0)
    parser.add_argument("--max-data", type=int, default=100000)
    args = parser.parse_args()

    main(
        args.area,
        args.net_file_name,
        args.sumocfg_file_name,
        args.length,
        args.max_data,
    )
