# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2010-2020 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @original file    findAllRoutes.py
# @file    find_all_routes.py
# @author  tomo
# @date    2020-01-09

"""
determine all possible routes between source and destination edges
"""
from __future__ import absolute_import
from __future__ import print_function

import sys
import numpy as np
import sumolib

from tqdm import tqdm
import pathlib
from gym_sumo.envs import constans as gc

# from IPython import embed


AREA = gc.AREA
CONFIG_RELATIVE_PATH = "sumo_configs/"
SUMO_NET_EXTEND = ".net.xml"
SUMO_CONFIG_EXTEND = ".sumocfg"
SUMO_ROUTE_EXTEND = ".rou.xml"
SUMO_DEFAULT_NET_FILENAME = "osm.net.xml"
SUMO_DEFAULT_ROUTE_FILENAME = "osm.rou.xml"

CWD_PATH = pathlib.os.path.dirname(__file__)
CONFIG_DEFAULT_AREA = CONFIG_RELATIVE_PATH + AREA[0]
CONFIG_DEFAULT_ABS_PATH = pathlib.Path(
    pathlib.os.path.join(CWD_PATH, CONFIG_DEFAULT_AREA)
).resolve()
NET_DEAFULT_FILE_PATH = CONFIG_DEFAULT_ABS_PATH / SUMO_DEFAULT_NET_FILENAME
ROUTE_DEAFULT_FILE_PATH = CONFIG_DEFAULT_ABS_PATH / SUMO_DEFAULT_ROUTE_FILENAME


def get_options(args=None):
    argParser = sumolib.options.ArgumentParser()
    argParser.add_argument(
        "-n",
        "--net-file",
        default=str(NET_DEAFULT_FILE_PATH),
        help="the SUMO net filename",
    )
    argParser.add_argument(
        "-o",
        "--output-file",
        default=str(ROUTE_DEAFULT_FILE_PATH),
        help="the route output filename",
    )
    options = argParser.parse_args(args=args)
    if options.net_file is None or options.output_file is None:
        argParser.print_help()
        sys.exit()
    return options


def calc_distance(from_pos, to_pos):
    return float(
        np.sqrt((to_pos[0] - from_pos[0]) ** 2 + (to_pos[1] - from_pos[1]) ** 2)
    )


def get_all_abs_length(from_edge_obj, to_edge_obj):
    from_lane_obj = from_edge_obj.getLane(0)
    to_lane_obj = to_edge_obj.getLane(0)
    from_lane_shape = from_lane_obj.getShape()
    to_lane_shape = to_lane_obj.getShape()
    abs_length_list = []
    for from_pos in from_lane_shape:
        tmp = []
        for to_pos in to_lane_shape:
            dis = calc_distance(from_pos, to_pos)
            tmp.append(dis)
        abs_length_list.append(tmp)
    return abs_length_list


def get_abs_length(from_edge_obj, to_edge_obj):
    all_abs_length = get_all_abs_length(from_edge_obj, to_edge_obj)
    from_num, to_num = len(all_abs_length), len(all_abs_length[0])
    s_s_abs_length = all_abs_length[0][0]
    s_e_abs_length = all_abs_length[0][to_num - 1]
    e_e_abs_length = all_abs_length[from_num - 1][to_num - 1]
    e_s_abs_length = all_abs_length[from_num - 1][0]
    return [s_s_abs_length, s_e_abs_length, e_e_abs_length, e_s_abs_length]


def get_travel_time(edge_obj):
    speed = edge_obj.getSpeed()
    speed = speed if speed > 0 else 11.0
    length = edge_obj.getLength()
    return length / speed


def get_route_info(net, route_edges):
    num = len(route_edges)
    route_info = {
        "edge_num": 0,
        "length": 0.0,
        "travel_time": 0.0,
        "cost": 0.0,
        "abs_length_start_start": 0.0,
        "abs_length_start_end": 0.0,
        "abs_length_end_end": 0.0,
        "abs_length_end_start": 0.0,
    }
    if num == 0:
        return route_info
    start_edge_obj, end_edge_obj = route_edges[0], route_edges[num - 1]
    abs_length_list = get_abs_length(start_edge_obj, end_edge_obj)
    route_info["edge_num"] = num
    route_info["abs_length_start_start"] = abs_length_list[0]
    route_info["abs_length_start_end"] = abs_length_list[1]
    route_info["abs_length_end_end"] = abs_length_list[2]
    route_info["abs_length_end_start"] = abs_length_list[3]
    last_edge_obj = route_edges[0]
    length = last_edge_obj.getLength()
    travel_time = get_travel_time(last_edge_obj)
    for next_edge_obj in route_edges[1:]:
        if net.hasInternal:
            min_internal_length = 1e400
            internal_travel_time = 1e400
            for c in last_edge_obj.getConnections(next_edge_obj):
                via_laneID = c.getViaLaneID()
                if via_laneID != "":
                    via_lane_obj = net.getLane(via_laneID)
                    internal_length = via_lane_obj.getLength()
                    if min_internal_length >= internal_length:
                        min_internal_length = internal_length
                        internal_travel_time = get_travel_time(via_lane_obj)
            if min_internal_length < 1e400:
                length += min_internal_length
                travel_time += internal_travel_time
        length += next_edge_obj.getLength()
        travel_time += get_travel_time(next_edge_obj)
        last_edge_obj = next_edge_obj
    route_info["length"] = length
    route_info["cost"] = travel_time
    route_info["travel_time"] = travel_time
    return route_info


def to_xml(route_info):
    attr = "        <info"
    end = "/>"
    for key in route_info:
        tmp = " " + key + "=" + str(route_info.get(key))
        attr += tmp
    return attr + end


def main(options):
    net = sumolib.net.readNet(options.net_file, withInternal=True)
    stubs = [[e] for e in net.getEdges(withInternal=False)]
    targets = set(net.getEdges(withInternal=False))
    with open(options.output_file, "w") as outf:
        sumolib.xml.writeHeader(outf, root="routes")
        idx = 0
        start_stubs = stubs.copy()
        for target in tqdm(targets):
            stubs = start_stubs.copy()
            while stubs:
                s = stubs.pop(0)
                if s[-1] == target:
                    route_info = get_route_info(net, s)
                    child_attr = to_xml(route_info)
                    print(
                        '    <route id="route%s" edges="%s">\n%s\n    </route>'
                        % (idx, " ".join([e.getID() for e in s]), child_attr),
                        file=outf,
                    )
                    idx += 1
                else:
                    seen = set(s)
                    for edge in s[-1].getOutgoing():
                        if edge not in seen:
                            stubs.append(s + [edge])
        outf.write("</routes>\n")


if __name__ == "__main__":
    main(get_options())
