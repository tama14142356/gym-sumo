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


def main(options):
    net = sumolib.net.readNet(options.net_file)
    stubs = [[e] for e in net.getEdges()]
    targets = set(net.getEdges())
    with open(options.output_file, "w") as outf:
        sumolib.xml.writeHeader(outf, root="routes")
        idx = 0
        start_stubs = stubs.copy()
        for target in tqdm(targets):
            stubs = start_stubs.copy()
            while stubs:
                s = stubs.pop(0)
                if s[-1] == target:
                    print(
                        '    <route id="%s" edges="%s"/>'
                        % (idx, " ".join([e.getID() for e in s])),
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
