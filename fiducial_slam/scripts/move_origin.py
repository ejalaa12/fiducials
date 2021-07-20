#!/usr/bin/python

"""
Move origin of fiducial co-ordinate system
"""

import numpy, sys, os
# from fiducial_slam.map import Map
from typing import List
from dataclasses import dataclass

@dataclass
class Fiducial:
    id: int
    x: float
    y: float
    z: float
    pan: float
    tilt: float
    roll: float
    variance: float
    num_obs: int
    links: List[int]


class Map(object):
    def __init__(self, filename):
        print(f"Opening {filename}")
        self.filename = filename
        self.fiducials = {}
        with open(filename, 'r') as f:
            for l in f.readlines():
                fid = Fiducial(l.split()[0], *map(float, l.split()[1:9]), l.split()[9:])
                self.fiducials[fid.id] = fid
                print(fid)

    def save(self):
        with open(self.filename, 'w') as f:
            for fid in self.fiducials.values():
                l = f'{fid.id} {fid.x} {fid.y} {fid.z} {fid.pan} {fid.tilt} {fid.roll} {fid.variance} {fid.num_obs} {" ".join(fid.links)}\n'
                print(l)
                f.write(l)



if __name__ == "__main__":
    argc = len(sys.argv)
    if argc != 4 and argc != 5:
        print("Usage: %s x y z [file]" % sys.argv[0])
        sys.exit(1)
    offset = numpy.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
    if argc == 5:
        filename = sys.argv[4]
    else:
        filename = "~/.ros/slam/map2.txt"
    filename = os.path.expanduser(filename)
    m = Map(filename)
    for fid in m.fiducials.values():
        fid.x += offset[0]
        fid.y += offset[1]
        fid.z += offset[2]
    m.save()
