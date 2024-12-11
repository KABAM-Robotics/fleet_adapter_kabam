#!/usr/bin/env python3
import math
import nudged


class MapTransform():
    """A Module that stores transforms for mapping between RMF and Smart+."""

    def __init__(
            self,
            rmf_coords,
            robot_coords,
            ori_diff
            ):
        self.transforms = {
            'rmf_to_robot': nudged.estimate(rmf_coords, robot_coords),
            'robot_to_rmf': nudged.estimate(robot_coords, rmf_coords),
        }
        self.transforms['orientation_offset'] = (self.transforms['rmf_to_robot'].get_rotation()
                                                 + math.radians(ori_diff))
