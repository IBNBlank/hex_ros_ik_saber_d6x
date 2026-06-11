#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-27
################################################################

import numpy as np

from hex_util_ros import HexDynUtil

POSE_LIST = [
    (np.array([0.3, 0.0, 0.2]), np.array([1.0, 0.0, 0.0, 0.0])),
    (np.array([0.3, 0.3, 0.3]), np.array([1.0, 0.0, 0.0, 0.0])),
    (np.array([0.3, -0.3, 0.3]), np.array([1.0, 0.0, 0.0, 0.0])),
    (np.array([0.3, 0.3, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
    (np.array([0.3, -0.3, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
]


def main():
    dyn_util = HexDynUtil(
        'saber_750d.urdf',
        'link_6',
    )
    print(f"joint num: {dyn_util.get_joint_num()}")

    for pos, quat in POSE_LIST:
        ### inverse kinematics
        success, tar_state, err_norm = dyn_util.inverse_kinematics(
            (pos, quat),
            np.array([0.0, 0.80224039, 1.73428836, 0.0, -0.96573333, 0.0]))
        print(f"IK success: {success}, err_norm: {err_norm}")
        print(f"IK q: {tar_state}")

        ### forward kinematics
        poses = dyn_util.forward_kinematics(tar_state)
        print(f"end pose: {poses[-1]}")


if __name__ == '__main__':
    main()
