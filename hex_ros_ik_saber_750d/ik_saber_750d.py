#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-11
################################################################

import os
import sys
import numpy as np
from hex_util_msg.dataclass import HexBaseJntState, HexBasePose, HexBaseVector3, HexBaseQuaternion
from hex_util_ros import HexDynUtil, part2se3, se32part

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface


class IkSaber750d:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("ik_saber_750d")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__prog_param = self.__data_interface.get_prog_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__limit_param = self.__data_interface.get_limit_param()

        ### utility
        self.__dyn_util = HexDynUtil(
            model_path=self.__model_param["path"],
            last_link=self.__model_param["end_effector"],
        )

        ### variables
        self.__cur_tar_se3 = None
        self.__cur_jnt_state = HexBaseJntState(
            position=np.array(
                [0.0, 0.80224039, 1.73428836, 0.0, -0.96573333, 0.0]),
            velocity=np.zeros(6),
            effort=np.zeros(6),
        )
        print(f"#### cur_jnt_state: {self.__cur_jnt_state} ####")
        print(f"#### pos: {self.__cur_jnt_state.position} ####")
        print(f"#### vel: {self.__cur_jnt_state.velocity} ####")
        print(f"#### eff: {self.__cur_jnt_state.effort} ####")

    def run(self):
        while self.__data_interface.ok():
            # update target pose
            cur_tar_pose = self.__data_interface.get_target_pose(latest=True)
            if cur_tar_pose is not None:
                self.__cur_tar_se3 = part2se3(
                    np.array([
                        cur_tar_pose.position.x,
                        cur_tar_pose.position.y,
                        cur_tar_pose.position.z,
                    ]),
                    np.array([
                        cur_tar_pose.orientation.w,
                        cur_tar_pose.orientation.x,
                        cur_tar_pose.orientation.y,
                        cur_tar_pose.orientation.z,
                    ]),
                )

            if self.__cur_tar_se3 is not None:
                # forward kinematics
                cur_ee_pos, cur_ee_quat = self.__dyn_util.forward_kinematics(
                    self.__cur_jnt_state.position)[-1]

                # se3 interpolation
                cur_ee_se3 = part2se3(cur_ee_pos, cur_ee_quat)
                err_se3 = self.__cur_tar_se3 - cur_ee_se3
                err_ratio = np.maximum(
                    np.fabs(err_se3 / self.__limit_param["se3_err"]).max(),
                    1.0)
                err_se3_mid = err_se3 / err_ratio + cur_ee_se3
                mid_pos, mid_quat = se32part(err_se3_mid)

                # inverse kinematics
                success, tar_jnt_state, _ = self.__dyn_util.inverse_kinematics(
                    (mid_pos, mid_quat), self.__cur_jnt_state.position)
                if success:
                    self.__cur_jnt_state.position = tar_jnt_state
                    self.__data_interface.pub_ik_success(True)
                    if self.__prog_param["debug"]:
                        ik_ee_pos, ik_ee_quat = self.__dyn_util.forward_kinematics(
                            self.__cur_jnt_state.position)[-1]
                        self.__data_interface.pub_debug_pose(
                            HexBasePose(
                                position=HexBaseVector3(
                                    x=ik_ee_pos[0],
                                    y=ik_ee_pos[1],
                                    z=ik_ee_pos[2],
                                ),
                                orientation=HexBaseQuaternion(
                                    w=ik_ee_quat[0],
                                    x=ik_ee_quat[1],
                                    y=ik_ee_quat[2],
                                    z=ik_ee_quat[3],
                                ),
                            ))
                else:
                    self.__data_interface.pub_ik_success(False)

            self.__data_interface.pub_joint_state(self.__cur_jnt_state)

            # sleep
            self.__data_interface.sleep()


def main():
    ik_saber_750d = IkSaber750d()
    try:
        ik_saber_750d.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
