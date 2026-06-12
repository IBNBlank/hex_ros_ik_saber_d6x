#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-12
################################################################

import threading
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from hex_util_msg.dataclass import HexBaseJntState, HexBasePose, HexBaseVector3, HexBaseQuaternion

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rospy.init_node(name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 100.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### parameter
        self.__node_name = rospy.get_name()

        # rate
        self._rate_param.update({
            "state": rospy.get_param('~rate_state', 100.0),
        })
        # prog
        self._prog_param = {
            "debug": rospy.get_param('~prog_debug', True),
        }
        # model
        self._model_param = {
            "path": rospy.get_param('~model_path', ""),
            "base": rospy.get_param('~model_base', "base_link"),
            "joint_names": rospy.get_param('~model_joint_names', [""]),
            "end_effector": rospy.get_param('~model_end_effector', ""),
            "end_quat": np.array(
                rospy.get_param('~model_end_quat', [1.0, 0.0, 0.0, 0.0])),
        }
        # limit
        self._limit_param = {
            "pos": np.array(
                self._str_to_list(
                    rospy.get_param('~limit_pos', [""]))),
            "vel": np.array(
                self._str_to_list(
                    rospy.get_param('~limit_vel', [""]))),
            "acc": np.array(
                self._str_to_list(
                    rospy.get_param('~limit_acc', [""]))),
            "joint_err": rospy.get_param('~limit_joint_err', 0.1),
            "se3_err": rospy.get_param('~limit_se3_err', 0.1),
        }

        ### publisher
        self.__joint_state_pub = rospy.Publisher(
            'joint_states',
            JointState,
            queue_size=10,
        )
        self.__debug_pose_pub = rospy.Publisher(
            'debug_pose',
            PoseStamped,
            queue_size=10,
        )
        self.__ik_success_pub = rospy.Publisher(
            'ik_success',
            Bool,
            queue_size=10,
        )

        ### subscriber
        self.__target_pose_sub = rospy.Subscriber(
            'target_pose',
            PoseStamped,
            self.__target_pose_callback,
        )

        ### spin thread
        self.__shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        if self.__shutting_down:
            return
        self.__shutting_down = True
        try:
            rospy.signal_shutdown("shutdown")
        except Exception:
            pass
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    def pub_joint_state(self, out: HexBaseJntState):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._model_param["base"]
        msg.name = self._model_param["joint_names"]
        msg.position = out.position.tolist()
        msg.velocity = out.velocity.tolist()
        msg.effort = out.effort.tolist()
        self.__joint_state_pub.publish(msg)

    def pub_debug_pose(self, out: HexBasePose):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._model_param["base"]
        msg.pose.position.x = out.position.x
        msg.pose.position.y = out.position.y
        msg.pose.position.z = out.position.z
        msg.pose.orientation.w = out.orientation.w
        msg.pose.orientation.x = out.orientation.x
        msg.pose.orientation.y = out.orientation.y
        msg.pose.orientation.z = out.orientation.z
        self.__debug_pose_pub.publish(msg)

    def pub_ik_success(self, out: bool):
        msg = Bool()
        msg.data = out
        self.__ik_success_pub.publish(msg)

    def __target_pose_callback(self, msg: PoseStamped):
        target_pose = HexBasePose(
            position=HexBaseVector3(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z,
            ),
            orientation=HexBaseQuaternion(
                x=msg.pose.orientation.x,
                y=msg.pose.orientation.y,
                z=msg.pose.orientation.z,
                w=msg.pose.orientation.w,
            ),
        )
        self._target_pose_deque.append(target_pose)
