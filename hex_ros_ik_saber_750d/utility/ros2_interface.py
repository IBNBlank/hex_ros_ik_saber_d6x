#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-12
################################################################

import threading
import numpy as np

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from hex_util_msg.dataclass import HexBaseJntState, HexBasePose, HexBaseVector3, HexBaseQuaternion

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 100.0)
        self._rate_param["ros"] = self.__node.get_parameter('rate_ros').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### pamameter
        # declare parameters
        self.__node.declare_parameter('rate_state', 100.0)
        self.__node.declare_parameter('prog_debug', True)
        self.__node.declare_parameter('model_path', "")
        self.__node.declare_parameter('model_base', "base_link")
        self.__node.declare_parameter('model_joint_names', [""])
        self.__node.declare_parameter('model_end_effector', "")
        self.__node.declare_parameter('model_end_quat', [1.0, 0.0, 0.0, 0.0])
        self.__node.declare_parameter('limit_pos', [""])
        self.__node.declare_parameter('limit_vel', [""])
        self.__node.declare_parameter('limit_acc', [""])
        self.__node.declare_parameter('limit_joint_err', 0.1)
        self.__node.declare_parameter('limit_se3_err', 0.1)

        # rate
        self._rate_param.update({
            "state":
            self.__node.get_parameter('rate_state').value,
        })
        # prog
        self._prog_param = {
            "debug": self.__node.get_parameter('prog_debug').value,
        }
        # model
        self._model_param = {
            "path":
            self.__node.get_parameter('model_path').value,
            "base":
            self.__node.get_parameter('model_base').value,
            "joint_names":
            self.__node.get_parameter('model_joint_names').value,
            "end_effector":
            self.__node.get_parameter('model_end_effector').value,
            "end_quat":
            np.array(self.__node.get_parameter('model_end_quat').value),
        }
        # limit
        self._limit_param = {
            "pos":
            np.array(
                self._str_to_list(
                    self.__node.get_parameter('limit_pos').value)),
            "vel":
            np.array(
                self._str_to_list(
                    self.__node.get_parameter('limit_vel').value)),
            "acc":
            np.array(
                self._str_to_list(
                    self.__node.get_parameter('limit_acc').value)),
            "joint_err":
            self.__node.get_parameter('limit_joint_err').value,
            "se3_err":
            self.__node.get_parameter('limit_se3_err').value,
        }

        ### publisher
        self.__joint_state_pub = self.__node.create_publisher(
            JointState,
            'joint_states',
            10,
        )
        self.__debug_pose_pub = self.__node.create_publisher(
            PoseStamped,
            'debug_pose',
            10,
        )
        self.__ik_success_pub = self.__node.create_publisher(
            Bool,
            'ik_success',
            10,
        )

        ### subscriber
        self.__target_pose_sub = self.__node.create_subscription(
            PoseStamped,
            'target_pose',
            self.__target_pose_callback,
            10,
        )

        ### spin thread
        self.__shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        if self.__shutting_down:
            return
        self.__shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    def pub_joint_state(self, out: HexBaseJntState):
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = self._model_param["base"]
        msg.name = self._model_param["joint_names"]
        msg.position = out.position.tolist()
        msg.velocity = out.velocity.tolist()
        msg.effort = out.effort.tolist()
        self.__joint_state_pub.publish(msg)

    def pub_debug_pose(self, out: HexBasePose):
        msg = PoseStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
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
