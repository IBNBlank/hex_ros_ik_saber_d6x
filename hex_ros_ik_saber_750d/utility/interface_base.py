#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-12
################################################################

import json
from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod
from hex_util_msg.dataclass import HexBaseJntState, HexBasePose


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}
        self._prog_param = {}
        self._model_param = {}
        self._limit_param = {}

        ### rx msg queues
        self._target_pose_deque = deque()

        ### name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### parameters
    ####################
    def _str_to_list(self, list_str: list) -> list:
        result = []
        for s in list_str:
            l = json.loads(s)
            result.append(l)
        return result

    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_prog_param(self) -> dict:
        return self._prog_param

    def get_model_param(self) -> dict:
        return self._model_param

    def get_limit_param(self) -> dict:
        return self._limit_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_joint_state(self, out: HexBaseJntState):
        raise NotImplementedError("InterfaceBase.pub_joint_state")

    @abstractmethod
    def pub_debug_pose(self, out: HexBasePose):
        raise NotImplementedError("InterfaceBase.pub_debug_pose")

    @abstractmethod
    def pub_ik_success(self, out: bool):
        raise NotImplementedError("InterfaceBase.pub_ik_success")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            ret = None
            while dq:
                ret = dq.popleft()
            return ret

    # get target pose
    def get_target_pose(self, latest: bool = False) -> Optional[HexBasePose]:
        return self.deque_helper(self._target_pose_deque, latest)
