#!/usr/bin/env python3
"""控制输入：目前只有零力矩，键盘控制以后加在这里。

物理线程每一步之前调一次 `update(data)`，控制器只负责写 `data.ctrl`。
以后接键盘时新增一个类即可（比如从队列里取最近一条指令），物理线程不用改。
"""

from __future__ import annotations


class ZeroTorque:
    """零力矩：等同于任务 2 的趴卧基线，用来验证仿真循环本身。"""

    name = "zero_torque"

    def update(self, data) -> None:
        data.ctrl[:] = 0.0
