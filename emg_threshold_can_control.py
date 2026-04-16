#!/usr/bin/env python3
"""EMG 阈值触发 CAN 控制脚本。

功能：
1) 仅接收 EMG（不接收 ACC）。
2) 根据指定通道 EMG 强度（uV）做阈值检测，阈值默认 30uV。
3) 超阈值后，按 robolimb_boardmain_large 代码中的 SDO 顺序，
   向电机节点 ID=3 发送一整套位置控制指令。
"""

from __future__ import annotations

import argparse
import socket
import time
from dataclasses import dataclass
from typing import Iterable, List

import can
import numpy as np


@dataclass(frozen=True)
class SDOFrame:
    name: str
    data: bytes


def can_id_for_node(node_id: int) -> int:
    return 0x600 + node_id


# 参考 robolimb_boardmain_large/USER/sdo_frames.c
SDO_ACTIVATE_INIT = SDOFrame("ACTIVATE_INIT", bytes([0x01, 0x00]))
SDO_ACTIVATE_PPM = SDOFrame("ACTIVATE_PPM", bytes([0x2F, 0x60, 0x60, 0x00, 0x01]))
SDO_ACTIVATE_SETV1000 = SDOFrame(
    "ACTIVATE_SETV1000", bytes([0x23, 0x81, 0x60, 0x00, 0xE8, 0x30, 0x00, 0x00])
)
SDO_DISABLE = SDOFrame("DISABLE", bytes([0x2B, 0x40, 0x60, 0x00, 0x06, 0x00]))
SDO_ENABLE = SDOFrame("ENABLE", bytes([0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00]))
SDO_GO = SDOFrame("GO", bytes([0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00]))

SDO_TARGET_POS_NODEUN500 = SDOFrame(
    "TARGET_POS_-500", bytes([0x23, 0x7A, 0x60, 0x00, 0x0C, 0xFE, 0xFF, 0xFF])
)
SDO_TARGET_POS_NODE500 = SDOFrame(
    "TARGET_POS_+500", bytes([0x23, 0x7A, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00])
)


class EmgReader:
    """通过 Delsys SDK 读取 EMG 流。"""

    def __init__(self, host: str, sample_rate: int = 2000, num_channels: int = 16):
        self.host = host
        self.sample_rate = sample_rate
        self.num_channels = num_channels
        self.comm_sock: socket.socket | None = None
        self.emg_sock: socket.socket | None = None

    def _send_cmd(self, cmd: str, wait_s: float = 0.1) -> str:
        assert self.comm_sock is not None
        self.comm_sock.sendall((cmd + "\r\n\r").encode("ascii"))
        time.sleep(wait_s)
        self.comm_sock.settimeout(0.1)
        try:
            return self.comm_sock.recv(4096).decode(errors="ignore").strip()
        except socket.timeout:
            return ""

    def connect(self) -> None:
        self.comm_sock = socket.create_connection((self.host, 50040), timeout=2)
        self.emg_sock = socket.create_connection((self.host, 50041), timeout=2)
        self._send_cmd(f"RATE {self.sample_rate}")
        self._send_cmd("START")

    def read_frame(self) -> np.ndarray:
        """读取一帧 EMG（16 通道 float32，单位 V）。"""
        assert self.emg_sock is not None
        nbytes = self.num_channels * 4
        data = bytearray()
        while len(data) < nbytes:
            chunk = self.emg_sock.recv(nbytes - len(data))
            if not chunk:
                raise ConnectionError("EMG socket disconnected")
            data.extend(chunk)
        return np.frombuffer(data, dtype="<f4")

    def close(self) -> None:
        if self.comm_sock:
            try:
                self._send_cmd("STOP", wait_s=0.05)
            except Exception:
                pass
            self.comm_sock.close()
            self.comm_sock = None
        if self.emg_sock:
            self.emg_sock.close()
            self.emg_sock = None


class MotorController:
    """封装 CAN SDO 发送。"""

    def __init__(self, channel: str, bustype: str, bitrate: int, node_id: int = 3):
        self.node_id = node_id
        self.bus = can.interface.Bus(channel=channel, interface=bustype, bitrate=bitrate)

    def send_sdo(self, frame: SDOFrame) -> None:
        msg = can.Message(
            arbitration_id=can_id_for_node(self.node_id),
            data=frame.data,
            is_extended_id=False,
            is_remote_frame=False,
        )
        self.bus.send(msg)
        print(f"[CAN] node={self.node_id} {frame.name} data={frame.data.hex(' ')}")

    def run_full_position_sequence(self) -> None:
        """向 ID=3 电机发送一整套位置运动指令（带合理延时）。"""
        # 对齐 main.c 的初始化顺序
        for frame in [SDO_ACTIVATE_INIT, SDO_ACTIVATE_PPM, SDO_ACTIVATE_SETV1000, SDO_DISABLE]:
            self.send_sdo(frame)
            time.sleep(0.2)

        # 第一段动作：目标 -500 -> GO
        self.send_sdo(SDO_ENABLE)
        time.sleep(0.2)
        self.send_sdo(SDO_TARGET_POS_NODEUN500)
        time.sleep(0.2)
        self.send_sdo(SDO_GO)
        time.sleep(2.0)

        # 第二段动作：目标 +500 -> GO（形成往返的一整套位置指令）
        self.send_sdo(SDO_ENABLE)
        time.sleep(0.2)
        self.send_sdo(SDO_TARGET_POS_NODE500)
        time.sleep(0.2)
        self.send_sdo(SDO_GO)
        time.sleep(2.0)


def rms_uv(samples_v: Iterable[float]) -> float:
    arr = np.asarray(list(samples_v), dtype=np.float32)
    if arr.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(arr**2)) * 1e6)


def main() -> None:
    parser = argparse.ArgumentParser(description="EMG 阈值触发 ID=3 电机 CAN 位置控制")
    parser.add_argument("--host", default="192.168.0.58", help="Trigno Utility 主机 IP")
    parser.add_argument("--emg-channel", type=int, default=3, help="用于触发的 EMG 通道号(1-16)")
    parser.add_argument("--threshold-uv", type=float, default=30.0, help="阈值，单位 uV")
    parser.add_argument("--cooldown", type=float, default=3.0, help="触发后冷却时间(s)")
    parser.add_argument("--can-if", default="can0", help="CAN 通道名，如 can0")
    parser.add_argument("--can-type", default="socketcan", help="python-can interface 类型")
    parser.add_argument("--bitrate", type=int, default=1000000, help="CAN 波特率")
    args = parser.parse_args()

    ch_idx = args.emg_channel - 1
    if not (0 <= ch_idx < 16):
        raise ValueError("--emg-channel 必须在 1~16")

    emg = EmgReader(host=args.host)
    motor = MotorController(
        channel=args.can_if,
        bustype=args.can_type,
        bitrate=args.bitrate,
        node_id=3,
    )

    last_trigger = 0.0
    try:
        emg.connect()
        print("开始监测 EMG... (仅EMG, 不接收ACC)")
        while True:
            frame = emg.read_frame()
            strength_uv = rms_uv([frame[ch_idx]])
            now = time.time()
            print(f"EMG ch{args.emg_channel}: {strength_uv:.2f} uV")

            if strength_uv >= args.threshold_uv and (now - last_trigger) >= args.cooldown:
                print(
                    f"[TRIGGER] ch{args.emg_channel}={strength_uv:.2f}uV >= {args.threshold_uv:.2f}uV, 发送电机指令"
                )
                motor.run_full_position_sequence()
                last_trigger = time.time()

            time.sleep(0.01)
    finally:
        emg.close()


if __name__ == "__main__":
    main()
