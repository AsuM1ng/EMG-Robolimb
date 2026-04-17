#!/usr/bin/env python3
"""Delsys Trigno SDK 实时 EMG 数据接收与绘图（Python 重写版）。

对应 MATLAB 文件: real_time_data_stream_plotting.m
仅保留 EMG 数据流（端口 50041），不接收 ACC 数据。
"""

from __future__ import annotations

import argparse
import socket
import threading
import time
from collections import deque
from typing import List

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


class TrignoEMGStreamer:
    """负责与 Trigno Control Utility 建立连接并接收 EMG 数据。"""

    def __init__(self, host: str, sample_rate: int = 2000, num_sensors: int = 8):
        self.host = host
        self.sample_rate = sample_rate
        self.num_sensors = num_sensors
        self.num_channels = num_sensors * 2  # MATLAB 例程为 16 通道 EMG

        self._comm_sock: socket.socket | None = None
        self._emg_sock: socket.socket | None = None
        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None

        # 为每个通道保留 1 秒窗口数据，单位：伏特（V）
        self._buffers: List[deque[float]] = [
            deque(maxlen=self.sample_rate) for _ in range(self.num_channels)
        ]
        self._lock = threading.Lock()

    @staticmethod
    def _recv_all(sock: socket.socket, nbytes: int) -> bytes:
        data = bytearray()
        while len(data) < nbytes:
            chunk = sock.recv(nbytes - len(data))
            if not chunk:
                raise ConnectionError("socket closed while receiving")
            data.extend(chunk)
        return bytes(data)

    def _send_cmd(self, cmd: str, wait_s: float = 0.2) -> str:
        assert self._comm_sock is not None
        self._comm_sock.sendall((cmd + "\r\n\r").encode("ascii"))
        time.sleep(wait_s)
        self._comm_sock.settimeout(0.1)
        try:
            resp = self._comm_sock.recv(4096)
            return resp.decode(errors="ignore").strip()
        except socket.timeout:
            return ""

    def connect(self) -> None:
        self._comm_sock = socket.create_connection((self.host, 50040), timeout=2.0)
        self._emg_sock = socket.create_connection((self.host, 50041), timeout=2.0)

        # 配置采样率并启动流
        self._send_cmd(f"RATE {self.sample_rate}")
        _ = self._send_cmd("RATE?")
        self._send_cmd("START", wait_s=0.1)

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _reader_loop(self) -> None:
        assert self._emg_sock is not None

        # 每帧 16 通道 float32，小端，4 字节
        frame_bytes = self.num_channels * 4

        while not self._stop_event.is_set():
            try:
                raw = self._recv_all(self._emg_sock, frame_bytes)
            except (ConnectionError, OSError):
                break

            # '<f4' 表示 little-endian float32
            values = np.frombuffer(raw, dtype="<f4")
            if values.size != self.num_channels:
                continue

            with self._lock:
                for i, v in enumerate(values):
                    self._buffers[i].append(float(v))

    def get_window(self) -> np.ndarray:
        """返回 shape=(num_channels, sample_rate) 的窗口数据，长度不足会前补零。"""
        with self._lock:
            out = np.zeros((self.num_channels, self.sample_rate), dtype=np.float32)
            for i, dq in enumerate(self._buffers):
                arr = np.asarray(dq, dtype=np.float32)
                # len(arr)==0 时，"-0" 会被解释为 0，直接赋值会触发广播错误。
                if arr.size == 0:
                    continue
                out[i, -arr.size :] = arr
            return out

    def close(self) -> None:
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

        if self._comm_sock:
            try:
                self._send_cmd("STOP", wait_s=0.05)
            except Exception:
                pass
            self._comm_sock.close()
            self._comm_sock = None

        if self._emg_sock:
            self._emg_sock.close()
            self._emg_sock = None



def run_plot(host: str, sample_rate: int, num_sensors: int) -> None:
    streamer = TrignoEMGStreamer(host=host, sample_rate=sample_rate, num_sensors=num_sensors)
    streamer.connect()

    num_channels = num_sensors * 2
    cols = 4
    rows = int(np.ceil(num_channels / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.2 * rows), sharex=True)
    axes = np.atleast_1d(axes).flatten()

    x = np.arange(sample_rate)
    lines = []
    for ch in range(num_channels):
        ax = axes[ch]
        (line,) = ax.plot(x, np.zeros_like(x), lw=1)
        ax.set_title(f"EMG {ch + 1}")
        ax.set_ylim(-0.005, 0.005)
        ax.grid(True, alpha=0.3)
        lines.append(line)

    # 隐藏未使用的子图
    for ax in axes[num_channels:]:
        ax.set_visible(False)

    fig.suptitle("Trigno Real-time EMG")
    plt.tight_layout()

    # 在部分 IDE/后端中，while+pause 可能只刷新一帧；改为 FuncAnimation 持续调度更新。
    def _update(_frame_idx: int):
        win = streamer.get_window()
        for ch in range(num_channels):
            lines[ch].set_ydata(win[ch])
        return lines

    ani = FuncAnimation(
        fig,
        _update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    plt.ion()
    try:
        plt.show(block=True)
    finally:
        # 防止动画对象被提前回收，并确保连接被正确关闭。
        _ = ani
        streamer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="实时接收 Trigno EMG 并绘图")
    parser.add_argument("--host", default="192.168.0.58", help="运行 Trigno Utility 的主机 IP")
    parser.add_argument("--rate", type=int, default=2000, help="EMG 采样率")
    parser.add_argument("--sensors", type=int, default=8, help="传感器数量")
    args = parser.parse_args()

    run_plot(host=args.host, sample_rate=args.rate, num_sensors=args.sensors)
