#!/usr/bin/env python3
"""轻量版 Trigno EMG 实时绘图（Windows 友好）。

特点：
- 仅使用标准库 + numpy + matplotlib
- 仅接收 EMG（端口 50041），不接收 ACC
- 对齐 MATLAB 示例的命令流程：RATE / RATE? / START
- 适合在 Windows 上直接运行
"""

from __future__ import annotations

import argparse
import socket
import time
from collections import deque
from typing import List

import matplotlib.pyplot as plt
import numpy as np


def send_cmd(sock: socket.socket, cmd: str, wait_s: float = 0.2, resp_timeout: float = 0.5) -> str:
    """向 Trigno 命令端口发送命令并读取响应。"""
    sock.sendall((cmd + "\r\n\r").encode("ascii"))
    time.sleep(wait_s)

    end_t = time.time() + resp_timeout
    sock.settimeout(0.1)
    chunks = []
    while time.time() < end_t:
        try:
            data = sock.recv(4096)
            if data:
                chunks.append(data)
                # 有响应后再给一个短窗口收尾包
                time.sleep(0.03)
        except socket.timeout:
            if chunks:
                break
    return b"".join(chunks).decode(errors="ignore").strip()


def drain_comm(sock: socket.socket) -> None:
    """清空命令口缓存（对齐 MATLAB 中 fread 清缓存步骤）。"""
    sock.settimeout(0.05)
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
    except socket.timeout:
        pass


def recv_exact(sock: socket.socket, nbytes: int) -> bytes:
    """从 TCP 流中读取固定长度字节。"""
    buf = bytearray()
    while len(buf) < nbytes:
        chunk = sock.recv(nbytes - len(buf))
        if not chunk:
            raise ConnectionError("EMG socket disconnected")
        buf.extend(chunk)
    return bytes(buf)


def run_plot(
    host: str,
    rate: int = 2000,
    sensors: int = 8,
    chunk_samples: int = 27,
    ylim_uv: float = 200.0,
) -> None:
    """实时读取并绘制 EMG。"""
    num_channels = sensors * 2  # 与 MATLAB 示例一致：8 传感器 -> 16 通道 EMG
    window_samples = rate  # 显示 1 秒窗口

    comm = socket.create_connection((host, 50040), timeout=2.0)
    emg = socket.create_connection((host, 50041), timeout=2.0)
    comm.settimeout(None)
    # 使用短超时，避免在无数据时阻塞GUI刷新（Windows下尤为重要）
    emg.settimeout(0.05)

    # 对齐 MATLAB 命令时序
    time.sleep(0.2)
    drain_comm(comm)
    send_cmd(comm, f"RATE {rate}", wait_s=0.3, resp_timeout=0.8)
    rate_resp = ""
    for _ in range(3):
        rate_resp = send_cmd(comm, "RATE?", wait_s=0.25, resp_timeout=0.8)
        if rate_resp:
            break
    send_cmd(comm, "START", wait_s=0.2, resp_timeout=0.3)

    if not rate_resp:
        print("[WARN] RATE? -> N/A （未在超时时间内收到命令口响应，但仍继续尝试接收EMG数据）")
    else:
        print(f"[INFO] connected to {host}, RATE? -> {rate_resp}")

    # 每通道 1 秒环形缓冲
    buffers: List[deque[float]] = [deque(maxlen=window_samples) for _ in range(num_channels)]

    # 绘图初始化
    cols = 4
    rows = int(np.ceil(num_channels / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.2 * rows), sharex=True)
    axes = np.atleast_1d(axes).flatten()

    x = np.arange(window_samples)
    y_limit_v = abs(ylim_uv) * 1e-6
    lines = []
    for ch in range(num_channels):
        ax = axes[ch]
        (line,) = ax.plot(x, np.zeros_like(x), lw=1)
        ax.set_title(f"EMG {ch + 1}")
        ax.set_ylim(-y_limit_v, y_limit_v)
        ax.grid(True, alpha=0.3)
        lines.append(line)

    for ax in axes[num_channels:]:
        ax.set_visible(False)

    status = fig.text(0.01, 0.99, "", ha="left", va="top", fontsize=10)
    fig.suptitle("Trigno EMG Real-time (Lightweight)")
    plt.tight_layout()
    plt.ion()
    plt.show(block=False)

    # 非阻塞式累计缓冲：即使短时无数据，也能保持窗口刷新，不会“卡死不画图”。
    frame_bytes = num_channels * 4
    pending = bytearray()

    try:
        while plt.fignum_exists(fig.number):
            # 1) 尝试收取网络数据（无数据时超时返回，继续刷新界面）
            try:
                chunk = emg.recv(4096)
                if chunk:
                    pending.extend(chunk)
            except socket.timeout:
                pass

            # 2) 解析尽可能多的完整帧（每轮最多处理 chunk_samples 帧，控制CPU占用）
            frames_parsed = 0
            while len(pending) >= frame_bytes and frames_parsed < chunk_samples:
                raw = bytes(pending[:frame_bytes])
                del pending[:frame_bytes]
                row = np.frombuffer(raw, dtype="<f4")
                if row.size != num_channels:
                    continue
                for ch in range(num_channels):
                    buffers[ch].append(float(row[ch]))
                frames_parsed += 1

            # 3) 更新曲线
            for ch in range(num_channels):
                arr = np.asarray(buffers[ch], dtype=np.float32)
                y = np.zeros(window_samples, dtype=np.float32)
                if arr.size:
                    y[-arr.size :] = arr
                lines[ch].set_ydata(y)

            # 4) 状态：通道1 RMS/峰峰值（uV）
            ch0 = np.asarray(buffers[0], dtype=np.float32)
            if ch0.size:
                rms_uv = float(np.sqrt(np.mean(ch0**2)) * 1e6)
                pp_uv = float((np.max(ch0) - np.min(ch0)) * 1e6)
            else:
                rms_uv = 0.0
                pp_uv = 0.0
            status.set_text(
                f"CH1 RMS={rms_uv:.2f}uV | PP={pp_uv:.2f}uV | pending={len(pending)} bytes"
            )

            fig.canvas.draw_idle()
            plt.pause(0.01)

    finally:
        try:
            send_cmd(comm, "STOP", wait_s=0.05)
        except Exception:
            pass
        comm.close()
        emg.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="轻量版 Trigno EMG 实时绘图")
    parser.add_argument("--host", default="192.168.0.58", help="Trigno Utility 所在主机 IP")
    parser.add_argument("--rate", type=int, default=2000, help="采样率")
    parser.add_argument("--sensors", type=int, default=8, help="传感器数量")
    parser.add_argument("--chunk-samples", type=int, default=27, help="每次读取样本数（默认 27）")
    parser.add_argument("--ylim-uv", type=float, default=200.0, help="绘图Y轴范围(±uV)")
    args = parser.parse_args()

    run_plot(
        host=args.host,
        rate=args.rate,
        sensors=args.sensors,
        chunk_samples=args.chunk_samples,
        ylim_uv=args.ylim_uv,
    )
