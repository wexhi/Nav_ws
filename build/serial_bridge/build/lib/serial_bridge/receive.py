#!/usr/bin/env python3
"""
Receive and decode Vision_Send_s frames from STM32.

Frame format (little‑endian, total 40 bytes):

| Byte(s) | Name       | C type     |
|---------|------------|------------|
| 0       | header     | uint8  0x5A |
| 1‑4     | real_vx    | float       |
| 5‑8     | real_wz    | float       |
| 9‑24    | q[0..3]    | 4 × float   |
| 25‑28   | ax         | float       |
| 29‑32   | ay         | float       |
| 33‑36   | az         | float       |
| 37‑38   | checksum   | uint16 CRC‑16 over bytes 0‑36 |
| 39      | tail       | uint8  0xAA |

Corresponds to Vision_Send_s in miniPC_process.h
"""
import serial
import struct
from collections import deque
from pathlib import Path
from datetime import datetime
from crc_table import CRC16_TABLE

# ========= CRC‑16 (polynomial 0x1021, initial 0xFFFF) =========
poly = 0x1021
for byte in range(256):
    crc = byte << 8
    for _ in range(8):
        crc = ((crc << 1) ^ poly) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    CRC16_TABLE.append(crc)


def crc16(buf: bytes, init_crc: int = 0xFFFF) -> int:
    crc = init_crc
    for b in buf:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


# ========= Vision frame constants =========
HEADER = 0x5A
TAIL = 0xAA
FRAME_LEN = 40  # bytes

# Little‑endian format string (1B header already handled separately)
_STRUCT = struct.Struct("<ff4ffff")  # 2 × float + 4 × float + 3 × float


def parse_frame(frame: bytes):
    """Return dict with all fields; assume frame length & CRC already checked."""
    body = frame[1 : 1 + _STRUCT.size]  # skip header
    (
        real_vx,
        real_wz,
        q0,
        q1,
        q2,
        q3,
        ax,
        ay,
        az,
    ) = _STRUCT.unpack(body)
    return {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "real_vx": real_vx,
        "real_wz": real_wz,
        "q": (q0, q1, q2, q3),
        "ax": ax,
        "ay": ay,
        "az": az,
    }


def find_frames(stream: serial.Serial):
    """
    Generator that yields valid Vision_Send_s frames from a byte stream.
    Works with blocking or timeout reads.
    """
    buf = deque(maxlen=2 * FRAME_LEN)  # rolling window big enough to cover gaps
    while True:
        data = stream.read(stream.in_waiting or 1)
        if not data:
            continue
        buf.extend(data)
        # Try to align on header byte
        while len(buf) >= FRAME_LEN:
            if buf[0] != HEADER:
                buf.popleft()
                continue
            candidate = bytes([buf.popleft() for _ in range(FRAME_LEN)])
            if candidate[-1] != TAIL:
                # header matched but tail wrong—slide by one and re‑sync
                buf.appendleft(candidate[1])
                continue
            if crc16(candidate[:-3]) != int.from_bytes(candidate[-3:-1], "little"):
                # CRC mismatch—discard first byte and continue
                buf.appendleft(candidate[1])
                continue
            yield candidate


def main():
    # -------- Serial configuration (adjust to your device) --------
    ser = serial.Serial(
        port="/dev/ttyUSB0",  # or /dev/ttyUSB0, COM3, …
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,  # 20 ms non‑blocking
    )

    log_file = Path("vision_log.csv")
    if not log_file.exists():
        log_file.write_text(
            "timestamp,real_vx,real_wz,q0,q1,q2,q3,ax,ay,az\n", encoding="utf‑8"
        )

    print("Listening …  Ctrl‑C to stop.")
    try:
        for frame in find_frames(ser):
            msg = parse_frame(frame)
            print(
                f"[{msg['timestamp']}] vx={msg['real_vx']:.3f} m/s  "
                f"wz={msg['real_wz']:.3f} rad/s  "
                f"q={msg['q']}  "
                f"a=({msg['ax']:.2f},{msg['ay']:.2f},{msg['az']:.2f}) m/s²"
            )
            # append to CSV
            with log_file.open("a", encoding="utf-8") as f:
                f.write(
                    ",".join(
                        [
                            msg["timestamp"],
                            f"{msg['real_vx']}",
                            f"{msg['real_wz']}",
                            *map(str, msg["q"]),
                            f"{msg['ax']}",
                            f"{msg['ay']}",
                            f"{msg['az']}",
                        ]
                    )
                    + "\n"
                )
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
