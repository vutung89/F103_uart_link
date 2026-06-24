"""
    @file test_rc_control.py
    @brief Thử nghiệm gửi lệnh RC qua cổng serial bất đồng bộ.
    @author VuTung
    @date 2025-07-29
    @version 2.0
    @details:
    - Mục đích của tập tin này là để kiểm tra việc gửi 
    lệnh RC và nhận phản hồi từ bộ điều khiển RC thông qua cổng serial.
    - Sử dụng thư viện serial_asyncio để thực hiện các thao tác bất đồng 
    bộ với cổng serial.
    @usage:
    - Chạy tập tin này để kết nối tới cổng RC, gửi lệnh và nhận dữ liệu RC channels.
    - Có thể điều chỉnh các giá trị RC (pitch, roll, throttle, yaw) trong 
    hàm main để kiểm tra các lệnh khác nhau
    - Có thể thay đổi TIME_DELAY để điều chỉnh thời gian chờ giữa các lệnh.
    - Nhấn Ctrl+C để dừng chương trình.
    @Mode RC_control:
    - Tay điều khiển SIYI MK15
        1. Chế độ Auto RC_control pitch, roll, throttle, yaw
            - Đồng thời: Gạt Switch bên phải sang nấc dưới cùng + nhấn Button A sáng đèn
        2. Chế độ Manual RC_control pitch, roll, throttle, yaw
            - Một trong hai, hoặc cả hai: Gạt Switch bên phải sang nấc giữa/ trên cùng / nhấn Button A tắt đèn
        3. Chế độ Ngắt gửi Sbus_out vào Px4
            - Nhấn Button B sáng đèn
        4. Chế độ gửi Sbus_out vào Px4
            - Nhấn Button B tắt đèn 
    @note:
    - Đảm bảo rằng cổng serial được kết nối đúng và thiết bị đã sẵn sàng nhận lệnh.
    - Lưu ý rằng cổng serial và baudrate cần được cấu hình đúng với thiết bị của bạn
    - Thời gian chờ giữa các lệnh có thể điều chỉnh thông qua biến TIME_DELAY.
    - Nếu gặp lỗi kết nối, hãy kiểm tra lại cổng serial và thiết bị.
    - Đảm bảo rằng thư viện crc đã được cài đặt, mã crc của máy tính nhúng và mạch giống nhau để tính toán CRC16.
"""
# -----------------------------------------------------------------------------------------------------
import asyncio
import serial_asyncio
import logging
from datetime import datetime
import random

# ------------------------------------------------------------------------------------------------------
# PARAMETERS
# ------------------------------------------------------------------------------------------------------


import asyncio
import serial_asyncio
import logging
import random

# =============================================================================
# CONFIG
# =============================================================================

PORT = "COM7"
BAUDRATE = 115200

PROTO_HEADER = 0xAA
PAYLOAD_MAX_SIZE = 32

TIME_DELAY = 0.0005

CMD_ECHO_PAYLOAD = 0x04

#------------------------------------------------------------------------------------------------------

# =============================================================================
# LOG
# =============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)

# ------------------------------------------------------------------------------------------------------
# CRC16 Calculation
# ------------------------------------------------------------------------------------------------------
# Lookup tables for the high and low byte of the CRC
aucCRCHi = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
]

aucCRCLo = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
]

def calculate_crc16(data):
    ucCRCHi = 0xFF
    ucCRCLo = 0xFF
    
    for byte in data:
        iIndex = ucCRCLo ^ byte
        ucCRCLo = ucCRCHi ^ aucCRCHi[iIndex]
        ucCRCHi = aucCRCLo[iIndex]
    
    return (ucCRCHi << 8) | ucCRCLo
# ------------------------------------------------------------------------------------------------------
# MACH GHI DE RC
# ------------------------------------------------------------------------------------------------------
"""
test_async_command.py

Protocol:

HEADER(1)
LEN(1)
PAYLOAD(N)
CRC16_LO
CRC16_HI

CRC16 tính trên:
LEN + PAYLOAD
"""

# =============================================================================
# PROTOCOL
# =============================================================================


def build_frame(payload: bytes) -> bytes:

    if len(payload) == 0:
        raise ValueError("payload empty")

    if len(payload) > PAYLOAD_MAX_SIZE:
        raise ValueError("payload too large")

    frame = bytearray()

    frame.append(PROTO_HEADER)
    frame.append(len(payload))
    frame.extend(payload)

    crc = calculate_crc16(
        bytes([PROTO_HEADER, len(payload)]) + payload
    )

    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)

    return bytes(frame)


def verify_frame(frame: bytes) -> bool:

    if len(frame) < 4:
        return False

    if frame[0] != PROTO_HEADER:
        return False

    length = frame[1]

    if len(frame) != length + 4:
        return False

    payload = frame[2:2 + length]

    crc_recv = (
        frame[2 + length]
        |
        (frame[3 + length] << 8)
    )

    crc_calc = calculate_crc16(
        bytes([PROTO_HEADER, length]) + payload
    )

    if crc_recv != crc_calc:
        return False

    return True


# =============================================================================
# SERIAL LINK
# =============================================================================

class AsyncCommandLink:

    def __init__(self, port, baudrate):

        self.port = port
        self.baudrate = baudrate

        self.reader = None
        self.writer = None

    async def connect(self):

        self.reader, self.writer = \
            await serial_asyncio.open_serial_connection(
                url=self.port,
                baudrate=self.baudrate
            )

        logging.info(
            f"Connected {self.port} @ {self.baudrate}"
        )

    async def close(self):

        if self.writer:
            self.writer.close()

        await asyncio.sleep(0.2)

    # -------------------------------------------------------------------------
    # Low level
    # -------------------------------------------------------------------------

    async def send_frame(
        self,
        payload: bytes
    ):

        frame = build_frame(payload)

        self.writer.write(frame)

        await self.writer.drain()

    async def receive_frame(self):

        while True:

            b = await self.reader.readexactly(1)

            if b[0] == PROTO_HEADER:
                break

        length = (
            await self.reader.readexactly(1)
        )[0]

        if length == 0:
            raise RuntimeError("invalid length")

        remain = await self.reader.readexactly(
            length + 2
        )

        frame = bytes([
            PROTO_HEADER,
            length
        ]) + remain

        if verify_frame(frame) is False:
            raise RuntimeError(
                "CRC verify failed"
            )

        return frame

    # -------------------------------------------------------------------------
    # Mid level
    # -------------------------------------------------------------------------

    async def send_command(
        self,
        payload: bytes,
        timeout=1.0
    ):

        await self.send_frame(payload)

        echo = await asyncio.wait_for(
            self.receive_frame(),
            timeout
        )

        if echo != payload:
            raise RuntimeError(
                "Echo mismatch"
            )

    async def request(
        self,
        payload: bytes,
        timeout=1.0
    ):

        await self.send_frame(payload)

        response = await asyncio.wait_for(
            self.receive_frame(),
            timeout
        )

        return response
    
    async def echo_payload(self, payload: bytes, timeout=1.0):
        await self.send_frame(payload)
        frame = await asyncio.wait_for(
            self.receive_frame(),
            timeout
        )
        len_payload = frame[1]
        echo_payload = frame[2:(2+len_payload)]
        print(echo_payload)
        if (echo_payload != payload):
            print("echo payload Fail")


# =============================================================================
# MAIN
# =============================================================================

async def main():

    link = AsyncCommandLink(
        PORT,
        BAUDRATE
    )

    await link.connect()

    try:
        payload = bytes([CMD_ECHO_PAYLOAD, 0xaa, 0xfd, 0xff, 0x00])
        await link.echo_payload(payload, timeout=1)


        for i in range(100):
            print(f"num {i}")
            await link.echo_payload(payload, timeout=1)

            await asyncio.sleep(
                TIME_DELAY
            )

    finally:

        await link.close()


if __name__ == "__main__":

    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        logging.info("Exit")