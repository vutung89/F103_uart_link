# Get-PnpDevice -Class Ports
# pip install pyserial

import serial
import time

PROTO_HEADER = 0xAA
CMD_SET_PID = 0x01
CMD_GET_STATUS = 0x02
CMD_HEARTBEAT = 0x03

COM_PORT = "COM7"
BAUDRATE = 115200

TIME_DELAY = 0.1 #seconds

def crc8_update(crc: int, byte: int) -> int:
    crc ^= byte

    for _ in range(8):
        if crc & 0x01:
            crc = (crc >> 1) ^ 0x8C
        else:
            crc >>= 1

    return crc & 0xFF


def crc8(data: bytes) -> int:
    crc = 0

    for b in data:
        crc = crc8_update(crc, b)

    return crc


def build_frame(payload: bytes) -> bytes:
    length = len(payload)

    if length == 0 or length > 16:
        raise ValueError("Payload length must be 1..16 bytes")

    crc = crc8(bytes([length]) + payload)

    return bytes([
        PROTO_HEADER,
        length
    ]) + payload + bytes([crc])


def send_frame(ser: serial.Serial, payload: bytes):
    frame = build_frame(payload)

    print(
        "TX:",
        " ".join(f"{b:02X}" for b in frame)
    )

    ser.write(frame)

def set_pid(ser: serial.Serial, kp: float, ki: float, kd: float):
    # Convert PID gains to raw integer values (scaled by 100)
    # Range: 0.00 to 655.35
    raw_kp = int(kp * 100)
    raw_ki = int(ki * 100)
    raw_kd = int(kd * 100)

    payload = bytes([
        CMD_SET_PID,
        (raw_kp >> 8) & 0xFF, raw_kp & 0xFF,
        (raw_ki >> 8) & 0xFF, raw_ki & 0xFF,
        (raw_kd >> 8) & 0xFF, raw_kd & 0xFF
    ])

    send_frame(ser, payload)

def rx_frame_process(data: bytes):
    data = data[2:-1] # skip header + length and crc
    cmd_id = data[0]
    if cmd_id == CMD_SET_PID:
        print("Received CMD_SET_PID")
        print("Payload:", " ".join(f"{b:02X}" for b in data[1:]))
        raw_kp = (data[1] << 8) | data[2]
        raw_ki = (data[3] << 8) | data[4]
        raw_kd = (data[5] << 8) | data[6]
        kp = raw_kp / 100
        ki = raw_ki / 100
        kd = raw_kd / 100
        print(f"Parsed PID: Kp={kp}, Ki={ki}, Kd={kd}")

def main():

    try:

        ser = serial.Serial(
        port=COM_PORT,      # sửa lại cho phù hợp
        baudrate=BAUDRATE,
        timeout=0.5
        )

        print("UART Connected")

        

        while True:
            # send_frame(ser, bytes([CMD_HEARTBEAT]))
            set_pid(ser, kp=200.0, ki=0.085, kd=0.0)
            # đọc phản hồi nếu có
            rx = ser.read(64)

            if len(rx) > 0:
                print(
                    "RX:",
                    " ".join(f"{b:02X}" for b in rx)
                )
                rx_frame_process(rx)
            
            print('-' * 30)

            time.sleep(TIME_DELAY)

    except KeyboardInterrupt:
        print("Exit")

    finally:
        ser.close()
        pass


if __name__ == "__main__":
    main()