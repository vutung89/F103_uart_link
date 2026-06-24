import serial

PORT = "COM6"
BAUDRATE = 115200

tx_buf = bytes([
    0xAA,
    0x55,
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x06,
    0x07,
    0x08
])

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

ser.write(tx_buf)
ser.flush()

print("TX:", tx_buf.hex(' '))

rx_buf = ser.read(10)

print("RX:", rx_buf.hex(' '))

ser.close()