import struct
import time
import serial
from tqdm import tqdm


CMD_READ = 0x00
CMD_WRITE = 0x01

def readWord(ser, address):
    command = CMD_READ
    header = struct.pack("<LQQ", command, address, 0)
    ser.write(header)
    # print("write:", header)

    received = ser.read(4)
    rx_data, = struct.unpack("<L", received)
    # for i in range(len(received)):
    #   print("ret:", hex(received[i]))
    return hex(rx_data)

def writeWord(ser, address, data):
    command = CMD_WRITE
    header = struct.pack("<LQQ", command, address, 0)
    payload = struct.pack("<L", data)
    buffer = header + payload
    # print("write:", header)
    
    # for i in range(len(buffer)):
    #   print("write:", hex(buffer[i]))
    ser.write(buffer)