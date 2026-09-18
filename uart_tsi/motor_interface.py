import struct
import time
import serial
from tqdm import tqdm

from uart_tsi import *


BASE = 0x10080000

NUM_MOTORS = 8
OFFSET = 0x100


TARGET_POS = 0x0
TARGET_VEL = 0x4
STATE = 0x8
ENABLE = 0xC
DIR = 0x10
PRESC = 0x14 
    
ENC_POS = 0x20
ENC_VEL = 0x28
ENC_RST = 0x30

# Convert to twos complement
def twos_complement(hexstr, bits):
    value = int(hexstr, 16)
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

# Read from joint data
def readJoint(ser, addr, num):
    return readWord(ser, BASE + addr + (num * OFFSET))

# Write from joint data
def writeJoint(ser, num, addr, value):
    writeWord(ser, BASE + addr + (num * OFFSET), value)

# Set the motor speed and prescaler
def set_motor_speed(ser, num, value):
    writeJoint(ser, num, STATE, 0)         
    writeJoint(ser, num, PRESC, 100)
    writeJoint(ser, num, TARGET_VEL, value)
    if value == 0:
        writeJoint(ser, num, ENABLE, 0)
    else:
        writeJoint(ser, num, ENABLE, 1)

# Set target motor position
def set_motor_pos(ser, num, value):
    writeJoint(ser, num, STATE, 1)
    writeJoint(ser, num, TARGET_POS, value)
    writeJoint(ser, num, PRESC, 100)
    writeJoint(ser, num, ENABLE, 1)

# Get the overal status of a motor
def get_motor_status(ser, num):
    pos_target = twos_complement(readJoint(ser, TARGET_POS, num), 32)
    vel_target = twos_complement(readJoint(ser, TARGET_VEL, num), 32)
    mode = twos_complement(readJoint(ser, STATE, num), 32)
    enable = twos_complement(readJoint(ser, ENABLE, num), 32)
    presc = twos_complement(readJoint(ser, PRESC, num), 32)

    print(f"motor number:{num}")
    print(f"pos:{pos_target}, vel:{vel_target}")
    print(f"mode:{mode}, en:{enable}, presc:{presc}\n")