import struct
import time
import serial
from tqdm import tqdm

from uart_tsi import *
from motor_interface import *

port = "/dev/tty.usbserial-0ABC011"
# port = "/dev/tty.usbserial-101"
baudrate = 921600
# baudrate = 115200

ser = serial.Serial(port=port, baudrate=baudrate)

def test_dram():
    logfile = open("logs/dram.log", "w")

    print("\nTest full DRAM")
    for i in tqdm(range(0x80000000, 0x8FFFFFFF, 4)):
    # for i in range(0x80000000, 0x80001000, 4):
        initial = readWord(i)
        writeWord(i, i)
        final = readWord(i)
        if final == hex(i):
            # print(f"DRAM address 0x{hex(i)} Initial:[{initial}], read:[{final}] [SUCCESS]")
            logfile.write(f"DRAM address 0x{hex(i)} Initial:[{initial}], read:[{final}] [SUCCESS]\n")
        else:
            # print(f"DRAM address 0x{hex(i)} Initial:[{initial}], read:[{final}] [FAIL]")
            logfile.write(f"DRAM address 0x{hex(i)} Initial:[{initial}], read:[{final}] [FAIL]\n")

    print("\nDRAM Test completed")


if __name__ == "__main__":

    print("start")

    print("\nTest DRAM")
    ADDR = 0x8000000
    # print(readWord(ser, ADDR))
    writeWord(ser, 0x8000000, 0xaaaaaaaa)
    print(readWord(ser, 0x8000000))
    # print(readWord(ser, 0x10080000))

    MOTORNUM = 0
    # 5, 4, 3
    
    # for i in range(NUM_MOTORS):
    #     set_motor_speed(ser, i, 0)
    # writeJoint(ser, MOTORNUM, ENABLE, 1)
    # writeJoint(ser, MOTORNUM, STATE, 0)

    # for i in range(1000):
    #     set_motor_pos(ser, MOTORNUM, 0)
    #     time.sleep(2)
    #     set_motor_pos(ser, MOTORNUM, 1000)
    #     time.sleep(2)

    # # Test which reads encoder positions
    # print("Start polling")
    # for i in range(1000):

    #     # set_motor_pos(MOTORNUM, 800)
    #     time.sleep(0.1)

    #     pr = "-\n"
    #     for i in range(NUM_MOTORS):
    #         pr += str(twos_complement(readJoint(ser, ENC_POS, i), 32)) + ", "
    #     # pr += "\n"

    #     # for i in range(NUM_MOTORS):
    #     #     pr += str(twos_complement(readJoint(ser, ENC_VEL, i), 32)) + ", "
    #     print(pr)

    


