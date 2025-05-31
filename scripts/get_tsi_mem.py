import struct
import time
import serial
from tqdm import tqdm

from uart_tsi import *
from motor_interface import *

port = "/dev/tty.usbserial-0ABC011"
# port = "/dev/tty.usbserial-101"
# baudrate = 921600
baudrate = 115200

ser = serial.Serial(port=port, baudrate=baudrate)


def main():
    SCRATCH =   0x10020000
    SCRATCH2 =  0x10080000
    FLASH =     0x20000000
    DRAM =      0x80000000


    print("start")

    print("\nTest SCRATCH")
    writeWord(ser, 0x10080000, 0xdeadbeef)
    print(readWord(ser, 0x10080000))

    

    # print("\nTest DRAM")
    # writeWord(ser, DRAM, 0xdeadbeef)
    # print(readWord(ser, DRAM))




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
    main()


   