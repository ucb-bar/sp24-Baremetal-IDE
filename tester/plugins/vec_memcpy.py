"""
Test plugin for a benchmark that performs a vector memcpy.
"""
import logging
import random
import struct

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class BufferTest(ShmooTest):

    def create_payload(self):
        seed = random.randbytes(4)
        return seed, {'seed': seed}
    
    def check_output(self, context, value):
        cycles_numeric, passed = struct.unpack('<Q?', value)
        ShmooTestHarness.log_as_misc(f'{cycles_numeric} cycles, {passed}')
        return passed == 1, f'{cycles_numeric} cycles'


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("memcpy",
    "build/dsp24-bmarks/bandwidth-bmarks/membw-bmark.elf",
    BufferTest("CPU memcpy", 0x0),
    BufferTest("GCC memcpy", 0x1),
    BufferTest("RVV memcpy", 0x2),
    BufferTest("DMA memcpy", 0x3),
))

# Exports (if necessary)
__all__ = []
