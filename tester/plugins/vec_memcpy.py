"""
Test plugin for a benchmark that performs a vector memcpy.
"""
import logging
import random

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class BufferTest(ShmooTest):

    def create_payload(self):
        seed = random.randbytes(4)
        return seed, {'seed': seed}
    
    def check_output(self, context, value):
        val = value[:7].hex()
        ShmooTestHarness.log_as_misc(val)
        return value[8] == 1, val


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("memcpy",
    "build/dsp24-bmarks/bandwidth-bmarks/membw-bmark.elf",
    BufferTest("Memcpy", 0x2),
))

# Exports (if necessary)
__all__ = []