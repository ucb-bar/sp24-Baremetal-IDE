"""
Test plugin for a benchmark that performs convolutions.
"""
import logging
import random
import struct

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class ConvTest(ShmooTest):

    def create_payload(self):
        seed = random.randbytes(4)
        return seed, {'seed': seed}
    
    def check_output(self, context, value):
        cycles_numeric, passed = struct.unpack('<Q?', value)
        ShmooTestHarness.log_as_misc(f'{cycles_numeric} cycles, {passed}')
        return passed == 1, f'{cycles_numeric} cycles'


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("conv-bmark",
    "build/dsp24-bmarks/conv-bmarks/conv-bmark.elf",
    # ConvTest("CPU FP16 Convolution", 0x0),
    ConvTest("Conv Accelerator FP16 Convolution", 0x2, timeout=50),
    ConvTest("Conv CPU FP32 Convolution", 0x3, timeout=20),
))

# Exports (if necessary)
__all__ = []
