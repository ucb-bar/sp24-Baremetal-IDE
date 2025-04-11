"""
Test plugin for a benchmark that performs an FFT comparison.
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
ShmooTestHarness.register_test_suite(TestSuite("app-fft-bmark",
    "build/dsp24-bmarks/app-fft-bmark/app-fft-bmark.elf",
    BufferTest("run_fft_dma_bmark_sequence", 0x0, timeout=25),
    BufferTest("run_cpu_fft_bmark_sequence", 0x1, timeout=80),
))

# Exports (if necessary)
__all__ = []
