"""
Test plugin for a Power Virus benchmark that performs a 1D Convolution.
"""
import logging
import random

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class TimedPVirusTest(ShmooTest):

    def __init__(self, name, runtime, *args, timeout=5, **kwargs):
        super().__init__(name, 0x0, *args, timeout=timeout, **kwargs)
        self.runtime = runtime

    def create_payload(self):
        runtime_bytes = self.runtime.to_bytes(8, byteorder='little')
        return runtime_bytes, {}
    
    def check_output(self, context, value):
        return True, f'{self.runtime} ms'


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("conv_pvirus",
    "build/dsp24-bmarks/conv-pvirus/conv-pvirus.elf",
    TimedPVirusTest("1D Convolution Power Virus", runtime=2000),
))

# Exports (if necessary)
__all__ = []