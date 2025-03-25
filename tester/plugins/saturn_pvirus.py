"""
Test plugin for a benchmark that performs a vector memcpy.
"""
import logging
import random

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class TimedSaturnPVirusTest(ShmooTest):

    def __init__(self, name, runtime, *args, timeout=5, **kwargs):
        super().__init__(name, 0x0, *args, timeout=timeout, **kwargs)
        self.runtime = runtime

    def create_payload(self):
        runtime_bytes = self.runtime.to_bytes(8, byteorder='little')
        return runtime_bytes, {}
    
    def check_output(self, context, value):
        return True, f'{self.runtime} ms'
    
class MultiCoreSaturnPVirusTest(TimedSaturnPVirusTest):

    def __init__(self, name, harts, runtime, *args, timeout=5, **kwargs):
        super().__init__(name, runtime, *args, timeout=timeout, **kwargs)
        self.harts = harts

    def create_payload(self):
        harts_bytes = self.harts.to_bytes(1, byteorder='little')
        runtime_bytes = self.runtime.to_bytes(8, byteorder='little')
        return harts_bytes + runtime_bytes, {}
    
    def check_output(self, context, value):
        return True, f'{self.harts} cores at {self.runtime} ms'


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("saturn_pvirus",
    "build/dsp24-bmarks/saturn-pvirus/saturn-pvirus.elf",
    TimedSaturnPVirusTest("Saturn Power Virus", runtime=2000),
))

# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("saturn_pvirus_mc",
    "build/dsp24-bmarks/saturn-pvirus-mc/saturn-pvirus-mc.elf",
    MultiCoreSaturnPVirusTest("Saturn Power Virus (4 cores)", harts=4, runtime=2000),
))

# Exports (if necessary)
__all__ = []