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

    def __init__(self, name, harts, runtime, *args, timeout=5, **kwargs):
        super().__init__(name, 0x0, *args, timeout=timeout, **kwargs)
        self.runtime = runtime
        self.harts = harts


    def create_payload(self):
        runtime_bytes = self.runtime.to_bytes(4, byteorder='little')
        harts_bytes = self.harts.to_bytes(1, byteorder='little')
        return runtime_bytes + harts_bytes, {}
    
    def check_output(self, context, value):
        return True, f'{self.harts} cores at {self.runtime} ms'


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("saturn_pvirus_1h",
    "build/dsp24-bmarks/saturn-pvirus/saturn-pvirus.elf",
    TimedSaturnPVirusTest("Saturn Power Virus", harts=1, runtime=2000),
))

ShmooTestHarness.register_test_suite(TestSuite("saturn_pvirus_2h",
    "build/dsp24-bmarks/saturn-pvirus/saturn-pvirus.elf",
    TimedSaturnPVirusTest("Saturn Power Virus", harts=2, runtime=2000),
))

ShmooTestHarness.register_test_suite(TestSuite("saturn_pvirus_3h",
    "build/dsp24-bmarks/saturn-pvirus/saturn-pvirus.elf",
    TimedSaturnPVirusTest("Saturn Power Virus", harts=3, runtime=2000),
))

ShmooTestHarness.register_test_suite(TestSuite("saturn_pvirus_4h",
    "build/dsp24-bmarks/saturn-pvirus/saturn-pvirus.elf",
    TimedSaturnPVirusTest("Saturn Power Virus (4 cores)", harts=4, runtime=2000),
))

# Exports (if necessary)
__all__ = []