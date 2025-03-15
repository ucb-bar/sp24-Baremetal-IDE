"""
Test plugin for a benchmark that performs a simple "Hello World" ping to the
chip.
"""
import logging
import random

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:

class HelloTest(ShmooTest):

    def create_payload(self):
        return b'Hey, Chip!', {}
    
    def check_output(self, context, value):
        return value == b'Hello World!', value


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("hello",
    "build/dsp24-bmarks/example-bmark/example-bmark.elf",

    # ShmooConstantTest can be used for a deterministic send/receive.
    ShmooConstantTest("Hello World Test", 0x1,
                      to_chip=b'Hello, Chip!',
                      expect=b'Hello World!'),

    # Perhaps this test happens to take longer, thus it needs a 10 sec timeout.
    # HelloTest("The Same Hello World Test", 0x2, timeout=10),
))

# Exports (if necessary)
__all__ = []