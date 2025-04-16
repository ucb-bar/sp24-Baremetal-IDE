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
    
    
class TimedHelloWorldTest(ShmooTest):

    def __init__(self, name, runtime, *args, timeout=5, **kwargs):
        super().__init__(name, 0x0, *args, timeout=timeout, **kwargs)
        self.runtime = runtime

    def create_payload(self):
        ShmooTestHarness.log_as_misc(f'Converted {self.runtime} to bytes')
        runtime_bytes = self.runtime.to_bytes(4, byteorder='little')
        #return b'\xde\xad\xbe\xef', {}
        return runtime_bytes, {}
    
    def check_output(self, context, value):
        return value == b'Hello World!', value


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("hello_timed",
    "build/dsp24-bmarks/example-bmark-timed/example-bmark-timed.elf",

    # ShmooConstantTest can be used for a deterministic send/receive.
    # ShmooConstantTest("Hello World Test", 0x1,
    #                   to_chip=b'Hello, Chip!',
    #                   expect=b'Hello World!'),

    # Perhaps this test happens to take longer, thus it needs a 10 sec timeout.
    TimedHelloWorldTest("2 Sec. Hello World Test", runtime=2000),
))

# Exports (if necessary)
__all__ = []