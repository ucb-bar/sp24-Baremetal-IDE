"""
Test plugin for a benchmark that performs a simple "Hello World" ping to the
chip.
"""
import logging
import random

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here...


# Define the tests here:
ShmooTestHarness.register_test_suite("hello", TestSuite(
    "build/dsp24-bmarks/example-bmark/example-bmark.elf",
    ShmooConstantTest("Hello World Test", 0x1,
                      b'Hello, Chip!', b'Hello World!')
))

# Exports (if necessary)
__all__ = []