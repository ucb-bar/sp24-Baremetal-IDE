"""
Test plugin for a benchmark that performs a saturn benchmark.
"""
import logging
import random
import struct

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class BufferTest(ShmooTest):

    def __init__(self, name, runtime, *args, timeout=45, **kwargs):
        super().__init__(name, runtime, *args, timeout=timeout, **kwargs)
        self.runtime = runtime


    def create_payload(self):
        seed = random.randbytes(4)
        return seed, {'seed': seed}

    def check_output(self, context, value):
        naive_cycles, naive_perf, vector_cycles, vector_perf = struct.unpack('<QfQf', value)
        log_msg = (f'Naive: {naive_cycles} cycles, {naive_perf:.2f} perf | '
                f'Vector: {vector_cycles} cycles, {vector_perf:.2f} perf')
        ShmooTestHarness.log_as_misc(log_msg)

        if naive_cycles > 0 and vector_cycles > 0: 
            passed = (vector_cycles < naive_cycles) and (vector_perf >= naive_perf)
        else: 
            passed = 1
            
        return passed, {
            "naive_cycles": naive_cycles,
            "naive_perf": naive_perf,
            "vector_cycles": vector_cycles,
            "vector_perf": vector_perf
        }



# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("saturn_bmarks_shmoo",
    "build/dsp24-bmarks/saturn-bmarks-shmoo/saturn-bmarks-shmoo.elf",
    BufferTest("naive_igemm", 0x0),
    BufferTest("vec_igemm", 0x1),
    BufferTest("compare", 0x2),
))


# Exports (if necessary)
__all__ = []
