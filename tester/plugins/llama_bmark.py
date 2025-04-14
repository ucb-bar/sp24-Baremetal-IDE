"""
Test plugin for a benchmark that performs Llama2 inference.
"""
import logging
import struct

from utils import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


# Add any new ShmooTest subclasses with custom behavior here:
class LlamaTest(ShmooTest):

    def __init__(self, name, id, steps, *args, timeout=5, **kwargs):
        super().__init__(name, id, *args, timeout=timeout, **kwargs)
        self.steps = steps

    def create_payload(self):
        steps_bytes = self.steps.to_bytes(4, byteorder='little')
        return steps_bytes, {'steps': self.steps}
    
    def check_output(self, context, value):
        # float tok_per_s = (pos-1) / (((double)(end-start))/target_frequency);
        chip_freq = context['chip_freq']
        cycles, steps = struct.unpack('<QI', value)
        tok_per_sec = steps / (cycles / chip_freq)
        stat_str = f'{steps} steps in {cycles} cycles [{tok_per_sec} tok/sec]'
        ShmooTestHarness.log_as_misc(f'Llama: {stat_str}')
        return True, stat_str


# Define the tests here:
ShmooTestHarness.register_test_suite(TestSuite("llama_int8",
    "build/dsp24-bmarks/borai-int8-bmarks/boraiq_bmark.elf",
    LlamaTest("Int8 Llama2 Inference - 256 Steps", 0x0, steps=256, timeout=65),
    LlamaTest("Int8 Llama2 Inference - 8 Steps", 0x1, steps=8, timeout=1),
    LlamaTest("Int8 Llama2 Inference - 16 Steps", 0x2, steps=16, timeout=1),
))

# Exports (if necessary)
__all__ = []
