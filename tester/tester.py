#!/usr/bin/env python3
"""
Shmoo Test Environment Host by Jasmine Angle
"""

import argparse
import logging
from typing import *

import numpy as np
import colorama

from utils import *
from plugins import *


# Logging Setup
LOGGER = logging.getLogger(__name__)


if __name__ == '__main__':
    colorama.init()

    # Called directly, default to CLI.
    parser = argparse.ArgumentParser(
                    prog='Jasmine\'s ShmooTester',
                    description='Performs Shmoo testing using the default BEL/ETB Bringup communication protocol.',
                    epilog='Created by Jasmine Angle (angle@berkeley.edu)')
    
    parser.add_argument('-i', '--input', help='Path to an existing Shmoo test run to import for Shmoo plot generation. If specified, this will only generate a Shmoo plot from existing data and not run any tests.')
    parser.add_argument('-o', '--output', type=str, default=None, help='Path for storing output files. If the path does not already exist, it will be created. If unspecified, a default path containing the suite name and test start timestamp will be created.')

    args_test = parser.add_argument_group('Shmoo Testing Options')
    args_test.add_argument('-c', '--chip', default='dsp24', help='Chip to run the test on. This is used to locate a configuration file with the path `./platform/chipname/chipname.cfg`. Defaults to `dsp24`.')
    args_test.add_argument('-s', '--suite', help='Name of the test suite to run')
    args_test.add_argument('-t', '--test', type=int, dest='tests', action='append', help='ID of the test you wish to run (as a decimal number). This argument can be passed multiple times to run multiple specific tests within a test suite. If unspecified, all tests will run.')
    args_test.add_argument('--min-v', dest='min_v', type=float, default='0.85', help='Voltage lower bound')
    args_test.add_argument('--max-v', dest='max_v', type=float, default='0.86', help='Voltage upper bound (exclusive)')
    args_test.add_argument('--step-v', dest='step_v', type=float, default='0.05', help='Voltage step size')
    args_test.add_argument('--min-freq', dest='min_freq', type=int, default=100, help='Frequency lower bound (in MHz)')
    args_test.add_argument('--max-freq', dest='max_freq', type=int, default=151, help='Frequency upper bound (in MHz, exclusive)')
    args_test.add_argument('--step-freq', dest='step_freq', type=int, default=50, help='Frequency step size (in MHz)')
    args_test.add_argument("--max-v-fail", dest="max_v_fail", type=int, default=1,
        help="Maximum number of consecutive failures until the tester changes to a new voltage."
    )
    args_test.add_argument("--retries", dest="retries", type=int, default=0,
        help="Maximum number of retries permitted for a single frequency test at a given voltage. Retries are disabled (0) by default."
    )
    args_test.add_argument("-r", "--num-runs", type=int, default=1,
        help="Number of times to re-run the test for a given attempt. Only the final test run will have data captured. Any fail within these runs will fall back to the specified `attempts` count. Defaults to 1."
    )
    args_test.add_argument('-f', '--force', action='store_true', default=False, help='Force start without a confirmation of limits. Only use this setting if you are absolutely sure that the testbench setup is correct and you are aware of the assigned limits.')
    
    args_psu = parser.add_argument_group('PSU Configuration Options')
    args_psu.add_argument('--psu-mode',
        choices=["INT", "EXT"],
        default="INT",
        help='Mode to set the PSU to sense with. `EXT` for 4-wire remote sense, `INT` for 2-wire local sense. Defaults to INT.'
    )
    args_psu.add_argument('--psu-channel', type=int, default=1,
        help='Channel to use for the PSU (1-3). Defaults to 1.'
    )
    args_psu.add_argument('--no-psu', action='store_true', default=None, help='Disables sending any commands to the PSU and instead redirects all SCPI commands to the log.')
    
    args_dbg = parser.add_argument_group('Debugging Options')
    args_dbg.add_argument('-m', '--mock-test', action='store_true', help='Only mock run the host payload creation function of a test suite')
    args_dbg.add_argument('-d', '--debug', dest='debug', action='store_true', help='Enables debugging mode, disabling all serial timeouts and allowing the user to run through a Shmoo test step-by-step.')
    args_dbg.add_argument('-n', '--no-upload', action='store_true', help='Disables OpenOCD reset and program of the chip. Useful for debugging with an external OpenOCD+GDB configuration.')
    
    args_dbg.add_argument('-l', '--list-suites', action='store_true', help='Ignore all other commands and print a list of test suites')
    
    args_log = parser.add_argument_group('Logging Options')
    args_log.add_argument("--log", dest="log_level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="INFO",
        help="Set the logging level"
    )
    args_log.add_argument('--logfile', type=str, default=None, help='Path to an optional log file for terminal capture. If this setting is used, STDOUT will only be used for user prompts.')

    args = parser.parse_args()
    logging.basicConfig(filename=args.logfile, level=args.log_level)
    
    if args.list_suites:
        ShmooTestHarness.list_suites()
        exit()

    if args.input:
        # Shmoo plot only.
        results = ShmooSuiteResults.load_from_run(args.input)
        ShmooTestHarness.make_shmoo_plot(results)
        exit()

    if not args.suite or len(args.suite) == 0:
        parser.print_help()
        exit()
    
    if args.suite not in ShmooTestHarness.TEST_SUITES:
        raise Exception(f'Test suite with name "{args.suite}" does not exist.')
    
    voltages = np.arange(args.min_v, args.max_v, args.step_v)
    frequencies = np.arange(args.min_freq, args.max_freq, args.step_freq)
    
    if args.mock_test:
        ShmooTestHarness.test_run_suite(args.suite, voltages, frequencies)
    else:
        psu_mode = PSUSourceMode[args.psu_mode]
        if not args.force and \
            not ShmooTestHarness.terminal_confirm_params(
                voltages, frequencies, psu_mode, args.no_psu, args.psu_channel,
                args.debug):
            exit()
        ShmooTestHarness.CHIP_NAME = args.chip
        results = ShmooTestHarness.run_suite(args.suite, args.tests,
                                   voltages, frequencies,
                                   max_consec_voltage_fails=args.max_v_fail,
                                   freq_retries=args.retries,
                                   psu_mode=psu_mode,
                                   psu_channel=args.psu_channel,
                                   psu_dummy=args.no_psu,
                                   debug=args.debug,
                                   no_upload=args.no_upload,
                                   output_path=args.output,
                                   test_runs=args.num_runs)

