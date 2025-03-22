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

#from openocd import OpenOcdTclRpc

# from pyftdi.ftdi import Ftdi
# from pyftdi.gpio import *
# from time import sleep

# Logging Setup
LOGGER = logging.getLogger(__name__)


if __name__ == '__main__':
    colorama.init()

    # Called directly, default to CLI.
    parser = argparse.ArgumentParser(
                    prog='Jasmine\'s SMU ShmooTester',
                    description='Performs Shmoo testing using the default Bringup communication protocol',
                    epilog='Created by Jasmine Angle (angle@berkeley.edu)')
    parser.add_argument('-s', '--suite', help='Name of the test suite to run')
    parser.add_argument('-d', '--debug', action='store_true', help='Enables debugging mode to only mock run the test sending portion of a test suite')
    parser.add_argument('-l', '--list-suites', action='store_true', help='Ignore all other commands and print a list of test suites')
    
    parser.add_argument('-i', '--input', help='Path to an existing Shmoo test run to import for Shmoo plot generation. If specified, this will only generate a Shmoo plot from existing data and not run any tests.')

    parser.add_argument('--min-v', dest='min_v', type=float, default='0.85', help='Voltage lower bound')
    parser.add_argument('--max-v', dest='max_v', type=float, default='0.86', help='Voltage upper bound (exclusive)')
    parser.add_argument('--step-v', dest='step_v', type=float, default='0.05', help='Voltage step size')
    parser.add_argument('--min-freq', dest='min_freq', type=int, default=100, help='Frequency lower bound (in MHz)')
    parser.add_argument('--max-freq', dest='max_freq', type=int, default=151, help='Frequency upper bound (in MHz, exclusive)')
    parser.add_argument('--step-freq', dest='step_freq', type=int, default=50, help='Frequency step size (in MHz)')
    parser.add_argument("--max-cmul-fail", dest="max_cmul_fail", type=int, default=1,
        help="Maximum number of cumulative failures until the tester changes to a new voltage."
    )
    parser.add_argument('-f', '--force', action='store_true', default=None, help='Force start without a confirmation of limits.')
    parser.add_argument('--psu-mode',
        choices=["INT", "EXT"],
        default="INT",
        help='Mode to set the PSU to sense with. `remote` for 4-wire remote sense, `local` for 2-wire local sense, `none` for no PSU support.'
    )
    parser.add_argument('--psu-channel', type=int, default=1,
        help='Channel to use for the PSU.'
    )
    parser.add_argument('--no-psu', action='store_true', default=None, help='Disables sending any commands to the PSU and instead redirects all SCPI commands to the log.')
    parser.add_argument("--log", dest="log_level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="INFO",
        help="Set the logging level"
    )
    parser.add_argument('--logfile', type=str, default=None, help='Path to an optional log file for terminal capture. If this setting is used, STDOUT will only be used for user prompts.')

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
    
    if args.debug:
        ShmooTestHarness.test_run_suite(args.suite, voltages, frequencies)
    else:
        psu_mode = PSUSourceMode[args.psu_mode]
        if not args.force and \
            not ShmooTestHarness.terminal_confirm_params(
                voltages, frequencies, psu_mode, args.no_psu, args.psu_channel):
            exit()
        results = ShmooTestHarness.run_suite(args.suite, voltages, frequencies,
                                   max_cmul_freq_fails=args.max_cmul_fail,
                                   psu_mode=psu_mode,
                                   psu_channel=args.psu_channel,
                                   psu_dummy=args.no_psu)

