"""
Shmoo testing utilities for use with plugins.
"""
from datetime import datetime, timedelta
import logging
import enum
import os
from pathlib import Path
import shlex
import struct
import subprocess
from time import sleep
from typing import *
from matplotlib import pyplot as plt
import matplotlib
from mpl_toolkits.axes_grid1 import make_axes_locatable
from openocd.tclrpc import TclException
from typing_extensions import Self
from collections import OrderedDict, defaultdict, deque

import pyvisa
import serial  # PySerial
from serial.tools import list_ports
import numpy as np

from openocd import OpenOcdTclRpc
from pyftdi.ftdi import Ftdi, UsbTools
from pyftdi.gpio import *

# I'm too tired to use vt100 commands
from colorama import Fore, Style

LOGGER = logging.getLogger(__name__)

class SerialDebug(serial.Serial):
    def __init__(self, *args, **kwargs):
        self.logger = logging.getLogger(__name__)
        super().__init__(*args, **kwargs)
        self.logger.debug(f"Serial port {self.port} initialized with settings: {self.get_settings()}")

    @staticmethod
    def create(baud_rate, timeout=None) -> Union[Self, None]:
        """
        Generates a new SerialDebug (pySerial) instance for the appropriate
        UART device for the chip using a simple ping/pong procedure.

        Returns:
            Union[Self, NoneType]: An instance of SerialDebug for the chip UART.
        """
        LOGGER.debug('Searching for available UART devices...')
        ports = list_ports.grep('/dev/ttyUSB\d')
        
        for port in ports:
            # Check for FTDI chip and that it is not a jtag port
            LOGGER.debug(f'Found device {port.device} ({port.description}, Mfr: {port.manufacturer}, Loc: {port.location})')
            if port.manufacturer == 'FTDI' and port.location.endswith('1.1'):
                LOGGER.debug(f'Testing for valid handshake.')
                # Ping to find if it is the correct FTDI
                tmp = SerialDebug(port.device, baud_rate, timeout=0.5)
                tmp.write(b'\x05')
                recv = tmp.read()
                if recv == b'\x06':
                    LOGGER.debug(f'Valid handshake received at {port.device}.')
                    tmp.timeout = timeout
                    return tmp
            else:
                LOGGER.debug(f'Not a valid FTDI UART device.')
        return None

    def read(self, size=1):
        data = super().read(size)
        self.logger.debug(f"Serial Received: {data!r}")
        return data

    def write(self, data):
        self.logger.debug(f"Serial Sent: {data!r}")
        return super().write(data)

    def close(self):
        self.logger.debug(f"Closing serial port {self.port}")
        super().close()


class PSUSourceMode(enum.Enum):
    """
    Enumeration class to store the various source modes for the PSU.
    """

    INT = '2-Wire Local Sensing Mode'
    """
    Sets the remote sense relays to local sensing. Use this if you have the
    front remote sense terminals connected to the DUT. 2-wire mode.
    """

    EXT = '4-Wire Remote Sensing Mode (CONFIRM REAR TERMINAL CONNECTION)'
    """
    Sets the remote sense relays to remote sensing. Use this if you have the
    rear remote sense terminals connected to the DUT. 4-wire mode.
    """


class PSU:
    """
    Wrapper class for all PyVisa PSU commands and data acquisition management.
    """

    IDN = 'Keysight Technologies,E36312A,MY57010726,1.0.4-1.0.0-1.04'
    """
    Identification string to check for valid equipment setup.
    """
    
    INIT_CURRENT_LIMIT = "2"
    """
    Defines the default current limit for the SourceMeter.
    """
    
    INIT_VOLTAGE_LIMIT = "0.85"
    """
    Defines the default voltage limit for the SourceMeter.
    """

    VISA_PATH = 'TCPIP::169.254.201.77::lan0::INSTR'
    """
    Defines the default path to use for communication with the SourceMeter.
    """

    DUMMMY_LOG_PREFIX = f'{Fore.LIGHTBLUE_EX}{Style.BRIGHT}[Dummy PSU]{Style.RESET_ALL}'

    def __init__(self, dummy: bool=False):
        """
        PSU Initialization

        Args:
            dummy (bool, optional): Whether to create a fake simulation PSU to
                test the command flow. This will cause all `PSU.write` commands
                to be output to the log rather than over LAN. Any `PSU.query`
                commands will return an empty string. Defaults to False.

        Raises:
            Exception: _description_
        """
        self.dummy = dummy
        if self.dummy:
            self.dummy_log('Dummy PSU has been created.')
        else:
            self.rm = pyvisa.ResourceManager("@py")
            self._psu = self.rm.open_resource(self.VISA_PATH)

        # Verify that we have the correct equipment
        verification = self.query("*IDN?")
        LOGGER.debug(f'Found equipment with IDN: {verification}')

        if not verification.startswith(self.IDN):
            raise Exception(f'Attempt to connect to `{self.IDN}` at {self.VISA_PATH} failed, IDN returned `{verification}` instead.')

        self.display_meter()

    def dummy_log(self, val):
        LOGGER.debug(f'{self.DUMMMY_LOG_PREFIX} {val}')

    def query(self, querystr: str) -> str:
        """
        Sends a query over GPIB to the PSU, expecting a result. This can be a
        multiline script query.

        Args:
            querystr (str): The query to send to the PSU.

        Returns:
            str: Data returned by the PSU.
        """
        if self.dummy:
            self.dummy_log(f'QUERY: {querystr}')

            if querystr.startswith('MEAS'):
                return '0.000000e-00\n'

            if querystr == "*IDN?":
                return self.IDN

            return ""
        else:
            return self._psu.query(querystr)

    def write(self, cmdstr: str) -> str:
        """
        Writes a command over GPIB to the PSU. This can be a multiline script.

        Args:
            cmdstr (str): The command to send to the PSU.

        Returns:
            str: Data returned by the PSU.
        """
        if self.dummy:
            self.dummy_log(f'QUERY: {cmdstr}')
            return ""
        else:
            return self._psu.write(cmdstr)
        
    def enable(self, channel):
        """
        Enables PSU output on a specific channel.
        """
        self.write(f'OUTP ON,(@{channel})')

    def disable(self, channel):
        """
        Disables PSU output on a specific channel.
        """
        self.write(f'OUTP OFF,(@{channel})')
    
    def reset_channel(self, channel: int):
        """
        Resets a channel back to a default voltage/current limit.

        Args:
            channel (int): Channel to reset.
        """
        self.set_limits(channel, self.INIT_VOLTAGE_LIMIT,
                        self.INIT_CURRENT_LIMIT)
        

    def set_mode(self, mode: PSUSourceMode, channel):
        """
        Changes the remote sense relays for a given output channel.

        Args:
            mode (PSUSourceMode): Mode to set based on sensing type.
            channel (int, optional): Channel to modify. Defaults to 1.
        """

        self.write(f'VOLT:SENS:SOUR {mode.name},(@{channel})')
    
    def set_limits(self, channel, voltage: Union[str, int, float] = None,
                   current: Union[str, int, float] = None):
        """
        Assigns limit values to a certain channel on the PSU.

        Args:
            channel (int): Channel number to apply limits to.
            voltage (Union[str, int, float]): Voltage limit (i.e. `0.85`)
            current (Union[str, int, float]): Current limit (i.e. `2`)
        """
        if voltage is None:
            voltage = self.INIT_VOLTAGE_LIMIT
        if current is None:
            current = self.INIT_CURRENT_LIMIT

        if isinstance(voltage, float):
            voltage = float_to_str(voltage)
        if isinstance(current, float):
            current = float_to_str(voltage)

        self.write(f'APPL Ch{channel}, {voltage}, {current}')
        self.write(f'VOLT:PROT 1.5, (@{channel})')

    def display_meter(self):
        self.write(f'DISP:VIEW METER3')


class ShmooTest:
    """
    Base class for a Shmoo Test. This should be extended through other classes
    for varying test functionality.
    """

    def __init__(self, name: str, id: int, *args, timeout=5, **kwargs):
        self.name = name
        if id > 0xff:
            raise Exception(f'Test ID {id} for test {name} exceeds max 8-bit unsigned integer value.')
        self.id = id
        self.timeout = timeout

    def create_payload(self) -> tuple[bytes, dict]:
        """
        Creates a byte array payload to be sent to the chip.

        Returns:
            tuple[bytes, dict]: Tuple containing a byte array of data to send
                to the chip, as well as a context dictionary to maintain info for
                future test output verification. This dictionary will be passed
                into a future `check_output` call.
        """
        return b'Hello, Chip!', {}

    def check_output(self, context: dict, value: bytes) -> tuple[bool, str]:
        """
        Checks the output from the chip against an arbitrary result.
        This function is called after the ETB and chip payload has been sent
        from the chip and the payload size has been verified.
        
        Args:
            context (dict): Dictionary containing test-specific information to
                be maintained for output verification (i.e., a seed or expected
                result).
            value (bytes): The payload returned from the chip.

        Returns:
            tuple[bool, str]: True if the test has "passed" according to the
                test output, False otherwise. Second parameter contains the
                printable string that will be output to the `result.tsv` file.
        """
        return value == "Hey, Host!", value


class ShmooConstantTest(ShmooTest):
    """
    Shmoo test to send a constant value to the chip, checking a for the output
    equaling a constant result. 
    """

    def __init__(self, name: str, id: int, to_chip, expect, *args, **kwargs):
        super().__init__(name, id, *args, **kwargs)
        self.to_chip = to_chip
        self.expect = expect

    def create_payload(self) -> tuple[bytes, dict]:
        return self.to_chip, {}

    def check_output(self, context: dict, value: bytes) -> tuple[bool, str]:
        return value == self.expect, value


class TestSuite:
    """
    Describes all tests that are available for a given program. Test suites
    should be defined by the plugin associated with the program running on
    the chip.
    """

    def __init__(self, name, elf, *args):
        self.name = name
        self.elf = elf
        self.tests = OrderedDict()
        for test in args:
            self.tests[test.id] = test


class TestStatus(enum.Enum):
    PASS = 0
    """
    Test passed with no errors.
    """

    FAIL_CHECK = 1
    """
    Test failed the associated ShmooTest `check_result` function call.
    """

    FAIL_NO_BEL = 2
    """
    Host did not receive a BEL header packet acknowledgment.
    """

    FAIL_NO_ETB = 3
    """
    Host did not receive an ETB test completion acknowledgment.
    """

    FAIL_NO_UART = 4
    """
    Host was unable to connect to the chip via UART.
    """

    SKIP_MAX_FREQ_FAIL = 5
    """
    Test was skipped due to earlier tests meeting the maximum frequency failure
    limit.
    """

    SKIP_VOLTAGE_FAIL = 6
    """
    Test was skipped due to the lowest tested frequency failing for the given
    voltage.
    """

    FAIL_INVALID_CHIP_PKT = 7
    """
    The chip sent an invalid packet to the host.
    """
    

class TestArtifact:
    """
    Contains information pertaining to a specific test run after a test has
    finished.
    """

    def __init__(self, status: TestStatus=None, context=None, host_payload=None,
                 chip_payload=None, check_data=None, csv_path=None,
                 avg_power=None, energy=None, has_measurements=None):
        self.status = status
        self.context = context
        self.host_payload = host_payload
        self.chip_payload = chip_payload
        self.check_data = check_data
        self.avg_power = avg_power
        self.energy = energy
        self.csv_path = csv_path
        self.has_measurements = has_measurements

    def __str__(self):
        return '\t'.join([
            str(self.status.name),
            str(self.context),
            self.host_payload.hex() if self.host_payload else 'None',
            str(self.check_data)
        ])


def float_to_str(val: float):
    return str.format('%.2f'%val)


class ShmooSuiteResults:

    def __init__(self, suite: TestSuite, output_dir: Optional[str]=None,
                 voltage_range: list=None, freq_range: list=None,
                 step_v=None, step_freq=None, readonly=False):
        self.suite = suite
        self.results = OrderedDict()
        self.results_list = []
        self.voltage_range = voltage_range or []
        self.freq_range = freq_range or []
        self.step_v = step_v
        self.step_freq = step_freq
        self.readonly = readonly

        if not readonly:
            self.output_dir = Path(output_dir or \
                f'data_{self.suite.name}_{datetime.now().isoformat()}')
            self.result_path = self.output_dir.joinpath('result.tsv')
            os.mkdir(self.output_dir)

            with open(self.result_path, 'a', encoding='utf-8') as f:
                f.write('\t'.join(['Suite Name', 'Test Name', 'Test ID', 'Voltage', 'Frequency', 'Status', 'Context', 'Host Payload', 'Compare Check Data']) + '\n')

    def add_result(self, test: ShmooTest, voltage, freq,
                          artifact: TestArtifact, data: Optional[np.ndarray] = None):
        self.results_list.append((test, voltage, freq, artifact))
        if test not in self.results:
            self.results[test] = OrderedDict()

        if voltage not in self.results[test]:
            self.results[test][voltage] = OrderedDict()

        self.results[test][voltage][freq] = artifact

        if not self.readonly:
            with open(self.result_path, 'a', encoding='utf-8') as f:
                f.write('\t'.join(
                    [self.suite.name, test.name, str(test.id),
                     float_to_str(voltage), str(freq), str(artifact)]) + '\n')
                
        # Compute the power values (if any)
        if isinstance(data, np.ndarray) and data.size > 0:
            powers = np.multiply(data[0], data[2])
            dt = np.diff(data[1])
            energy = np.sum(np.multiply(powers[:-1], dt)) 
            avg_power = energy / data[1][-1]
            artifact.avg_power = avg_power
            artifact.energy = energy
            artifact.has_measurements = True
        else:
            artifact.has_measurements = False
            
    @staticmethod
    def load_from_run(path: str):
        path = Path(path)

        res = None
        res_path = Path.joinpath(path, 'result.tsv')

        with open(res_path) as r:
            r.readline()

            voltage_range = set()
            freq_range = set()

            while line := r.readline():
                (suite_name, _, testid, voltage, freq, status,
                 context, host_payload, compare_data) = line.split('\t')
                testid = int(testid)
                voltage = float(voltage)
                freq = int(freq)

                #host_payload = bytes.fromhex(host_payload)

                suite = ShmooTestHarness.TEST_SUITES[suite_name]
                if not res:
                    res = ShmooSuiteResults(ShmooTestHarness.TEST_SUITES[suite_name],
                                            readonly=True)
                    res.output_dir = path
                    res.result_path = res_path

                voltage_range.add(float(voltage))
                freq_range.add(int(freq))

                # Extract all tests and compute powers
                artifact = TestArtifact(
                    status=TestStatus[status],
                    context=context,
                    # host_payload=host_payload,
                    check_data=compare_data,
                    csv_path=os.path.join(path, f'test_{testid}_{float_to_str(voltage)}v_{freq // 1000000}MHz.csv')
                )
                data = None
                if os.path.exists(artifact.csv_path):
                    data = np.genfromtxt(artifact.csv_path, delimiter=',').T
                res.add_result(
                    suite.tests[testid], voltage, freq, artifact, data)
            
            res.voltage_range = sorted(list(voltage_range))
            res.freq_range = sorted(list(freq_range))
            
            if len(res.voltage_range) > 1:
                res.step_v = res.voltage_range[1] - res.voltage_range[0]
            else:
                res.step_v = 0

            if len(res.freq_range) > 1:
                res.step_freq = res.freq_range[1] - res.freq_range[0]
            else:
                res.step_freq = 0
        return res

    def print_results(self):
        log = ''
        f'{Fore.MAGENTA}{Style.BRIGHT}--- Test Summary for Test Suite "{self.suite.name}" ---{Style.RESET_ALL}\n'
        log += f'{Fore.MAGENTA}{Style.BRIGHT}Test "{self.test.name}" [Test ID {self.test.id}] ---{Style.RESET_ALL}\n'


def annotate_heatmap(im, data=None, valfmt="{x:.2f}",
                     textcolors=("white", "black"),
                     threshold=None, **textkw):
    """
    A function to annotate a heatmap.

    Parameters
    ----------
    im
        The AxesImage to be labeled.
    data
        Data used to annotate.  If None, the image's data is used.  Optional.
    valfmt
        The format of the annotations inside the heatmap.  This should either
        use the string format method, e.g. "$ {x:.2f}", or be a
        `matplotlib.ticker.Formatter`.  Optional.
    textcolors
        A pair of colors.  The first is used for values below a threshold,
        the second for those above.  Optional.
    threshold
        Value in data units according to which the colors from textcolors are
        applied.  If None (the default) uses the middle of the colormap as
        separation.  Optional.
    **kwargs
        All other arguments are forwarded to each call to `text` used to create
        the text labels.
    """

    if not isinstance(data, (list, np.ndarray)):
        data = im.get_array()

    # Normalize the threshold to the images color range.
    if threshold is not None:
        threshold = im.norm(threshold)
    else:
        threshold = im.norm(data.max())/2.

    # Set default alignment to center, but allow it to be
    # overwritten by textkw.
    kw = dict(horizontalalignment="center",
              verticalalignment="center")
    kw.update(textkw)

    # Get the formatter in case a string is supplied
    if isinstance(valfmt, str):
        valfmt = matplotlib.ticker.StrMethodFormatter(valfmt)

    # Loop over the data and create a `Text` for each "pixel".
    # Change the text's color depending on the data.
    texts = []
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            if data[i, j] == 0:
                texts.append('')
                continue
            kw.update(color=textcolors[int(im.norm(data[i, j]) > threshold)])
            text = im.axes.text(j, i, valfmt(data[i, j], None), **kw)
            texts.append(text)

    return texts


class ShmooTestHarness:

    TEST_SUITES = {}
    """
    Mapping to maintain registered test suites.
    """

    UART_BAUD_RATE = 115200
    """
    Baud rate to use for UART communication between the host and chip.
    """

    TEST_PASSED_STR = Fore.GREEN + Style.BRIGHT + '[+++PASSED+++]' + Style.RESET_ALL
    TEST_FAILED_STR = Fore.RED + Style.BRIGHT + '[---FAILED---]' + Style.RESET_ALL

    CHIP_NAME = 'dsp24'
    """
    Name of the chip to test.
    """

    @staticmethod
    def register_test_suite(suite: TestSuite):
        ShmooTestHarness.TEST_SUITES[suite.name] = suite

    @staticmethod
    def list_suites():
        LOGGER.info(f'{Style.BRIGHT}Registered test suites:{Style.RESET_ALL}')
        for ste_name, suite in ShmooTestHarness.TEST_SUITES.items():
            LOGGER.info(f'\t{Fore.CYAN}{Style.BRIGHT}{ste_name}{Style.RESET_ALL} ({len(suite.tests)} tests)')
            for test_id, test in suite.tests.items():
                LOGGER.info(f'\t\t{Fore.MAGENTA}{Style.BRIGHT}{test.name}{Style.RESET_ALL} (Test ID: {test_id} [{hex(test_id)}])')

    @staticmethod
    def terminal_confirm_params(voltages: list, frequencies: list,
                                psu_mode: PSUSourceMode, no_psu: bool, psu_ch: int,
                                debug: bool):
        prompt = Fore.RED + Style.BRIGHT + 'Please confirm the following parameter sweep:\n' + Style.RESET_ALL
        prompt += Style.BRIGHT + 'Voltages: ' + str(voltages) + '\n' + Style.RESET_ALL
        prompt += Style.BRIGHT + 'Frequencies: ' + str(frequencies) + '\n' + Style.RESET_ALL
        prompt += Style.BRIGHT + 'PSU Source Settings: ' + psu_mode.value
        prompt += f' (Channel {psu_ch})\n{Style.RESET_ALL}'
        prompt += Style.BRIGHT + 'Dummy PSU (Log Redirect): ' + str(no_psu)

        if no_psu:
            prompt += Fore.MAGENTA + ' (SCPI commands will be redirected to a log. No power data will be collected.)'

        prompt += '\n' + Style.RESET_ALL
        
        if debug:
            prompt += f'{Style.BRIGHT}{Fore.CYAN}Debugging Mode Enabled\n{Style.RESET_ALL}'

        while True:
            resp = input(f"{prompt} (y/n): ").lower()
            if resp in ['y', 'yes']:
                return True
            elif resp in ['n', 'no']:
                print(Fore.RED + "Abort. No tests run." + Style.RESET_ALL)
                return False
            else:
                print("Invalid input. Please enter 'y' or 'n'.")

    @staticmethod
    def dbg_bp(command: str):
        """
        Debugging "breakpoint" within the host, permitting step-by-step test
        execution.

        Args:
            command (str): Description of what will run after pressing Enter.
        """
        input(f'{Style.BRIGHT}{Fore.LIGHTRED_EX}[DBG] Press Enter to {command}.{Style.RESET_ALL}')

    @staticmethod
    def test_run_suite(suite_name: str, voltages: list, frequencies: list):
        """
        Invokes the commands for a test suite to visualize the packets to be
        sent to the chip during debugging. 

        Args:
            suite_name (str): The name of the registered suite to run.
        """
        # Retrieve the correct test suite to run.
        suite = ShmooTestHarness.TEST_SUITES[suite_name]
        LOGGER.info(f'{Style.BRIGHT}--- Running potential outputs for suite "{suite_name}" ---{Style.RESET_ALL}')

        for _, test in suite.tests.items():
            LOGGER.info(f'{Style.BRIGHT}{Fore.MAGENTA}Test "{test.name}" (Test ID: {test.id} [{hex(test.id)}]){Style.RESET_ALL}')
            payload, context = test.create_payload()
            LOGGER.info(f'\t{Style.BRIGHT}Host-to-Chip Payload:{Style.RESET_ALL} {payload}')
            LOGGER.info(f'\t{Style.BRIGHT}Host-to-Chip Payload Size:{Style.RESET_ALL} {len(payload)}')
            LOGGER.info(f'\t{Style.BRIGHT}Generated Context Object:{Style.RESET_ALL} {context}')

    @staticmethod
    def log_as_host(val: str):
        LOGGER.info(f"{Fore.CYAN}{Style.BRIGHT}[Host]{Style.RESET_ALL} %s", val)

    @staticmethod
    def log_as_chip(val: str, red=False):
        clr = Fore.RED if red else Fore.BLUE
        LOGGER.info(f"{clr}{Style.BRIGHT}[Chip]{Style.RESET_ALL} %s", val)

    @staticmethod
    def log_as_misc(val: str):
        LOGGER.info(f"{Fore.YELLOW}{Style.BRIGHT}[Misc]{Style.RESET_ALL} %s", val)

    @staticmethod
    def log_test_result(test: ShmooTest, artifact: TestArtifact):
        """
        Outputs test completion information to the log.

        Args:
            test (ShmooTest): The test to display a result from.
            passed (bool): True if the test passed.
            csv_path (Optional[str], optional): Path to an output PSU Data
                File. Defaults to None.
        """
        if artifact.status == TestStatus.PASS:
            status_str = ShmooTestHarness.TEST_PASSED_STR
        else:
            status_str = ShmooTestHarness.TEST_FAILED_STR
        
        file_str = ''
        if artifact.csv_path:
            file_str = f' (PSU Data File: {artifact.csv_path})'
        LOGGER.info(f'{status_str} Test ID {test.id} [{test.name}]{file_str}')


    @staticmethod
    def kill_process_with_fire(proc: subprocess.Popen):
        while proc.poll() is None:
            proc.send_signal(11) # sigsegv
            proc.terminate() # sigterm
            proc.kill() # sigkill

    @staticmethod
    def reset_and_program_elf(elf: str):
        """
        Resets the chip, programs it with an elf at the path specified, then
        executes the program on the chip. Starts and stops an OpenOCD process.

        Args:
            elf (str): Path to the ELF file to upload.
        """
        
        # Reset the chip.
        success = False
        while not success:
            devices = UsbTools.build_dev_strings('ftdi', Ftdi.VENDOR_IDS, Ftdi.PRODUCT_IDS, Ftdi.list_devices())
            if not devices:
                raise Exception("No FTDI device found.")
            LOGGER.debug('Available FTDI Devices: %s', str(devices))
            for device in [d for d in devices if d[0].endswith('/1')]:
                gcont = GpioMpsseController()
                gcont.configure(device[0], direction=0x0100, frequency=10e6)
                port = gcont.get_gpio()
                LOGGER.debug(f"Resetting FTDI device {device}")
                port.write(0x00)
                sleep(.3)
                while gcont.is_connected:
                    gcont.close()

            # Attempt to launch OpenOCD subprocess
        
            ocd_proc = None
            ocd_failed_attempts = 0
            ocd_max_failures = 5
            while not ocd_proc and ocd_failed_attempts < ocd_max_failures:
                chipname = ShmooTestHarness.CHIP_NAME
                openocd_args = shlex.split(f"openocd -f ./platform/{chipname}/{chipname}.cfg")
                ocd_proc = subprocess.Popen(openocd_args, cwd=os.getcwd(),
                                            stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT,
                                            text=True)
                while ocd_proc:
                    line = ocd_proc.stdout.readline()
                    LOGGER.debug(f'[OpenOCD] {line}')
                    if not line:
                        ShmooTestHarness.kill_process_with_fire(ocd_proc)
                        ocd_proc = None
                    if line.startswith('Info : TAP riscv.cpu does not have valid IDCODE (idcode=0x0)') \
                        or line.startswith('Error:'):
                        ShmooTestHarness.log_as_misc(
                            "OpenOCD failed to launch. Retrying.")
                        ShmooTestHarness.kill_process_with_fire(ocd_proc)
                        ocd_proc = None
                        ocd_failed_attempts += 1
                    elif 'Info : Listening on port 4444 for telnet connections' in line:
                        break
            
            if not ocd_proc:
                ShmooTestHarness.log_as_misc(
                    f"OpenOCD failed {ocd_max_failures} times. Attempting another reset.")
                continue

            ShmooTestHarness.log_as_misc(
                "OpenOCD successfully connected to JTAG controller.")

            # Connect to OpenOCD via TCL
            with OpenOcdTclRpc() as openocd:
                # Run the program
                try:
                    if not os.path.exists(elf):
                        ShmooTestHarness.kill_process_with_fire(ocd_proc)
                        raise Exception(f'Unable to find a binary file at "{elf}".')
                    openocd.run('reset run')
                    openocd.run('halt')
                    openocd.run(f'load_image {elf} 0x0 elf')
                    openocd.run('resume 0x80000000')
                    success = True
                except TclException:
                    # The chip has not started up fast enough. Try again.
                    ShmooTestHarness.log_as_misc('Failed to program the chip. It is likely that you need to reset/re-program the FPGA. Trying again...')
                    ShmooTestHarness.kill_process_with_fire(ocd_proc)
                    continue

            ShmooTestHarness.log_as_misc(f"Uploaded program {elf} via OpenOCD.")

            # Try with all our might to kill OpenOCD
            ShmooTestHarness.kill_process_with_fire(ocd_proc)
            
            ShmooTestHarness.log_as_misc("Killed OpenOCD process.")

    @staticmethod
    def save_data_as_csv(data: np.ndarray, status: TestStatus, dir: str,
                         testid: int, voltage: float, freq: str) -> str:
        """
        Saves a NumPy array containing PSU-acquired data to a standardized CSV
        file.

        Returns:
            str: Path to the created output file.
        """
        filename = f'{dir}/test_{testid}_{float_to_str(voltage)}v_{freq}MHz.csv'
        np.savetxt(filename, data, delimiter=",")
        return filename
    
    @staticmethod
    def np_arr_to_float_vectorized(arr, dtype):
        new_arr = np.zeros_like(arr, dtype=dtype)
        
        can_convert = np.vectorize(lambda x: True if isinstance(x, (int, float, str)) else False)(arr)

        try:
            new_arr[can_convert] = arr[can_convert].astype(dtype)
        except (ValueError, TypeError):
            new_arr[can_convert] = 0
            
        return new_arr

    @staticmethod
    def run_suite(suite_name: str, tests: Union[list[int], None], voltages: list, frequencies: list,
                  max_consec_voltage_fails: int, freq_retries: int,
                  psu_mode: PSUSourceMode, psu_dummy: bool, psu_channel: int,
                  output_path: str, debug: bool, no_upload: bool, test_runs: int
                  )-> ShmooSuiteResults:
        """
        Runs a test with the full flow, along with serial instantiation, for
        a given test harness.

        Args:
            suite_name (str): The name of the registered suite to run.
            tests (Union[list[int], None]): Test IDs to run. If None, this will
                run all tests for a provided test suite.
            voltages (list): List of voltages to sweep.
            frequencies (list): List of frequencies (in MHz) to sweep.
            max_consec_voltage_fails (int): Max number of consecutive
                failures permitted for a given voltage. For example, a value
                of 2 means that if two subsequent failures occur for a voltage,
                all other frequencies will be skipped for that voltage. Setting
                this value to 0 disables the check overall.
            freq_retries (int): Max number of cumulative
                retries permitted for a given frequency. For example, a value
                of 1 means that if one failure occurs at a specific frequency
                for the current voltage, the test will be re-run. Once the test
                exceeds the provided retries, the tester will then follow
                necessary failover rules based on the `max_cmul_voltage_fails`
                value.
            psu_mode (PSUSourceMode): Sets the remote sense relay sensing mode
                for the PSU.
            psu_dummy (bool): True if we want to not use the PSU, but rather
                redirect all SCPI commands to the log.
            psu_channel (int): Channel to control on the PSU connected to the
                DUT.
            output_path (str): Path to store output testing collateral files.
            debug (bool): Enables debugging mode, which has a step-
                by-step debugging system and infinite timeouts.
            no_upload (bool): If True, disables reprogramming the
                chip via OpenOCD.
            test_runs (int): Number of runs of the test prior to capturing
                power data. This can be used to warm up caches.
        """
        LOGGER.info(f'{Style.BRIGHT}{Fore.YELLOW}--- Starting enumeration for test suite "{suite_name}" ---{Style.RESET_ALL}')
        
        # Toggle debugging breakpoints
        dbg_bp = ShmooTestHarness.dbg_bp if debug else lambda x: None
        
        ### PSU Initialization ###
        psu = PSU(dummy=psu_dummy)
        psu.set_mode(psu_mode, channel=psu_channel)
        #psu.reset_channel(psu_channel)
        # psu.enable(psu_channel)
        
        # Retrieve the correct test suite to run.
        suite = ShmooTestHarness.TEST_SUITES[suite_name]

        # Convert all frequencies to hertz
        frequencies = [int(x) * 1000000 for x in frequencies]

        results = ShmooSuiteResults(suite, output_dir=output_path)
        results.voltage_range = [float(v) for v in voltages]
        results.freq_range = frequencies.copy()
        ShmooTestHarness.log_as_misc(f'Output will be stored within "{results.output_dir}/"')


        # This routine treats voltages and frequencies as a stack, such that we
        # can re-attempt at will.
        if tests:
            tests_to_run = []
            for test_id in tests:
                if test_id not in suite.tests:
                    LOGGER.warning(f'A registered test with ID {test_id} was not found for suite "{suite.name}"')
                else:
                    tests_to_run.append(suite.tests[test_id])
        else:
            tests_to_run = list(suite.tests.values())

        for test in tests_to_run:
            test_timeout = None if debug else test.timeout

            # test_results = ShmooTestResults(suite, suite_results.result_path)
            pending_voltages_stack = deque(voltages)
            while pending_voltages_stack:
                cur_v = float(pending_voltages_stack.popleft())

                # Count of how many previous tests at this voltage failed (to
                # know when to stop trying, perhaps by a configurable amount).
                voltage_consec_fails = 0

                # Create a deque of tuples: (freq to test, # of retries)
                pending_freqs_stack = deque(
                    ((x, max_consec_voltage_fails) for x in frequencies))

                def test_finish_handler(freq_hz: int, art: TestArtifact, retries: int,
                                        intermediate:bool, data=None):
                    nonlocal test, voltage_consec_fails, max_consec_voltage_fails, pending_freqs_stack, results, cur_v
                    
                    ShmooTestHarness.log_test_result(test, artifact)
                    
                    # Update the cumulative failure counter
                    if art.status != TestStatus.PASS:

                        # If failed, can we still retry the test?
                        if retries > 0:
                            remaining_retries = retries - 1
                            ShmooTestHarness.log_as_misc(
                                f'Retrying previous test "{test.name}" at {freq_hz} Hz ({remaining_retries} retries left).')
                            pending_freqs_stack.appendleft(
                                (freq_hz, remaining_retries))
                            return
                        else:
                            ShmooTestHarness.log_as_misc(f'No retries available for failed test at {freq_hz} Hz.')
                        
                        voltage_consec_fails += 1
                    
                    if not intermediate or (art.status != TestStatus.PASS and intermediate):
                        results.add_result(test, cur_v, freq_hz, artifact, data.T if data is not None else data)

                    # If we have exceeded our allowed cumulative fail count,
                    # stop processing freqs for this voltage.
                    if max_consec_voltage_fails > 0 and voltage_consec_fails >= max_consec_voltage_fails:
                        ShmooTestHarness.log_as_misc(f'Met maximum consecutive failures for {cur_v} V. Skipping the following remaining frequency tests at this voltage: {pending_freqs_stack}.')
                        
                        # Clean out the rest of the frequencies
                        while pending_freqs_stack:
                            freq_skipped, retries = pending_freqs_stack.popleft()
                            results.add_result(test,
                                cur_v, freq_skipped,
                                TestArtifact(TestStatus.SKIP_MAX_FREQ_FAIL))

                while pending_freqs_stack:
                    freq_hz, retries = pending_freqs_stack.popleft()
                    freq_mhz = freq_hz // 1000000

                    LOGGER.info(f'{Style.BRIGHT}{Fore.MAGENTA}--- [Test ID {test.id}] Running at {freq_mhz} MHz and {float_to_str(cur_v)} V ---{Style.RESET_ALL}')
                    artifact = TestArtifact()

                    ### Serial Port Evaluation / FTDI Reset / PSU Setup ###
                    
                    # Set PSU Limits based on sweep.
                    dbg_bp('set PSU limits and enable PSU')
                    psu.set_limits(psu_channel, voltage=cur_v)
                    psu.enable(psu_channel)
                    sleep(0.3)
                    
                    if no_upload:
                        ShmooTestHarness.log_as_misc("No Upload is True, skipping chip programming step.")
                    else:
                        dbg_bp('reset and program the chip')
                        ShmooTestHarness.reset_and_program_elf(suite.elf)

                    dbg_bp('attempt ENQ/ACK-verified UART connection (Program needs to be within init_test for this to work)')
                    ser = SerialDebug.create(ShmooTestHarness.UART_BAUD_RATE,
                                             timeout=test_timeout)

                    if not ser:
                        ShmooTestHarness.log_as_chip(
                            f'Unable to connect to the chip over UART. No destinations available for ENQ connection handshake.',
                            red=True)
                        artifact.status = TestStatus.FAIL_NO_UART
                        test_finish_handler(freq_hz, artifact, retries, intermediate=False)
                        continue

                    run_errored = False
                    for run_number in range(test_runs):
                        is_intermediate = not (run_number+1 == test_runs)

                        # This will check if a subtest failed. If so, break to outer attempt layer
                        if run_errored:
                            break
                        
                        ShmooTestHarness.log_as_misc(f'Starting run {run_number + 1} of {test_runs}.')

                        ser.flushInput()
                        ser.flushOutput()
                        
                        ### Begin Host<-->Chip Communication ###

                        # Host sends a signal for the start of header (SOH)
                        dbg_bp('send host payload to chip')
                        ShmooTestHarness.log_as_host("Sending Start of Header (SOH)")
                        ser.write(b'\x01')

                        # Host sends the size of the data packet
                        host_to_chip_payload, context = test.create_payload()
                        artifact.host_payload = host_to_chip_payload
                        artifact.context = context

                        data_pkt_size = len(host_to_chip_payload)
                        ShmooTestHarness.log_as_host(
                            f'Size of header data packet is {data_pkt_size}')
                        ser.write(data_pkt_size.to_bytes(4, byteorder='little'))

                        # Host sends clock frequency over UART (Hz) (64-bit int)
                        ShmooTestHarness.log_as_host(
                            f'Clock frequency is {freq_hz} Hz')
                        ser.write(freq_hz.to_bytes(8, byteorder='little'))

                        # Host sends test ID (8-bit value)
                        ShmooTestHarness.log_as_host(
                            f'Current Test ID is {test.id}')
                        ser.write(test.id.to_bytes(1, byteorder='little'))

                        # Host sends header data
                        ser.write(host_to_chip_payload)
                        ShmooTestHarness.log_as_host(
                            f'Sent the following payload: {host_to_chip_payload}')

                        # Chip responds with BEL (7)
                        ShmooTestHarness.log_as_host(
                            f'Waiting for BEL (7, 0x07) payload acknowledgment...')

                        ser_data = ser.read_until(b'\x07')
                        start_time = datetime.now()
                        if not ser_data:
                            # Timeout occurred, treat as chip fail
                            ShmooTestHarness.log_as_chip(
                                f'No BEL (7) payload acknowledgment was received within timeout period ({test_timeout} seconds).',
                                red=True)
                            run_errored = True
                            artifact.status = TestStatus.FAIL_NO_BEL
                            test_finish_handler(freq_hz, artifact, retries,
                                                intermediate=is_intermediate)
                            break
                        
                        ShmooTestHarness.log_as_chip(
                            f'Sent BEL (7) payload acknowledgment!')
                        
                        ShmooTestHarness.log_as_host(
                            f'Waiting for ETB (23, 0x17) test completion acknowledgment...')

                        # Chip performs work. Host ignores any UART that is not ETB.
                        meas_v = []
                        meas_i = []
                        done = False
                        etb_data = None
                        timeout_time = start_time + timedelta(seconds=test.timeout)
                        if debug:
                            timeout_time = start_time + timedelta(days=7)
                        else:
                            timeout_time = start_time + timedelta(seconds=test.timeout)
                        end_time = None
                        
                        while (not done and datetime.now() <= timeout_time):
                            while not ser.in_waiting and datetime.now() <= timeout_time:
                                meas_v.append(psu.query(f"MEAS:VOLT? CH{psu_channel}"))
                                meas_i.append(psu.query(f"MEAS:CURR? CH{psu_channel}"))

                            # If we still don't have UART data, then test timed out.
                            if not ser.in_waiting:
                                break

                            while ser.in_waiting:
                                etb_data = ser.read()
                                if etb_data == b'\x17':
                                    end_time = datetime.now()
                                    done = True
                                    break

                        # Chip responds with ETB (23)
                        if not etb_data:
                            # Timeout occurred, treat as chip fail
                            ShmooTestHarness.log_as_chip(
                                f'No ETB (23) test completion acknowledgment was received within timeout period ({test.timeout} seconds).',
                                red=True)
                            run_errored = True
                            artifact.status = TestStatus.FAIL_NO_ETB
                            test_finish_handler(freq_hz, artifact, retries,
                                                intermediate=is_intermediate)
                            break
                        
                        ShmooTestHarness.log_as_chip(
                            f'Sent ETB (23, 0x17) test completion acknowledgment!')
                        
                        if end_time and start_time:
                            test_time = end_time - start_time
                            ShmooTestHarness.log_as_misc(f'Test completed in {test_time.microseconds} μs')

                        # Chip sends size of payload packet (in bytes) (32-bit int)
                        ShmooTestHarness.log_as_host(
                            f'Awaiting chip payload packet size...')
                        
                        payload_size_bytes = ser.read(4)
                        if len(payload_size_bytes) < 4:
                            # Invalid payload size
                            ShmooTestHarness.log_as_chip(
                                f'Invalid chip payload size received: {payload_size_bytes}. Expected 4 bytes wide.',
                                red=True)
                            run_errored = True
                            artifact.status = TestStatus.FAIL_INVALID_CHIP_PKT
                            test_finish_handler(freq_hz, artifact, retries,
                                                intermediate=is_intermediate)
                            break

                        payload_size = struct.unpack('<I', payload_size_bytes)[0]
                        ShmooTestHarness.log_as_chip(
                            f'Payload packet size is {payload_size} ({payload_size_bytes})')

                        # Chip responds with Payload
                        ShmooTestHarness.log_as_host(
                            f'Waiting for chip payload packet...')
                        
                        chip_payload = ser.read(payload_size)
                        artifact.chip_payload = chip_payload

                        ShmooTestHarness.log_as_chip(
                            f'Payload packet: {chip_payload}')

                        ### Post-Processing ###

                        # Check the output against the ShmooTest function.
                        artifact.has_measurements = True

                        if isinstance(context, dict):
                            context['chip_freq'] = freq_hz

                        passed, check_data = test.check_output(context, chip_payload)
                        if passed:
                            artifact.status = TestStatus.PASS
                        else:
                            artifact.status = TestStatus.FAIL_CHECK
                        artifact.check_data = check_data
                        
                        # Parse the data from the PSU and form it into a matrix.
                        timestamps = np.linspace(0.0, test_time.microseconds,
                                                num=len(meas_v), endpoint=True)
                        meas_as_text = np.column_stack((meas_v, timestamps, meas_i))
                        if len(meas_as_text) > 0:
                            meas_as_text = np.char.strip(meas_as_text)

                            measurements = ShmooTestHarness.np_arr_to_float_vectorized(
                                meas_as_text, np.number)
                        else:
                            measurements = meas_as_text

                        LOGGER.debug(f'[PSU Measurements] {measurements}')

                        # Generate and save a CSV of the PSU data.
                        if artifact.has_measurements and not is_intermediate:
                            csv_path = ShmooTestHarness.save_data_as_csv(
                                measurements, artifact.status, results.output_dir,
                                test.id, cur_v, freq_mhz)
                            artifact.csv_path = csv_path
                        
                        # Output appropriate result to the log and keep track of
                        # the result in our results object.
                        test_finish_handler(freq_hz, artifact, retries,
                                            intermediate=is_intermediate,
                                            data=measurements)
                    ser.close()

        ShmooTestHarness.make_shmoo_plot(results)
        return results
    
    def make_shmoo_plot(results: ShmooSuiteResults):
        tests = {}
        new_arr = lambda: np.zeros((len(results.voltage_range), len(results.freq_range)))
        voltages_idxs = {v: i for i, v in enumerate(results.voltage_range)}
        freq_idxs = {v: i for i, v in enumerate(results.freq_range)}
        display_numbers = True

        for res in results.results_list:
            test, voltage, freq, artifact = res
            if test not in tests:
                tests[test] = new_arr()

            value = 0
            if artifact.has_measurements:
                value = artifact.avg_power
            else:
                # display_numbers = False
                value = 1 if artifact.status == TestStatus.PASS else 0
            tests[test][voltages_idxs[voltage], freq_idxs[freq]] = value * 1000
        
        for test in tests:
            fig = plt.figure(figsize=(12, 8))
            ax = plt.axes()
            plt.xlabel('Frequency (MHz)')
            plt.ylabel('Voltage (V)')
            plt.title(f'Shmoo Plot - {test.name}')

            # Colormap
            colors1 = plt.cm.binary(120)
            colors2 = plt.cm.plasma(np.linspace(0, 1, 128))

            # Combine the sampled colors
            colors = np.vstack((colors1, colors2))

            # Create a new colormap
            shmoo_cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
                "shmoo_cmap", colors)

            plt.xticks(
                np.arange(len(results.freq_range)),
                [x // 1000000 for x in results.freq_range],
                rotation=45)
            plt.yticks(np.arange(len(results.voltage_range)), [str("%.2f"%x) for x in results.voltage_range])


            im = plt.imshow(tests[test], cmap=shmoo_cmap,
                            aspect='equal', origin='lower')

            if display_numbers:
                annotate_heatmap(im, valfmt="{x:.3g}", size=8)
                divider = make_axes_locatable(ax)
                cax = divider.append_axes("right", size="2%", pad=0.05)
                cbar = plt.colorbar(im, cax=cax)
                cbar.set_label('Power (mW)', rotation=270, labelpad=10)

            plt.tight_layout()
            imgpath = os.path.join(results.output_dir, f'test_{test.id}_plot.png')
            plt.savefig(imgpath, dpi=600)
            ShmooTestHarness.log_as_misc(f'Saved Shmoo plot for test ID {test.id} to {imgpath}')


# Describe exports
__all__ = ['SerialDebug', 'ShmooTest', 'ShmooConstantTest', 'TestSuite',
           'ShmooTestHarness', 'ShmooSuiteResults', 'PSUSourceMode']
