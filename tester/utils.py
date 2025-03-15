"""
Shmoo testing utilities for use with plugins.
"""
from datetime import datetime
import logging
import enum
import os
import shlex
import struct
import subprocess
from time import sleep
from typing import *
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


class SMU:
    """
    Wrapper class for all PyVisa SMU commands and data acquisition management.
    """

    SMU_IDN = 'Keithley Instruments Inc., Model 2602A'
    """
    Identification string to check for valid equipment setup.
    """
    
    INIT_CURRENT_LIMIT = "500e-3"
    """
    Defines the default current limit for the SourceMeter.
    """
    
    INIT_VOLTAGE_LIMIT = "0.85"
    """
    Defines the default voltage limit for the SourceMeter.
    """

    VISA_PATH = 'TCPIP::169.254.58.10::gpib0,13::INSTR'
    """
    Defines the default path to use for communication with the SourceMeter.
    """

    DUMMMY_LOG_PREFIX = f'{Fore.LIGHTBLUE_EX}{Style.BRIGHT}[Dummy SMU]{Style.RESET_ALL}'
    
    def __init__(self, dummy=False):
        """
        SMU Initialization

        Args:
            dummy (bool, optional): Whether to create a fake simulation SMU to
                test the command flow. This will cause all `SMU.write` commands
                to be output to the log rather than over LAN. Any `SMU.query`
                commands will return an empty string. Defaults to False.

        Raises:
            Exception: _description_
        """
        self.dummy = dummy
        if self.dummy:
            self.dummy_log('Dummy SMU has been created.')
            return
        
        self.rm = pyvisa.ResourceManager("@py")
        self._smu = self.rm.open_resource(self.VISA_PATH)

        # Verify that we have the correct equipment
        smu_verification = self.query("*IDN?")
        LOGGER.debug(f'SourceMeter: {smu_verification}')

        if not smu_verification.startswith(self.SMU_IDN):
            raise Exception(f'Attempt to connect to `{self.SMU_IDN}` at {self.VISA_PATH} failed, IDN returned `{smu_verification}` instead.')

        # Reset with our defaults
        self.reset()

    def dummy_log(self, val):
        LOGGER.info(f'{self.DUMMMY_LOG_PREFIX} {val}')

    def query(self, querystr: str) -> str:
        """
        Sends a query over GPIB to the SMU, expecting a result. This can be a
        multiline script query.

        Args:
            querystr (str): The query to send to the SMU.

        Returns:
            str: Data returned by the SMU.
        """
        if self.dummy:
            self.dummy_log(f'QUERY: {querystr}')

            if querystr == "*IDN?":
                return self.SMU_IDN

            return ""
        else:
            return self._smu.query(querystr)

    def write(self, cmdstr: str) -> str:
        """
        Writes a command over GPIB to the SMU. This can be a multiline script.

        Args:
            cmdstr (str): The command to send to the SMU.

        Returns:
            str: Data returned by the SMU.
        """
        if self.dummy:
            self.dummy_log(f'QUERY: {cmdstr}')
            return ""
        else:
            return self._smu.write(cmdstr)
        
    def set_timeout(self, timeout: int):
        """
        Sets a command timeout for a query response from the SMU.

        Args:
            timeout (int): Millisecond timeout duration
        """
        if self.dummy:
            self.dummy_log(f'Host<-->SMU timeout set to {timeout} ms')
        else:
            self._smu.timeout = timeout

    def get_timeout(self):
        """
        Gets a command timeout for a query response from the SMU.
        """
        if self.dummy:
            return 5000
        else:
            return self._smu.timeout


    def set_voltage_limit(self, voltage: Union[str, int, float]):
        """
        Sets a voltage limit for the SMU over GPIB.

        Args:
            voltage (Union[str, int, float]): The voltage limit (V) to set for
                the SMU.
        """
        self.write("smub.source.func = smub.OUTPUT_DCVOLTS")
        # smu.write("smub.source.autorangev = smub.AUTORANGE_ON")
        self.write(f"smub.source.limitv = {voltage}")
        self.write(f"smub.source.levelv = {voltage}")
        self.write(f"smub.source.rangev = {voltage}")

    def set_current_limit(self, current: Union[str, int, float]):
        """
        Sets a current limit for the SMU over GPIB.

        Args:
            current (Union[str, int, float]): The current limit (A) to set for
                the SMU.
        """
        self.write(f"smub.measure.rangei = {current}")
        self.write(f"smub.source.limiti = {current}")

    def clear_buffers(self, src_vals=True, timestamps=True, append=True):
        """
        Clear SMU buffers and set data collection settings.

        Args:
            idx (int): 1 or 2 depending on the buffer to clear.
            src_vals (bool, optional): Whether to collect source values in the
                buffer during a measurement. Defaults to True.
            timestamps (bool, optional): Whether to collect timestamps during a
                measurement. Defaults to True.
            append (bool, optional): Whether to append or overwrite the buffer
                upon new measurements. Defaults to True.
        """
        src_vals = '1' if src_vals else '0'
        timestamps = '1' if timestamps else '0'
        append = '1' if append else '0'
        self.write(f"smub.nvbuffer1.clear()")
        self.write(f"smub.nvbuffer1.collectsourcevalues = {src_vals}")
        self.write(f"smub.nvbuffer1.collecttimestamps = {timestamps}")
        self.write(f"smub.nvbuffer1.appendmode = {append}")
        self.write(f"smub.nvbuffer2.clear()")
        self.write(f"smub.nvbuffer2.collectsourcevalues = {src_vals}")
        self.write(f"smub.nvbuffer2.collecttimestamps = {timestamps}")
        self.write(f"smub.nvbuffer2.appendmode = {append}")

    def set_nplc(self, nplc: Union[str, int, float]):
        """
        Sets the integration aperture for SMU measurements.
        
        This setting controls the integration aperture for the integrating
        analog-to-digital converter (ADC). The integration aperture is based on
        the number of power line cycles (NPLC), where 1 PLC for 60 Hz is 16.67 ms
        (1/60) and 1 PLC for 50 Hz is 20 ms (1/50).

        For example, 0.5 sets the integration time for SMU channel B to 0.5/60
        seconds.

        Args:
            nplc (Union[str, int, float]): Integration aperture [0.001, 25]
        """
        self.write(f"smub.measure.nplc = {nplc}")

    def reset(self, reset_buffers=True):
        """
        Resets the SMU to default settings.

        Args:
            reset_buffers (bool, optional): If true, resets both data capture
                buffers. Defaults to True.
        """
        # Initialize SMU
        self.write("smub.reset()")
        self.set_current_limit(self.INIT_CURRENT_LIMIT)
        self.set_voltage_limit(self.INIT_VOLTAGE_LIMIT)

        # Clear and reset buffers
        if reset_buffers:
            self.clear_buffers()

        # Capture count / integration aperture
        self.write("smub.measure.count = 1")
        self.set_nplc('0.1')
        self.enable()

    def start_continuous_capture(self, gpio: Union[int, str], buffer_idx: int,
                                 timeout: int, voltage: float, freq: float):
        """
        Performs a GPIO-interrupted continuous capture routine on the SMU
        through a looping routine running on the SMU itself. Once the GPIO pin
        specified goes high, the routine stops. This command is non-blocking,
        and a subsequent `retrieve_buffer()` call should be made to acquire the
        data from this capture session.

        This function does not assume that buffers are set to `append` mode. It
        is advised to clear the buffers outside of a testing context prior to
        calling this function.

        Args:
            gpio (Union[int, str]): The index of the GPIO bit to poll. This bit
                should be held low for the duration of the test, then pulled
                high when finished.
            buffer_idx (int): 1 or 2 to specify the buffer index to use for
                data capture.
            timeout (int): Timeout (in seconds) for when the program assumes
                chip failure and quits.
        """
        timeout_str = str(timeout)
        gpib_cmd = f'''
        errorqueue.clear()
        display.clear()
        display.setcursor(1, 1)
        display.settext("Waiting..")
        display.setcursor(2, 1)
        display.settext("LimV: {str.format("%.2f"%voltage)} V | Freq: {freq} MHz")
        start_time = os.clock()
        end_time = start_time + {timeout_str}
        while (digio.readbit({gpio}) == 1.00000e+00 and os.clock() < end_time) do  end
        display.setcursor(1, 1)
        display.settext("Measuring")
        while (digio.readbit({gpio}) == 0.00000e+00 and os.clock() < end_time) do smub.measure.overlappediv(smub.nvbuffer1, smub.nvbuffer2) waitcomplete() end
        display.clear()
        display.setcursor(1, 1)
        display.settext("Done!")
        display.setcursor(2, 1)
        display.settext("LimV: {str.format("%.2f"%voltage)} V | Freq: {freq} MHz")'''
        self.write(f"display.screen = display.USER")
        self.write('display.clear()')
        self.write(gpib_cmd)
        

    def retrieve_buffer(self, idx: int) -> np.ndarray:
        """
        Retrieves content from a data capture buffer on the SMU.

        Content is ordered as follows:
        ```
        meas1, timstamp1, srcval1
        ```

        Args:
            idx (int): 1 or 2 depending on the buffer you wish to read from.
            
        Returns:
            str: Buffer contents
        """
        if self.dummy:
            return np.array([])

        old_timeout = self.get_timeout()
        self.set_timeout(5000)
        try:
            values_i = self.query('printbuffer(1, smub.nvbuffer1.n, smub.nvbuffer1)')
            values_v = self.query('printbuffer(1, smub.nvbuffer2.n, smub.nvbuffer2, smub.nvbuffer1.timestamps)')
        except Exception:
            return None
        
        data_v = np.fromstring(values_v, dtype=float, sep=",").reshape((-1, 2))
        data_i = np.fromstring(values_i, dtype=float, sep=",")
        data = np.column_stack((data_v, data_i))
        self.set_timeout(old_timeout)

        return data
    
    def enable(self):
        """
        Enables SMU output on channel B.
        """
        self.write('smub.source.output = smub.OUTPUT_ON')

    def disable(self):
        """
        Disables SMU output on channel B.
        """
        self.write('smub.source.output = smub.OUTPUT_OFF')


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

    FAIL_SMU = 4
    """
    Host did not receive a response from the SMU during testing.
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
    

class TestArtifact:
    """
    Contains information pertaining to a specific test run after a test has
    finished.
    """

    def __init__(self, status: TestStatus=None, context=None, host_payload=None,
                 chip_payload=None, check_data=None, csv_path=None):
        self.status = status
        self.context = context
        self.host_payload = host_payload
        self.chip_payload = chip_payload
        self.check_data = check_data
        self.csv_path = csv_path

    def __str__(self):
        return '\t'.join([
            str(self.status.name),
            str(self.context),
            self.host_payload.hex() if self.host_payload else 'None',
            str(self.check_data)
        ])


class ShmooSuiteResults:

    def __init__(self, suite: TestSuite):
        self.suite = suite


class ShmooTestResults:
    """
    Container class to store multiple test artifacts with their associated
    statuses and voltage/frequency parameters.
    """

    def __init__(self, test, result_csv_path):
        # Results stored as {Voltage: {Freq: (PassFail, CSV Path), ...}, ...}
        self.suite = test
        self.result_csv_path = result_csv_path
        self.results = OrderedDict()

    def add_result(self, voltage, freq,
                   artifact: TestArtifact):
        if voltage not in self.results:
            self.results[voltage] = OrderedDict()
        self.results[voltage][freq] = artifact

        with open(self.result_csv_path, 'a', encoding='utf-8') as f:
            f.write('\t'.join([str(voltage), str(freq), str(artifact)]) + '\n')

    def log_summary(self):
        data = []
        # log = ''
        # log += f'{Fore.MAGENTA}{Style.BRIGHT}--- Test Summary for Test "{self.test.name}" [Test ID {self.test.id}] ---{Style.RESET_ALL}\n'
        # log += f'{Fore.MAGENTA}{Style.BRIGHT}--- Test Summary for Test Suite "{self.suite.name}" ---{Style.RESET_ALL}'
        # for voltage, freqs in self.results.items():
        #     for freq, artifact in freqs.items():
                

        # LOGGER.info(log)


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
    def terminal_confirm_params(voltages: list, frequencies: list):
        prompt = Fore.RED + Style.BRIGHT + 'Please confirm the following parameter sweep:\n' + Style.RESET_ALL
        prompt += Style.BRIGHT + 'Voltages: ' + str(voltages) + '\n' + Style.RESET_ALL
        prompt += Style.BRIGHT + 'Frequencies: ' + str(frequencies) + '\n' + Style.RESET_ALL
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
            csv_path (Optional[str], optional): Path to an output SMU Data
                File. Defaults to None.
        """
        if artifact.status == TestStatus.PASS:
            status_str = ShmooTestHarness.TEST_PASSED_STR
        else:
            status_str = ShmooTestHarness.TEST_FAILED_STR
        
        file_str = ''
        if artifact.csv_path:
            file_str = f' (SMU Data File: {artifact.csv_path})'
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
        while not ocd_proc:
            openocd_args = shlex.split("openocd -f ./platform/dsp24/dsp24.cfg")
            ocd_proc = subprocess.Popen(openocd_args, cwd=os.getcwd(),
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT,
                                        text=True)
            sleep(0.5)
            while ocd_proc:
                line = ocd_proc.stdout.readline()
                if not line:
                    ShmooTestHarness.kill_process_with_fire(ocd_proc)
                    ocd_proc = None
                if line.startswith('Info : TAP riscv.cpu does not have valid IDCODE (idcode=0x0)') \
                    or line.startswith('Error:'):
                    ShmooTestHarness.log_as_misc(
                        "OpenOCD failed to launch. Retrying.")
                    ShmooTestHarness.kill_process_with_fire(ocd_proc)
                    ocd_proc = None
                elif 'Examination succeed' in line:
                    break
        
        ShmooTestHarness.log_as_misc(
            "OpenOCD successfully connected to JTAG controller.")

        # Connect to OpenOCD via TCL
        with OpenOcdTclRpc() as openocd:
            # Run the program
            openocd.run(f'load_image {elf} 0x0 elf')
            openocd.run('resume 0x80000000')

        ShmooTestHarness.log_as_misc(f"Uploaded program {elf} via OpenOCD.")

        # Try with all our might to kill OpenOCD
        ShmooTestHarness.kill_process_with_fire(ocd_proc)
        
        ShmooTestHarness.log_as_misc("Killed OpenOCD process.")

    @staticmethod
    def save_data_as_csv(data: np.ndarray, status: TestStatus, dir: str,
                         testid: int, voltage: float, freq: str) -> str:
        """
        Saves a NumPy array containing SMU-acquired data to a standardized CSV
        file.

        Returns:
            str: Path to the created output file.
        """
        filename = f'{dir}/test_{testid}_{str.format("%.2f"%voltage)}v_{freq}MHz.csv'
        np.savetxt(filename, data, delimiter=",")
        return filename

    @staticmethod
    def run_suite(suite_name: str, voltages: list, frequencies: list,
                  max_cmul_freq_fails: int = 1, use_smu: bool = True):
        """
        Runs a test with the full flow, along with serial instantiation, for
        a given test harness.

        Args:
            suite_name (str): The name of the registered suite to run.
            voltages (list): List of voltages to sweep.
            frequencies (list): List of frequencies (in MHz) to sweep.
            max_cmul_freq_fails (int, optional): Max number of cumulative
                failures permitted for a given voltage. For example, a value of
                2 means that if two subsequent failures occur for a voltage,
                all other frequencies will be skipped for that voltage. Setting
                this value to 0 disables the check overall. Defaults to 1.
        """
        LOGGER.info(f'{Style.BRIGHT}{Fore.YELLOW}--- Starting enumeration for test suite "{suite_name}" ---{Style.RESET_ALL}')
        
        ### Hardware Initialization ###
        smu = SMU(dummy=not use_smu)
        
        # Retrieve the correct test suite to run.
        suite = ShmooTestHarness.TEST_SUITES[suite_name]

        output_dir = f'data_{suite_name}_{datetime.now().isoformat()}'
        os.mkdir(output_dir)
        ShmooTestHarness.log_as_misc(f'Output will be stored within "{output_dir}/"')
        result_csv_path = f'{output_dir}/result.tsv'

        with open(result_csv_path, 'a', encoding='utf-8') as f:
            f.write('\t'.join(['Voltage', 'Frequency', 'Status', 'Context', 'Host Payload', 'Compare Check Data']) + '\n')

        suite_results = ShmooSuiteResults(suite)

        # This routine treats voltages and frequencies as a stack, such that we
        # can re-attempt at will. 
        for _, test in suite.tests.items():

            results = ShmooTestResults(suite, result_csv_path)
            pending_voltages_stack = deque(voltages)
            while pending_voltages_stack:
                cur_v = float(pending_voltages_stack.popleft())

                # Count of how many previous freq. tests failed (to know when to
                # stop trying, perhaps by a configurable amount).
                freq_last_failed = 0

                # Create a deque of available frequencies
                pending_freqs_stack = deque(frequencies)

                def max_fail_check(art):
                    nonlocal freq_last_failed, max_cmul_freq_fails, pending_freqs_stack, results, cur_v
                    # Update the cumulative failure counter
                    if art.status != TestStatus.PASS:
                        freq_last_failed += 1

                    # If we have exceeded our allowed cumulative fail count,
                    # stop processing freqs for this voltage.
                    if max_cmul_freq_fails > 0 and freq_last_failed >= max_cmul_freq_fails:
                        ShmooTestHarness.log_as_misc(f'Met maximum cumulative frequency failures for {cur_v} V. Skipping remaining frequency tests at this voltage: {pending_freqs_stack}.')
                        
                        # Clean out the rest of the frequencies
                        while pending_freqs_stack:
                            freq_skipped = pending_freqs_stack.popleft()
                            results.add_result(
                                cur_v, freq_skipped * 1000000,
                                TestArtifact(TestStatus.SKIP_MAX_FREQ_FAIL))

                while pending_freqs_stack:
                    cur_clk = int(pending_freqs_stack.popleft())

                    LOGGER.info(f'{Style.BRIGHT}{Fore.MAGENTA}--- [Test ID {test.id}] Running at {cur_clk} MHz and {cur_v} V ---{Style.RESET_ALL}')
                    artifact = TestArtifact()

                    ### Serial Port Evaluation / FTDI Reset / SMU Setup ###
                    
                    smu.set_voltage_limit(str(cur_v))
                    
                    ShmooTestHarness.reset_and_program_elf(suite.elf)

                    ser = SerialDebug.create(ShmooTestHarness.UART_BAUD_RATE,
                                             timeout=test.timeout)

                    if not ser:
                        raise Exception('Unable to establish a UART serial connection handshake.')

                    # SMU Setup
                    smu.clear_buffers(append=True)
                    smu.write(f"smub.measure.count = 1")
                    smu.set_nplc('0.1')

                    ser.flushInput()
                    ser.flushOutput()

                    smu.start_continuous_capture(gpio='1', buffer_idx=1, timeout=test.timeout,
                                                 voltage=cur_v, freq=cur_clk)
                    
                    ### Begin Host<-->Chip Communication ###

                    # Host sends a signal for the start of header (SOH)
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
                    freq_hz = cur_clk * 1_000_000
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
                    if not ser_data:
                        # Timeout occurred, treat as chip fail
                        ShmooTestHarness.log_as_chip(
                            f'No BEL (7) payload acknowledgment was received within timeout period ({test.timeout} seconds).',
                            red=True)
                        artifact.status = TestStatus.FAIL_NO_BEL
                        ShmooTestHarness.log_test_result(test, artifact)
                        results.add_result(cur_v, freq_hz, artifact)
                        ser.close()
                        max_fail_check(artifact)
                        continue

                    ShmooTestHarness.log_as_chip(
                        f'Sent BEL (7) payload acknowledgment!')
                        
                    # Non-GPIO routine
                    #while not ser.in_waiting:
                    #    smu.write('smub.measure.v(smub.nvbuffer1)')

                    # Chip performs work. Host ignores any UART that is not ETB.
                    # Chip responds with ETB (23)
                    ShmooTestHarness.log_as_host(
                        f'Waiting for ETB (23, 0x17) test completion acknowledgment...')
                    
                    etb_data = ser.read_until(b'\x17')
                    if not etb_data:
                        # Timeout occurred, treat as chip fail
                        ShmooTestHarness.log_as_chip(
                            f'No ETB (23) test completion acknowledgment was received within timeout period ({test.timeout} seconds).',
                            red=True)
                        artifact.status = TestStatus.FAIL_NO_ETB
                        ShmooTestHarness.log_test_result(test, artifact)
                        results.add_result(cur_v, freq_hz, artifact)
                        ser.close()
                        max_fail_check(artifact)
                        continue
                    
                    ShmooTestHarness.log_as_chip(
                        f'Sent ETB (23, 0x17) test completion acknowledgment!')

                    # Chip sends size of payload packet (in bytes) (32-bit int)
                    ShmooTestHarness.log_as_host(
                        f'Awaiting chip payload packet size...')
                    payload_size_bytes = ser.read(4)
                    payload_size = struct.unpack('<i', payload_size_bytes)[0]
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
                    smu_data = smu.retrieve_buffer(1)
                    if not isinstance(smu_data, np.ndarray):
                        ShmooTestHarness.log_as_misc(
                            f'SMU buffer reading failed. Retrying test with {cur_clk}.')
                        pending_freqs_stack.appendleft(cur_clk)
                        continue

                    passed, check_data = test.check_output(context, chip_payload)
                    if passed:
                        artifact.status = TestStatus.PASS
                    else:
                        artifact.status = TestStatus.FAIL_CHECK
                    artifact.check_data = check_data
                    
                    # Parse the data from the SMU and form it into a matrix.
                    
                    LOGGER.debug(f'[SMU Buffer Output] {smu_data}')

                    # Generate and save a CSV of the SMU data.
                    csv_path = ShmooTestHarness.save_data_as_csv(
                        smu_data, artifact.status, output_dir, test.id, cur_v, cur_clk)
                    artifact.csv_path = csv_path
                    
                    # Output appropriate result to the log and keep track of
                    # the result in our results object.
                    ShmooTestHarness.log_test_result(test, artifact)
                    results.add_result(cur_v, freq_hz, artifact)
                    ser.close()

                    max_fail_check(artifact)

        results.log_summary()
        return results


# Describe exports
__all__ = ['SerialDebug', 'SMU', 'ShmooTest', 'ShmooConstantTest', 'TestSuite',
           'ShmooTestHarness']
