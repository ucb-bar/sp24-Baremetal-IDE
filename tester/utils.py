"""
Shmoo testing utilities for use with plugins.
"""
from datetime import datetime
import logging
import os
import struct
from time import sleep
from typing import *
from typing_extensions import Self
from collections import OrderedDict

import pyvisa
import serial  # PySerial
from serial.tools import list_ports
import numpy as np

# I'm too tired to use vt100 commands
from colorama import Fore, Style

LOGGER = logging.getLogger(__name__)


class SerialDebug(serial.Serial):
    def __init__(self, *args, **kwargs):
        self.logger = logging.getLogger(__name__)
        super().__init__(*args, **kwargs)
        self.logger.debug(f"Serial port {self.port} initialized with settings: {self.get_settings()}")

    @staticmethod
    def create(baud_rate) -> Union[Self, None]:
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
                    tmp.timeout = None
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
    
    def __init__(self):
        self.rm = pyvisa.ResourceManager("@py")
        self._smu = self.rm.open_resource(self.VISA_PATH)

        # Verify that we have the correct equipment
        smu_verification = self.query("*IDN?")
        LOGGER.debug(f'SourceMeter: {smu_verification}')

        if not smu_verification.startswith(self.SMU_IDN):
            raise Exception(f'Attempt to connect to `{self.SMU_IDN}` at {self.VISA_PATH} failed, IDN returned `{smu_verification}` instead.')

        # Reset with our defaults
        # self.reset()

    def query(self, querystr: str) -> str:
        """
        Sends a query over GPIB to the SMU, expecting a result. This can be a
        multiline script query.

        Args:
            querystr (str): The query to send to the SMU.

        Returns:
            str: Data returned by the SMU.
        """
        return self._smu.query(querystr)

    def write(self, cmdstr: str) -> str:
        """
        Writes a command over GPIB to the SMU. This can be a multiline script.

        Args:
            cmdstr (str): The command to send to the SMU.

        Returns:
            str: Data returned by the SMU.
        """
        return self._smu.write(cmdstr)

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

    def clear_buffer(self, idx, src_vals=True, timestamps=True, append=True):
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
        self.write(f"smub.nvbuffer{idx}.clear()")
        self.write(f"smub.nvbuffer{idx}.collectsourcevalues = {src_vals}")
        self.write(f"smub.nvbuffer{idx}.collecttimestamps = {timestamps}")
        self.write(f"smub.nvbuffer{idx}.appendmode = {append}")

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
        self.write("smub.measure.nplc = {nplc}")

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
            self.clear_buffer(1)
            self.clear_buffer(2)

        # Capture count / integration aperture
        self.write("smub.measure.count = 1")
        self.set_nplc('0.1')

    def start_continuous_capture(self, gpio: Union[int, str], buffer_idx: int,
                                 stop_val: int = 1):
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
            stop_val (int, optional): The comparison value for the digital I/O
                pin. If 0, the measurement halts when the bit is low, and vice
                versa for high with 1. Defaults to 1.
        """
        buffer = f'smub.nvbuffer{buffer_idx}'
        gpib_cmd = f'while (digio.readbit({gpio}) == 0) do smub.measure.p({buffer}) end'
        self.write(gpib_cmd)

    def retrieve_buffer(self, idx: int) -> str:
        """
        Retrieves content from a data capture buffer on the SMU.

        Content is ordered as follows:
        meas1, timstamp1, srcval1

        Args:
            idx (int): 1 or 2 depending on the buffer you wish to read from.

        Returns:
            str: Buffer contents
        """
        self.write(f"rb1 = smub.nvbuffer{idx}")
        return self.query(f"printbuffer({idx}, rb1.n, rb1, rb1.timestamps, rb1.sourcevalues)")
    
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

    def __init__(self, name: str, id: int, *args, **kwargs):
        self.name = name
        if id > 0xff:
            raise Exception(f'Test ID {id} for test {name} exceeds max 8-bit unsigned integer value.')
        self.id = id

    def create_payload(self) -> tuple[bytes, dict]:
        """_summary_

        Args:
            context (_type_): _description_

        Returns:
            tuple[bytes, dict]: _description_
        """
        return "Hello, Chip!", {}

    def check_output(self, context: dict, value: bytes) -> bool:
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
            bool: True if the test has "passed" according to the test output,
                False otherwise.
        """
        return value == "Hey, Host!"


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

    def check_output(self, context: dict, value: bytes) -> bool:
        return value == self.expect


class TestSuite:
    """
    Describes all tests that are available for a given program. Test suites
    should be defined by the plugin associated with the program running on
    the chip.
    """

    def __init__(self, *args):
        self.tests = OrderedDict()
        for test in args:
            self.tests[test.id] = test


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
    def register_test_suite(name: str, suite: TestSuite):
        ShmooTestHarness.TEST_SUITES[name] = suite

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
    def log_as_chip(val: str):
        LOGGER.info(f"{Fore.BLUE}{Style.BRIGHT}[Chip]{Style.RESET_ALL} %s", val)

    @staticmethod
    def log_as_misc(val: str):
        LOGGER.info(f"{Fore.YELLOW}{Style.BRIGHT}[Misc]{Style.RESET_ALL} %s", val)

    @staticmethod
    def run_suite(suite_name: str, voltages: list, frequencies: list):
        """
        Runs a test with the full flow, along with serial instantiation, for
        a given test harness.

        Args:
            suite_name (str): The name of the registered suite to run.
        """
        LOGGER.info(f'{Style.BRIGHT}{Fore.YELLOW}--- Starting enumeration for test suite "{suite_name}" ---{Style.RESET_ALL}')
        # Initialize hardware connections
        smu = SMU()
        ser = SerialDebug.create(ShmooTestHarness.UART_BAUD_RATE)
        if not ser:
            raise Exception('Unable to establish a UART serial connection handshake.')
        
        # Retrieve the correct test suite to run.
        suite = ShmooTestHarness.TEST_SUITES[suite_name]

        output_dir = f'data_{suite_name}_{datetime.now().isoformat()}'
        os.mkdir(output_dir)
        ShmooTestHarness.log_as_misc(f'Output will be stored within "{output_dir}/"')

        for cur_v in voltages:
            cur_v = float(cur_v)
            for cur_clk in frequencies:
                cur_clk = int(cur_clk)
                # TODO: Sweep the ranges and make run_suite take it as input.
                smu.set_voltage_limit(str(cur_v))
                sleep(0.5)

                for _, test in suite.tests.items():
                    LOGGER.info(f'{Style.BRIGHT}{Fore.MAGENTA}--- [Test ID {test.id}] Running at {cur_clk} MHz and {cur_v} V ---{Style.RESET_ALL}')

                    # SMU Setup
                    smu.clear_buffer(1, append=True)
                    smu.write(f"smub.measure.count = 1")
                    smu.write(f"smub.measure.nplc = 0.1")

                    ser.flushInput()
                    ser.flushOutput()

                    # Host sends a signal for the start of header (SOH)
                    ShmooTestHarness.log_as_host("Sending Start of Header (SOH)")
                    ser.write(b'\x01')

                    # Host sends the size of the data packet
                    host_to_chip_payload, context = test.create_payload()
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
                    # TODO: Add timeout for tests to error when no BEL is received.
                    ShmooTestHarness.log_as_host(
                        f'Waiting for BEL (7, 0x07) payload acknowledgment...')
                    ser.read_until(b'\x07')
                    ShmooTestHarness.log_as_chip(
                        f'Sent BEL (7) payload acknowledgment!')

                    #smu.start_continuous_capture(gpio=1, buffer_idx=1)
                    # Non-GPIO routine
                    while not ser.in_waiting:
                        smu.write('smub.measure.v(smub.nvbuffer1)')

                    # Chip performs work. Host ignores any UART that is not ETB.
                    # Chip responds with ETB (23)
                    # TODO: Add timeout for tests to error when no ETB is received.
                    ShmooTestHarness.log_as_host(
                        f'Waiting for ETB (23, 0x17) test completion acknowledgment...')
                    ser.read_until(b'\x17')
                    ShmooTestHarness.log_as_chip(
                        f'Sent ETB (23, 0x17) test completion acknowledgment!')
                    smu_data = smu.retrieve_buffer(1)
                    LOGGER.debug(f'[SMU Buffer Output] {smu_data}')

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
                    ShmooTestHarness.log_as_chip(
                        f'Payload packet: {chip_payload}')

                    test_passed = test.check_output(context, chip_payload)
                    status_str = ShmooTestHarness.TEST_PASSED_STR \
                        if test_passed else ShmooTestHarness.TEST_FAILED_STR
                    
                    power_data = np.fromstring(smu_data, dtype=float, sep=",")
                    power_data = power_data.reshape((-1, 3))

                    fname_pf_str = 'PASS' if test_passed else 'FAIL'
                    filename = f'{output_dir}/test_{test.id}_{cur_v}v_{cur_clk}MHz_{fname_pf_str}.csv'
                    np.savetxt(filename, power_data, delimiter=",")
                    
                    LOGGER.info(f'{status_str} Test ID {test.id} [{test.name}] (SMU Data File: {filename})')


# Describe exports
__all__ = ['SerialDebug', 'SMU', 'ShmooTest', 'ShmooConstantTest', 'TestSuite',
           'ShmooTestHarness']
