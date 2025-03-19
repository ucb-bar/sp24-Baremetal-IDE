

class SMU:
    """
    Wrapper class for all PyVisa SMU commands and data acquisition management.
    """

    SMU_IDN = 'Keithley Instruments Inc., Model 2602A'
    """
    Identification string to check for valid equipment setup.
    """
    
    INIT_CURRENT_LIMIT = "3"
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
        display.settext("LimV: {float_to_str(voltage)} V | Freq: {freq} MHz")
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
        display.settext("LimV: {float_to_str(voltage)} V | Freq: {freq} MHz")'''
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

