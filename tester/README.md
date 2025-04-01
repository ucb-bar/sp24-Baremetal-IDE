# Jasmine Angle's ShmooTester

## Setup

### Python Environment

Create a virtual environment using the package manager of your choice from the provided requirements.txt

For example, using `venv`:
```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### PSU

The PSU will require some preliminary setup to ensure connectivity. Within `utils.py`, edit the following class values within the PSU class:

- **`IDN`**: The IDN for the PSU you are using.
- **`VISA_PATH`**: The GPIB LAN path for the PSU for a remote interface.

## Usage
```
usage: Jasmine's ShmooTester [-h] [-i INPUT] [-o OUTPUT] [-s SUITE] [-t TESTS] [--min-v MIN_V] [--max-v MAX_V] [--step-v STEP_V] [--min-freq MIN_FREQ] [--max-freq MAX_FREQ] [--step-freq STEP_FREQ] [--max-v-fail MAX_V_FAIL] [--retries RETRIES]
                             [-r NUM_RUNS] [-f] [--psu-mode {INT,EXT}] [--psu-channel PSU_CHANNEL] [--no-psu] [-m] [-d] [-n] [-l] [--log {DEBUG,INFO,WARNING,ERROR,CRITICAL}] [--logfile LOGFILE]

Performs Shmoo testing using the default BEL/ETB Bringup communication protocol.

options:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        Path to an existing Shmoo test run to import for Shmoo plot generation. If specified, this will only generate a Shmoo plot from existing data and not run any tests.
  -o OUTPUT, --output OUTPUT
                        Path for storing output files. If the path does not already exist, it will be created. If unspecified, a default path containing the suite name and test start timestamp will be created.

Shmoo Testing Options:
  -s SUITE, --suite SUITE
                        Name of the test suite to run
  -t TESTS, --test TESTS
                        ID of the test you wish to run (as a decimal number). This argument can be passed multiple times to run multiple specific tests within a test suite. If unspecified, all tests will run.
  --min-v MIN_V         Voltage lower bound
  --max-v MAX_V         Voltage upper bound (exclusive)
  --step-v STEP_V       Voltage step size
  --min-freq MIN_FREQ   Frequency lower bound (in MHz)
  --max-freq MAX_FREQ   Frequency upper bound (in MHz, exclusive)
  --step-freq STEP_FREQ
                        Frequency step size (in MHz)
  --max-v-fail MAX_V_FAIL
                        Maximum number of consecutive failures until the tester changes to a new voltage.
  --retries RETRIES     Maximum number of retries permitted for a single frequency test at a given voltage. Retries are disabled (0) by default.
  -r NUM_RUNS, --num-runs NUM_RUNS
                        Number of times to re-run the test for a given attempt. Only the final test run will have data captured. Any fail within these runs will fall back to the specified `attempts` count. Defaults to 1.
  -f, --force           Force start without a confirmation of limits. Only use this setting if you are absolutely sure that the testbench setup is correct and you are aware of the assigned limits.

PSU Configuration Options:
  --psu-mode {INT,EXT}  Mode to set the PSU to sense with. `EXT` for 4-wire remote sense, `INT` for 2-wire local sense. Defaults to INT.
  --psu-channel PSU_CHANNEL
                        Channel to use for the PSU (1-3). Defaults to 1.
  --no-psu              Disables sending any commands to the PSU and instead redirects all SCPI commands to the log.

Debugging Options:
  -m, --mock-test       Only mock run the host payload creation function of a test suite
  -d, --debug           Enables debugging mode, disabling all serial timeouts and allowing the user to run through a Shmoo test step-by-step.
  -n, --no-upload       Disables OpenOCD reset and program of the chip. Useful for debugging with an external OpenOCD+GDB configuration.
  -l, --list-suites     Ignore all other commands and print a list of test suites

Logging Options:
  --log {DEBUG,INFO,WARNING,ERROR,CRITICAL}
                        Set the logging level
  --logfile LOGFILE     Path to an optional log file for terminal capture. If this setting is used, STDOUT will only be used for user prompts.

Created by Jasmine Angle (angle@berkeley.edu)
```
