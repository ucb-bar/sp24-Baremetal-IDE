# BorAI Block-Level Testing

*Jasmine Angle*

Because there have been some interesting behaviors observed with the chip's blocks, this folder contains some extra testing that goes beyond what the tapeout class team had used. Each test contains a `results.txt` file, which represents the output from running the test on the respective chip.

## BearlyML'24 Quantized Transformer Dot Product (`qtdp`)

For the quantized transformer dot product functionality, we observed strange behavior when it came to integration with the BorAI `matmul` function. The tests within this directory check various behavior with different Int8 values.

### Testing Methodology

This test was written to mimic the following Behavior-Driven Development approach:

```gherkin
Given two vectors with eight int8_t values
And a valid scale factor value
When the dot product is executed
Then the dot product destination register should equal the naive result
```

### Running the Tests

In the root of this repository, run the following command:
```bash
make build TARGET=borai_qtdp CHIP=bearly24
```

In a separate terminal, ensure that OpenOCD is running by using:
```bash
make ocd CHIP=bearly24
```

Then, use JTAG to program the chip on hart 2 using
```bash
make gdb PORT=3335 BINARY=./build/borai_block_tests/borai_qtdp.elf
```

### Observations

Unfortunately, for our purposes, we will not be able to use this block for our demonstration. As evidenced by the `Unordered int8 unsigned values` test, the value is automatically quantized to an int8_t value, which means that a dot product must have a resulting value between [-128, 127] to be correct. Because the V_DOTPROD function automatically passes its value into the requantizer, there is no way to retrieve these lost upper bits unless a proper scale factor is computed ahead of time. Our matrix multiplication relies upon values that will almost always have this scenario occur, meaning that for our large, dense matrices, the Quantized Transformer Dot Product will output the wrong value for every computation using a simple scale factor of 1, 0.

## BearlyML'24 DMA Matrix-Vector Multiplication Engine (`dma_mac`)