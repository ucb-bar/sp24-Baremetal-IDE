# BearlyML'24 BorAI (Baremetal Llama2 for RISC-V)

This is a port of the [llama2.c](https://github.com/karpathy/llama2.c/tree/master) inferencing system created by the Fall 2024 EE 194 Berkeley IC Design Project Bringup course to work with the BearlyML'24 tapeout chip from Spring 2024.

## Compiling and Running BorAI (with Float32 Matmul)

```bash
make borai
```

This will produce an ELF binary file at `./build/borai/borai.elf`. A JTAG interface can then be initialized using

```bash
make ocd CHIP=bearly24
```

This command will automatically start an [OpenOCD](https://openocd.org/) server with the proper BearlyML'24 chip configuration. Hart IDs 0-3 will be mapped to ports 3333-3336 on your machine. We can then initialize a GDB interface with a server connection to Hart 2 using
```
make gdb BINARY=./build/borai/borai.elf PORT=3335
```

Upon running `load` in GDB, the binary will be loaded to the chip and ready for testing.

## Compiling and Running BorAIq (with Int8 Matmul)

```bash
make boraiq
```

This will produce an ELF binary file at `./build/borai/boraiq.elf`. A JTAG interface can then be initialized using

```bash
make ocd CHIP=bearly24
```

This command will automatically start an [OpenOCD](https://openocd.org/) server with the proper BearlyML'24 chip configuration. Hart IDs 0-3 will be mapped to ports 3333-3336 on your machine. We can then initialize a GDB interface with a server connection to Hart 2 using
```bash
make gdb BINARY=./build/borai/boraiq.elf PORT=3335
```


Upon running `load` in GDB, the binary will be loaded to the chip and ready for testing.

### Int8 Accelerators

The two primarily supported accelerators for BorAI are the Quantized Transformer's `V_DOTPROD` function, as well as the BearlyML'24 DMA MAC function. BorAI has been written such that these can both be enabled or disabled using `#define` directives within `hardware.h`.

```c
// Hardware Enable //
#define ENABLE_QT_DOTPROD
#define ENABLE_DMA_MATVEC
```

Removing either or both of these lines will remove all code pertaining to that accelerator, falling back to a naive C implementation where possible. This is particularly useful for scoped debugging or for limiting the program's text size.

## Converting Binary Files to Headers

Within the `scripts` directory, you can use the `bin2array.py` command-line tool to convert a binary file into a C header array.

```
usage: bin2array [-h] -b BINARY -o OUTPUT -n VARNAME [-c ROWCOUNT]

Converts a binary file into a C array.

options:
  -h, --help            show this help message and exit
  -b BINARY, --binary BINARY
                        Path to the binary file to convert.
  -o OUTPUT, --output OUTPUT
                        Output path for the C header file.
  -n VARNAME, --varname VARNAME
                        Name of the C array variable to declare inside the header file.
  -c ROWCOUNT, --rowcount ROWCOUNT
                        Optional number of elements to store per line within the C array.
```

We can, for example, convert weights in the following manner:

```bash
./bin2array.py -b ./stories260K.bin -o ./weights.h -n WEIGHTS
```

This will create a header file with a single variable of the name `WEIGHTS`, which contains the data stored within `./stories260K.bin`.
