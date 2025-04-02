#!/bin/bash

### Generate input frequencies ###

# 20 random numbers from 0 to 256
for ((i=0; i<20; i++)); do
    random_number=$((RANDOM % 257))  
    frequencies+=("$random_number")
done

# 0 to 256
# for ((i=0; i<256; i++)); do 
#     frequencies+=("$i")
# done

### Generate the input and output header files ###

## Run the Python script with the generated numbers as arguments
# python fft_auto_pointgen.py "${frequencies[@]}"

## twinkle twinkle little star, c c g g a a g f f e e d d c
# c = 131 g = 196 a = 220 f = 175 e = 165 d = 147
# python fft_auto_pointgen.py 131 131 196 196 220 220 196 175 175 165 165 147 147 131

## Run simulations - old tapeout setup ##
## Assumes you've already compiled tests

cd ../../../sims/vcs
bsub -I -XF -q ee194 make -j16 CONFIG=CombinedConfig BINARY=../../tests/combined-testing/fft_test.riscv run-binary-debug-hex
