# from scipy.fft import fft
import numpy as np
import os

def gen_points(frequency=0, amplitude=100, duration=1, num_points=512):
    # t = np.array(range(num_points))/512
    t = np.linspace(0, duration, num_points, endpoint=False)
    data_1 = (amplitude * np.cos(2 * np.pi * frequency * t) ).astype(np.int16)
    noise = (np.random.normal(0,10,512)).astype(np.int16)
    
    data = data_1 + noise
    print(data)
    hex_x = [gen_point(i, 0) for i in data]

    return hex_x, data

def gen_point(real, imag):
    bin_val_real = bin(np.int16(real).view('H'))[2:].zfill(16)
    hex_val_real = "{:04x}".format(int(bin_val_real, 2), 6)

    bin_val_imag = bin(np.int16(imag).view('H'))[2:].zfill(16)
    hex_val_imag = "{:04x}".format(int(bin_val_imag, 2), 6)

    return "0x" + hex_val_real

# Generate the data points
hex_values, data_values = gen_points(frequency=256)

# Define the file path
file_path = "fft_data.h"

# Write the data to the file
with open(file_path, "w") as file:
    file.write("uint32_t fft_len = 512;\n\n")
    file.write("uint32_t fft_data[512] = {\n")
    for i in range(512):
        file.write("  " + hex_values[i])
        if i != 511:
            file.write(",\n")
    file.write("\n};")

# Confirm the file has been written
if os.path.exists(file_path):
    print("fft_data.h has been successfully created.")
else:
    print("An error occurred while creating fft_data.h.")