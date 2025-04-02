import numpy as np
from collections import deque
import os

# fixed point
def gen_points(frequencies=[0], amplitudes=[256], duration=1, num_points=256):
    t = np.linspace(0, duration, num_points, endpoint=False)
    data_noiseless = sum([(amplitudes[i] * np.cos(2 * np.pi * frequencies[i] * t) ).astype(np.int16) for i in range(len(frequencies))])
    noise = (np.random.normal(0, 10, num_points)).astype(np.int16)
    
    data = data_noiseless # + noise
    encoded_data = [encode_point(i, np.int16(0)) for i in data]

    return encoded_data, data

def encode_point(real, imag):
    hex_val_real = "{:04x}".format(real.view('H'))

    hex_val_imag = "{:04x}".format(imag.view('H'))

    return "0x" + hex_val_imag + hex_val_real

def gen_expected_output(data):
    expected_output = np.fft.fft(data, norm="forward")
    real_output =  np.rint(np.real(expected_output)).astype(np.int16)
    imag_output =  np.rint(np.imag(expected_output)).astype(np.int16)
    expected_output = [encode_point(real_output[i], imag_output[i]) for i in range(num_points)]

    ordering = np.array(fft_ordering(0, 1, len(expected_output)))
    inverse_ordering = np.arange(len(expected_output))[ordering]
    expected_output = [expected_output[i] for i in inverse_ordering]
    return expected_output

def format_c_array(file, list2d):
    file.write("{\n")
    for i in range(len(list2d)):
        file.write("{\n")
        for j in range(len(list2d[i])):
            file.write(f"    {list2d[i][j]}")
            if j != len(list2d[i]) - 1:
                file.write(",\n")
        file.write("\n}")
        if i != len(list2d) - 1:
           file.write(",")
    file.write("\n}")

def fft_ordering(residue, mod, num_points):
    if mod >= num_points:
        return [residue]
    ordering = fft_ordering(residue, 2*mod, num_points) + fft_ordering(residue + mod, 2*mod, num_points)
    return ordering

if __name__ == '__main__':
    num_points = 256

    encoded_datas = []
    expected_outputs = []
    # Generate the data points
    encoded_data, data = gen_points(frequencies=[60], amplitudes=[256], num_points=num_points)
    encoded_datas.append(encoded_data)
    # Expeted FFT output usig Numpy FFT
    expected_output = gen_expected_output(data)
    expected_outputs.append(expected_output)

    # Define the file paths
    file_path = "../data/fft_data.h"
    fft_expected_data_file_path = "../data/fft_expected_data.h"

    # Write the FFT data to the file
    with open(file_path, "w") as file:
        file.write(f"uint32_t fft_len = {num_points};\n\n")
        file.write(f"uint32_t fft_data[{num_points}][{num_points}] = ")

        format_c_array(file, encoded_datas)
        file.write(";")

    # Write the expected FFT output to the file "fft_expected_data"
    with open(fft_expected_data_file_path, "w") as file:
        file.write("#ifndef FFT_EXPECTED_DATA_H\n")
        file.write("#define FFT_EXPECTED_DATA_H\n\n")
        file.write("#include <stdint.h>\n\n")
        file.write(f"uint32_t fft_expected_data[{num_points}][{num_points}] = ")
        format_c_array(file, expected_outputs)
        file.write(";\n")
        file.write("#endif")

    # Confirm the files have been written
    if os.path.exists(file_path):
        print("fft_data.h has been successfully created.")
    else:
        print("An error occurred while creating fft_data.h.")

    if os.path.exists(fft_expected_data_file_path):
        print("fft_expected_data.h has been successfully created.")
    else:
        print("An error occurred while creating fft_expected_data.h.")
