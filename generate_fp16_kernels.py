import numpy as np
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("M", help="number of kernels to generate", type=int)
parser.add_argument("N", help="length of kernel", type=int)
args = parser.parse_args()
# generate M kernels each with N entries in IEEE FP16 format
def generate_float16_binary_header(M, N, filename):
    with open(filename, 'w') as f:
        f.write("#ifndef KERNELSFP16_H\n")
        f.write("#define KERNELSFP16_H\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"const uint16_t kernelsFP16[{M}][{N}] = {{\n")
        for i in range(M):
            f.write("\t{\n")
            for j in range(N):
                sign = np.random.randint(0, 2)
                exponent = np.random.randint(0, 2**5-1)   # exclude INF and NaN
                mantissa = np.random.randint(0, 2**10)
                float_bits = (sign << 15) | (exponent << 10) | mantissa
                mantissa_ext = 1 + mantissa / (2**10)
                if exponent == 0:
                    mantissa_ext -= 1
                float16_num = np.float16(((-1)**sign) * 2**(exponent-15) * mantissa_ext)
                f.write(f"\t\t// Number: Sign bit: {sign}, Exponent: {exponent}, Float16: {float16_num}\n")
                # print(f"Number: Sign bit: {sign}, Exponent: {exponent}, Binary: {float_bits:016b}, Float16: {float16_num}")
                if j == N-1:
                    f.write(f"\t\t0b{float_bits:016b}\n")
                else:
                    f.write(f"\t\t0b{float_bits:016b},\n")
            if i == M-1:
                f.write("\t}\n")
            else:
                f.write("\t},\n")
        f.write("};\n\n#endif /* KERNELSFP16_H */\n")
generate_float16_binary_header(args.M, args.N, "kernelsFP16.h")