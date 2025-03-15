import sys
import os
import numpy as np

datapath = sys.argv[1]
outfile = sys.argv[2]
filenames = sorted(os.listdir(datapath))

datasets = {}
for f in filenames:
    name_components = f.split('_')
    volt = float(name_components[2][:-2])
    freq = int(name_components[3][:-7])
    if volt not in datasets:
        datasets[volt] = {}

    dat = np.genfromtxt(os.path.join(datapath, f), delimiter=',').T
    powers = np.multiply(dat[0], dat[2])
    dt = np.diff(dat[1])
    energy = np.sum(np.multiply(powers[:-1], dt)) 
    avg_power = energy / dat[1][-1]
    datasets[volt][freq] = {"power":avg_power, "energy":energy}

with open(outfile, "w") as datfile:
    for voltage, freqs in datasets.items():
        freqs = dict(sorted(freqs.items()))
        for freq, dat in freqs.items():
            datfile.write(f'{voltage}, {freq}, {dat["power"]}, {dat["energy"]}\n')
