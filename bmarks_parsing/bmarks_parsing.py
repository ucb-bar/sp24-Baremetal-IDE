## Run as:
# python3 bmarks_parsing.py  --folder /tools/C/srevi/sp24-Baremetal-IDE-fft/bmarks_parsing/data_fft-accel-bmarks --output result_summary.csv

import os
import csv
import re
import ast
import argparse
from collections import defaultdict


def extract_frequency(filename):
    match = re.search(r'_(\d+(?:\.\d+)?)MHz', filename)
    return float(match.group(1)) * 1e6 if match else None


def load_result_tsv(tsv_path):
    result_map = {}
    with open(tsv_path, 'r') as f:
        reader = csv.DictReader(f, delimiter='\t')
        for row in reader:
            try:
                voltage = float(row['Voltage'])
                frequency = int(row['Frequency'])
                status = row['Status']
                compare_data = row['Compare Check Data']

                # Primary format
                if status == 'PASS' and compare_data and compare_data != 'None':
                    try:
                        data_dict = ast.literal_eval(compare_data)
                        if 'vector_cycles' in data_dict:
                            result_map[(voltage, frequency)] = int(data_dict['vector_cycles'])
                            continue
                    except Exception:
                        pass

                # Fallback: e.g. "123456789 cycles"
                match = re.search(r'(\d+)\s+cycles', compare_data)
                if match:
                    result_map[(voltage, frequency)] = int(match.group(1))

            except Exception:
                continue
    return result_map


def process_csv_file(filepath, filename, result_map):
    frequency_hz = extract_frequency(filename)
    if frequency_hz is None:
        return None

    frequency_mhz = frequency_hz / 1e6
    voltage_match = re.search(r'_(\d+\.\d+)v_', filename)
    if not voltage_match:
        return None
    voltage = float(voltage_match.group(1))

    cycles = result_map.get((voltage, int(frequency_hz)))
    if not cycles:
        return None

    currents = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 3:
                try:
                    currents.append(float(row[2]))
                except ValueError:
                    continue

    if not currents:
        return None

    avg_current = sum(currents) / len(currents)
    power_w = voltage * avg_current
    time_s = cycles / frequency_hz
    energy_j = power_w * time_s

    return {
        'filename': filename,
        'voltage': voltage,
        'frequency_mhz': frequency_mhz,
        'cycles_million': cycles / 1e6,
        'avg_current': avg_current,
        'avg_power_mw': power_w * 1000,
        'time_s': time_s,
        'energy_mj': energy_j * 1000
    }


def build_data(folder, result_map):
    all_records = []
    grouped = defaultdict(list)

    for filename in os.listdir(folder):
        if filename.endswith('.csv') and not filename.endswith('summary.csv'):
            filepath = os.path.join(folder, filename)
            record = process_csv_file(filepath, filename, result_map)
            if record:
                all_records.append(record)
                group_key = (record['voltage'], filename.split('_')[1])
                grouped[group_key].append(record)

    for key in grouped:
        grouped[key] = sorted(grouped[key], key=lambda r: r['frequency_mhz'])

    return all_records, grouped


def find_extremes_with_context(all_records, grouped_records):
    min_energy = min(all_records, key=lambda r: r['energy_mj'])
    min_time = min(all_records, key=lambda r: r['time_s'])

    def get_context(record):
        group_key = (record['voltage'], record['filename'].split('_')[1])
        records = grouped_records[group_key]
        idx = next(i for i, r in enumerate(records) if r['frequency_mhz'] == record['frequency_mhz'])
        context = []
        if idx > 0:
            context.append(records[idx - 1])
        context.append(records[idx])
        if idx < len(records) - 1:
            context.append(records[idx + 1])
        return context

    return get_context(min_energy), get_context(min_time)


def save_summary_csv(eff_context, perf_context, output_path):
    def label(records, best, label):
        rows = []
        for r in records:
            kind = f"{label} (best)" if r == best else f"{label} (context)"
            rows.append([
                kind, r['filename'], r['voltage'], r['frequency_mhz'],
                r['cycles_million'], r['avg_current'], r['avg_power_mw'],
                r['time_s'], r['energy_mj']
            ])
        return rows

    best_eff = min(eff_context, key=lambda r: r['energy_mj'])
    best_perf = min(perf_context, key=lambda r: r['time_s'])

    rows = label(eff_context, best_eff, "High Efficiency") + label(perf_context, best_perf, "High Performance")

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Type', 'Filename', 'Voltage (V)', 'Frequency (MHz)', 'Cycles (M)',
            'Avg Current (A)', 'Avg Power (mW)', 'Time (s)', 'Energy (mJ)'
        ])
        writer.writerows(rows)

    print(f"\n✅ Summary written to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Analyze CSVs for energy and performance.')
    parser.add_argument('--folder', required=True, help='Path to folder containing CSVs and result.tsv')
    parser.add_argument('--output', help='Path to output summary CSV')

    args = parser.parse_args()
    folder = args.folder
    output = args.output or os.path.join(folder, 'summary.csv')

    result_path = os.path.join(folder, 'result.tsv')
    if not os.path.exists(result_path):
        print(f"❌ result.tsv not found in: {folder}")
        return

    result_map = load_result_tsv(result_path)
    all_records, grouped = build_data(folder, result_map)

    if not all_records:
        print("⚠️ No valid records found.")
        return

    eff_context, perf_context = find_extremes_with_context(all_records, grouped)
    save_summary_csv(eff_context, perf_context, output)


if __name__ == '__main__':
    main()
