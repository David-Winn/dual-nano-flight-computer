import struct, csv, os, re

SAMPLE_FMT = '6fI'  # 6 floats + 1 uint32 (second uint32 is padding to reach 32 bytes)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
DATA_DIR = '/Volumes/flight_data'

def find_latest_data_file(directory):
    """Return the path to the most recently created data file (data.txt, data1.txt, data2.txt, ...)."""
    candidates = []
    for name in os.listdir(directory):
        if name.upper() == 'DATA.TXT':
            candidates.append((0, name))
        else:
            m = re.fullmatch(r'data(\d+)\.txt', name, re.IGNORECASE)
            if m:
                candidates.append((int(m.group(1)), name))
    if not candidates:
        raise FileNotFoundError(f"No data files found in {directory}")
    _, latest = max(candidates, key=lambda x: x[0])
    return os.path.join(directory, latest)

input_path = find_latest_data_file(DATA_DIR)
output_path = os.path.join(DATA_DIR, 'parsed_data.csv')

print(f"Parsing: {input_path}")

with open(input_path, 'rb') as f, open(output_path, 'w', newline='') as out:
    writer = csv.writer(out)
    writer.writerow(['xA', 'yA', 'zA', 'xG', 'yG', 'zG', 'timestamp'])
    while chunk := f.read(SAMPLE_SIZE):
        if len(chunk) < SAMPLE_SIZE:
            break  # Skip incomplete trailing sample
        fields = struct.unpack(SAMPLE_FMT, chunk)
        writer.writerow(fields[:7])  # Discard padding word

print(f"Done -> {output_path}")