import sys

data_file = sys.argv[1]
names_file = sys.argv[2]
n_files = int(sys.argv[3])

# Load data
data = {}
# Try different encodings - batch files on Windows may use cp1252, cp850, or utf-8
for encoding in ['utf-8', 'cp1252', 'cp850', 'latin-1']:
    try:
        with open(data_file, 'r', encoding=encoding) as f:
            for line in f:
                parts = line.rstrip('\n\r').split('\t')
                if len(parts) < 8:
                    continue
                idx = int(parts[0])
                fb = parts[1]
                name = parts[2]
                status, thr, avg, minv, maxv = parts[3], parts[4], parts[5], parts[6], parts[7]
                if idx not in data:
                    data[idx] = {}
                data[idx][name] = [status, thr, avg, minv, maxv]
        break
    except (UnicodeDecodeError, ValueError):
        data = {}
        continue

# Load names
names = []
for encoding in ['utf-8', 'cp1252', 'cp850', 'latin-1']:
    try:
        with open(names_file, 'r', encoding=encoding) as f:
            names = [line.rstrip('\n\r') for line in f]
        break
    except UnicodeDecodeError:
        names = []
        continue

# Headers
headers = ['File', 'Benchmark Name', 'Status', 'Threshold (ms)', 'Avg (ms)', 'Min (ms)', 'Max (ms)']
widths = [max(4, len(h)) for h in headers]

# Compute widths
for name in names:
    if len(name) > widths[1]:
        widths[1] = len(name)
    for i in range(1, n_files + 1):
        if i in data and name in data[i]:
            for k, val in enumerate(data[i][name]):
                col = 2 + k
                if len(val) > widths[col]:
                    widths[col] = len(val)
        if len(str(i)) > widths[0]:
            widths[0] = len(str(i))

# Print header
header_line = ' | '.join([headers[i].ljust(widths[i]) for i in range(len(headers))])
print(header_line)

# Print separator
sep_line = ' | '.join(['-' * max(3, widths[i]) for i in range(len(headers))])
print(sep_line)

# Calculate percentage differences for sorting
name_diffs = []
for name in names:
    diff_pct = float('inf')  # Default to infinity if can't calculate
    if 1 in data and name in data[1] and 2 in data and name in data[2]:
        try:
            avg1 = float(data[1][name][2])  # avg is at index 2
            avg2 = float(data[2][name][2])
            if avg1 > 0:
                # Calculate percentage difference: ((avg2 - avg1) / avg1) * 100
                diff_pct = ((avg2 - avg1) / avg1) * 100
        except (ValueError, ZeroDivisionError):
            pass
    name_diffs.append((name, diff_pct))

# Sort by absolute value of percentage difference (descending)
name_diffs.sort(key=lambda x: abs(x[1]), reverse=True)
sorted_names = [name for name, _ in name_diffs]

# Print rows
for name in sorted_names:
    print()
    for i in range(1, n_files + 1):
        row = [str(i).ljust(widths[0]), name.ljust(widths[1])]
        if i in data and name in data[i]:
            row.extend([data[i][name][k].ljust(widths[2+k]) for k in range(5)])
        else:
            row.extend([''.ljust(widths[2+k]) for k in range(5)])
        print(' | '.join(row))
