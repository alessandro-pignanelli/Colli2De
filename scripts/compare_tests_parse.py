import sys
import re
import os

def trim(s):
    return s.strip()

def is_sep_line(line):
    return bool(re.match(r'^[ \t\-|]+$', line))

def main():
    if len(sys.argv) < 3:
        print("Usage: compare_tests_parse.py <output_file> <file1.md> [file2.md ...]", file=sys.stderr)
        sys.exit(1)
    
    output_file = sys.argv[1]
    input_files = sys.argv[2:]
    
    try:
        with open(output_file, 'w', encoding='utf-8') as out_f:
            file_idx = 0
            for filepath in input_files:
                file_idx += 1
                filebase = os.path.basename(filepath)
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        for line in f:
                            line = line.rstrip('\n\r')
                            if is_sep_line(line):
                                continue
                            if not '|' in line:
                                continue
                            parts = line.split('|')
                            # Drop trailing empty fields
                            while parts and trim(parts[-1]) == '':
                                parts.pop()
                            if len(parts) < 6:
                                continue
                            # Extract values from right to left
                            maxv = trim(parts[-1])
                            minv = trim(parts[-2])
                            avg = trim(parts[-3])
                            thr = trim(parts[-4])
                            status = trim(parts[-5])
                            # Remaining parts form the name
                            name_parts = [trim(p) for p in parts[:-5] if trim(p)]
                            name = ' | '.join(name_parts)
                            name = trim(name)
                            if re.search(r'^[Bb]enchmark[ ]+[Nn]ame$', name):
                                continue
                            if not name:
                                continue
                            out_f.write(f'{file_idx}\t{filebase}\t{name}\t{status}\t{thr}\t{avg}\t{minv}\t{maxv}\n')
                except Exception as e:
                    print(f"Error processing {filepath}: {e}", file=sys.stderr)
                    sys.exit(1)
    except Exception as e:
        print(f"Error writing to {output_file}: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
