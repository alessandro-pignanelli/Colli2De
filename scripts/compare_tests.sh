#!/usr/bin/env bash
set -euo pipefail

# compare_tests.sh
# Merge two or more benchmark markdown tables (same format, arbitrary row order)
# into a single markdown table. Benchmarks are sorted by name and, for each
# input file N, columns are prefixed with "N." (e.g., 1.Status, 2.Avg (ms), ...).
#
# Usage:
#   ./compare_tests.sh [-o output.md] [-n COUNT] [file1.md file2.md ...]
# If -n is provided, the script ignores positional files and merges the last COUNT
# benchmark files from test_data (newest first). If -o is not provided, the merged
# table is printed to stdout.

usage() {
  echo "Usage: $0 [-o output.md] [-n COUNT] [file1.md file2.md ...]" >&2
  echo "  -n COUNT  Merge the last COUNT files from test_data matching benchmarks--*.md (newest first)." >&2
}

out=""
ncount=""
while getopts ":o:n:h" opt; do
  case "$opt" in
    o) out="$OPTARG";;
    n) ncount="$OPTARG";;
    h) usage; exit 0;;
    \?) echo "Invalid option: -$OPTARG" >&2; usage; exit 1;;
    :) echo "Option -$OPTARG requires an argument." >&2; usage; exit 1;;
  esac
done
shift $((OPTIND-1))

files=()

if [ -n "$ncount" ]; then
  # Validate ncount is a positive integer >= 2
  if ! echo "$ncount" | grep -Eq '^[0-9]+$'; then
    echo "Error: -n requires a positive integer" >&2; exit 1
  fi
  if [ "$ncount" -lt 2 ]; then
    echo "Error: -n COUNT must be at least 2" >&2; exit 1
  fi
  # Collect last COUNT benchmark files matching the standard naming pattern
  # (by lexicographic order -> timestamp in name), newest first. Portable: avoid mapfile and tac
  while IFS= read -r fp; do
    files+=("$fp")
  done < <(ls -1 ../test_data/benchmarks--*.md 2>/dev/null \
           | LC_ALL=C sort \
           | tail -n "$ncount" \
           | awk '{ a[NR]=$0 } END { for (i=NR; i>=1; i--) print a[i] }')
  if [ "${#files[@]}" -lt "$ncount" ]; then
    echo "Error: found fewer than $ncount benchmark files matching pattern in test_data" >&2; exit 1
  fi
else
  if [ "$#" -lt 2 ]; then
    echo "Error: need at least two input files or use -n COUNT" >&2
    usage
    exit 1
  fi
  # Always prepend test_data/ to provided paths unless already present
  for f in "$@"; do
    case "$f" in
      ../test_data/*) files+=("$f");;
      *) files+=("../test_data/$f");;
    esac
  done
fi

# Validate inputs exist
for f in "${files[@]}"; do
  if [ ! -f "$f" ]; then
    echo "Error: file not found: $f" >&2
    exit 1
  fi
done

# Check if Python is available
if ! command -v python3 &> /dev/null; then
  echo "Error: Python 3 is required but not found in PATH" >&2
  exit 1
fi

# Check if parse script exists
if [ ! -f "compare_tests_parse.py" ]; then
  echo "Error: compare_tests_parse.py not found" >&2
  exit 1
fi

# Check if format script exists
if [ ! -f "compare_tests_format.py" ]; then
  echo "Error: compare_tests_format.py not found" >&2
  exit 1
fi

# Temporary files
tmp_data=$(mktemp)
tmp_names=$(mktemp)
cleanup() { rm -f "$tmp_data" "$tmp_names"; }
trap cleanup EXIT

# Parse files using Python script
python3 compare_tests_parse.py "$tmp_data" "${files[@]}"

# Extract and sort unique benchmark names
cut -f3 "$tmp_data" | LC_ALL=C sort -u > "$tmp_names"

N=${#files[@]}  # number of input files

# Generate output
if [ -n "$out" ]; then
  case "$out" in
    ../test_data/*) :;;
    *) out="../test_data/$out";;
  esac
  python3 compare_tests_format.py "$tmp_data" "$tmp_names" "$N" > "$out"
else
  python3 compare_tests_format.py "$tmp_data" "$tmp_names" "$N"
fi
