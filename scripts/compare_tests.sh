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

# Temporary files
tmp_data=$(mktemp)
tmp_names=$(mktemp)
cleanup() { rm -f "$tmp_data" "$tmp_names"; }
trap cleanup EXIT

# 1) Parse each file and emit normalized TSV: fileIndex \t name \t status \t thr \t avg \t min \t max
#    We skip header and separator lines, and trim whitespace.
awk '
  BEGIN { OFS="\t"; fileIdx=0 }
  function ltrim(s){ sub(/^[ \t]+/, "", s); return s }
  function rtrim(s){ sub(/[ \t]+$/, "", s); return s }
  function trim(s){ return rtrim(ltrim(s)) }
  # Detect separator lines like "--- | --- | ---" or lines of only pipes/dashes/spaces
  function is_sep_line(line) { return (line ~ /^[ \t\-|]+$/) }
  FNR==1 { fileIdx++; fb=FILENAME; gsub(/.*\//, "", fb) }
  {
    if (is_sep_line($0)) next;
    # Split by raw pipe; the first field(s) up to the last 5 pipes are the name
  n = split($0, a, /\|/);
  # Drop trailing empty fields due to a trailing pipe at line end
  while (n > 0 && trim(a[n]) == "") { n--; }
  if (n < 6) next;
    maxv = trim(a[n]);
    minv = trim(a[n-1]);
    avg  = trim(a[n-2]);
    thr  = trim(a[n-3]);
    status = trim(a[n-4]);
    # Join remaining parts back as the name (may contain internal pipes)
    name = "";
    for (i=1; i<=n-5; i++) {
      part = trim(a[i]);
      if (part == "") continue;
      if (name == "") name = part; else name = name " | " part;
    }
    name = trim(name);
    if (name ~ /^[Bb]enchmark[ ]+[Nn]ame$/) next; # skip header
    if (name == "") next;
    print fileIdx, fb, name, status, thr, avg, minv, maxv;
  }
' "${files[@]}" > "$tmp_data"

# 2) Collect and sort unique benchmark names
# names are in column 3 of tmp_data
cut -f3 "$tmp_data" | LC_ALL=C sort -u > "$tmp_names"

N=${#files[@]}  # number of input files

# 3) Prepare output stream
if [ -n "$out" ]; then
  case "$out" in
    ../test_data/*) :;;
    *) out="../test_data/$out";;
  esac
  exec >"$out"
fi

# 4+5) Compute widths, then print header, separator, and rows with padding
awk -v N="$N" -F"\t" '
  BEGIN { OFS = " | " }
  function esc(s) { gsub(/\|/, "|", s); return s }
  function pad(s, w) { return sprintf("%-*s", w, s) }
  function rep(ch, n,   r,i) { r=""; for (i=1; i<=n; i++) r=r ch; return r }
  FNR==NR {
    # dataset columns: 1=idx,2=fileBase,3=name,4=status,5=thr,6=avg,7=min,8=max
    i=$1; fb=$2; name=$3;
    data[i, name, 1]=$4; data[i, name, 2]=$5; data[i, name, 3]=$6; data[i, name, 4]=$7; data[i, name, 5]=$8;
    # remember file index existence
    files[i]=1;
    next
  }
  {
    names[++cnt]=$0
  }
  END {
    # headers and widths for 7 columns
    header[1] = "File"; widths[1] = (length(header[1]) < 4 ? 4 : length(header[1]));
    header[2] = "Benchmark Name"; widths[2] = length(header[2]);
    header[3] = "Status"; widths[3] = length(header[3]);
    header[4] = "Threshold (ms)"; widths[4] = length(header[4]);
    header[5] = "Avg (ms)"; widths[5] = length(header[5]);
    header[6] = "Min (ms)"; widths[6] = length(header[6]);
    header[7] = "Max (ms)"; widths[7] = length(header[7]);

    # compute widths
    for (n=1; n<=cnt; n++) {
      name = names[n];
      escname = esc(name);
      if (length(escname) > widths[2]) widths[2] = length(escname);
      for (i=1; i<=N; i++) {
        for (k=1; k<=5; k++) {
          key = i SUBSEP name SUBSEP k;
          val = (key in data) ? data[i, name, k] : "";
          col = 2 + k; # map k=1..5 -> cols 3..7
          if (length(val) > widths[col]) widths[col] = length(val);
        }
        if (length(i) > widths[1]) widths[1] = length(i);
      }
    }

    # print header
    line = pad(header[1], widths[1]);
    line = line OFS pad(header[2], widths[2]);
    for (c=3; c<=7; c++) line = line OFS pad(header[c], widths[c]);
    print line;

    # separator
    sep = pad(rep("-", (widths[1] < 3 ? 3 : widths[1])), widths[1]);
    sep = sep OFS pad(rep("-", (widths[2] < 3 ? 3 : widths[2])), widths[2]);
    for (c=3; c<=7; c++) sep = sep OFS pad(rep("-", (widths[c] < 3 ? 3 : widths[c])), widths[c]);
    print sep;

    # print rows: for each name, one row per file i=1..N
    for (n=1; n<=cnt; n++) {
      print "";
      name = names[n]; escname = esc(name);
      for (i=1; i<=N; i++) {
        row = pad(i, widths[1]);
        row = row OFS pad(escname, widths[2]);
        for (k=1; k<=5; k++) {
          key = i SUBSEP name SUBSEP k;
          val = (key in data) ? data[i, name, k] : "";
          col = 2 + k;
          row = row OFS pad(val, widths[col]);
        }
        print row;
      }
    }
  }
' "$tmp_data" "$tmp_names"
