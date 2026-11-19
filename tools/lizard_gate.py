#!/usr/bin/env python3
"""Complexity and size ratchet for src/ (QUALITY FW-2).

Limits (ISO 26262-6 Table 3 heuristics, not normative numbers): cyclomatic complexity
(CCN) <= 15 and length <= 100 lines per function, <= 600 lines per file. Code that
already exceeded a limit when the gate was introduced is listed in
tools/lizard_baseline.txt. The baseline only shrinks:

  * a function or file over a limit that is not in the baseline fails (new code complies);
  * a baselined entry that got worse (higher CCN, longer) fails;
  * a baselined entry that now complies fails until it is removed from the baseline
    (so the list cannot hide a fixed offender and then silently regress).

Needs `lizard` on PATH (pip install lizard==1.26.1). Run from the repo root.
  tools/lizard_gate.py            check
  tools/lizard_gate.py --update   rewrite the baseline from the current tree (review the diff)
"""
import re
import subprocess
import sys
from pathlib import Path

SRC = "src"
MAX_CCN = 15
MAX_FUNC_LEN = 100
MAX_FILE_LINES = 600
BASELINE = Path("tools/lizard_baseline.txt")
WARN = re.compile(
    r"^(?P<file>\S+?):\d+: warning: (?P<fn>\S+) has \d+ NLOC, (?P<ccn>\d+) CCN, "
    r".*?, (?P<len>\d+) length"
)


def measure():
    out = subprocess.run(
        ["lizard", "-l", "c", "-C", str(MAX_CCN), "-L", str(MAX_FUNC_LEN), "-w", SRC],
        capture_output=True, text=True, check=False,
    )
    if "warning" not in out.stdout and out.returncode not in (0, 1):
        sys.exit("lizard failed:\n" + out.stderr)
    funcs = {}
    for line in out.stdout.splitlines():
        m = WARN.match(line)
        if m:
            funcs["F %s::%s" % (m["file"], m["fn"])] = (int(m["ccn"]), int(m["len"]))
    files = {}
    for p in sorted(Path(SRC).rglob("*")):
        if p.suffix in (".c", ".h"):
            n = len(p.read_text(encoding="utf-8").splitlines())
            if n > MAX_FILE_LINES:
                files["L %s" % p.as_posix()] = (n,)
    return {**funcs, **files}


def load_baseline():
    base = {}
    for raw in BASELINE.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        base["%s %s" % (parts[0], parts[1])] = tuple(int(x) for x in parts[2:])
    return base


def fmt(key, val):
    return "%s %s" % (key, " ".join(str(v) for v in val))


def main():
    cur = measure()
    if "--update" in sys.argv:
        head = (
            "# Complexity/size baseline (tools/lizard_gate.py). Existing offenders only; it may\n"
            "# shrink, never grow. F <file>::<function> <ccn> <length>   L <file> <lines>\n"
            "# Limits: CCN %d, function length %d, file lines %d.\n" % (MAX_CCN, MAX_FUNC_LEN, MAX_FILE_LINES)
        )
        BASELINE.write_text(head + "\n".join(fmt(k, v) for k, v in sorted(cur.items())) + "\n", encoding="utf-8")
        print("baseline: %d entries" % len(cur))
        return 0
    base = load_baseline()
    bad = []
    for key, val in sorted(cur.items()):
        if key not in base:
            bad.append("NEW offender (limits CCN %d / %d lines per function, %d per file): %s"
                       % (MAX_CCN, MAX_FUNC_LEN, MAX_FILE_LINES, fmt(key, val)))
        elif any(v > b for v, b in zip(val, base[key])):
            bad.append("WORSE than baseline: %s (baseline %s)" % (fmt(key, val), " ".join(map(str, base[key]))))
    for key in sorted(base):
        if key not in cur:
            bad.append("FIXED, remove from tools/lizard_baseline.txt: %s" % key)
    for b in bad:
        print("::error::" + b)
    print("lizard gate: %d offenders, baseline %d, %d problems" % (len(cur), len(base), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
