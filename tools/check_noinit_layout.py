#!/usr/bin/env python3
"""Check the placement of the orphan .noinit section (the fault record) in a linked image.

.noinit must be NOBITS (no flash image, not copied), start at or after _ebss (outside the
startup .bss clear), end at or before the start of ._user_heap_stack and below _sstack (never
inside the stack region). The linker script is not edited: the section is an orphan that the
linker places after .bss (cubemx/README.md, "Files that are not generated").
"""
import argparse
import re
import subprocess
import sys


def sections(readelf, elf):
    out = subprocess.run([readelf, "-S", "-W", elf], check=True, capture_output=True, text=True).stdout
    found = {}
    for line in out.splitlines():
        m = re.match(r"^\s*\[\s*\d+\]\s+(\S+)\s+(\S+)\s+([0-9a-f]+)\s+[0-9a-f]+\s+([0-9a-f]+)", line)
        if m:
            found[m.group(1)] = (m.group(2), int(m.group(3), 16), int(m.group(4), 16))
    return found


def symbols(nm, elf):
    out = subprocess.run([nm, elf], check=True, capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3:
            syms[p[2]] = int(p[0], 16)
    return syms


def check(readelf, nm, elf):
    sec, sym = sections(readelf, elf), symbols(nm, elf)
    errors = []
    for name in (".noinit", "._user_heap_stack"):
        if name not in sec:
            errors.append(f"{elf}: section {name} missing")
    for name in ("_ebss", "_sstack"):
        if name not in sym:
            errors.append(f"{elf}: symbol {name} missing")
    if errors:
        return errors
    typ, addr, size = sec[".noinit"]
    end = addr + size
    if typ != "NOBITS":
        errors.append(f"{elf}: .noinit is {typ}, expected NOBITS")
    if addr < sym["_ebss"]:
        errors.append(f"{elf}: .noinit 0x{addr:08x} below _ebss 0x{sym['_ebss']:08x}")
    if end > sec["._user_heap_stack"][1]:
        errors.append(f"{elf}: .noinit end 0x{end:08x} beyond ._user_heap_stack 0x{sec['._user_heap_stack'][1]:08x}")
    if end >= sym["_sstack"]:
        errors.append(f"{elf}: .noinit end 0x{end:08x} not below _sstack 0x{sym['_sstack']:08x}")
    return errors


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("elf", nargs="+")
    ap.add_argument("--readelf", default="arm-none-eabi-readelf")
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    args = ap.parse_args()
    errors = [e for elf in args.elf for e in check(args.readelf, args.nm, elf)]
    for e in errors:
        print("error: " + e, file=sys.stderr)
    if not errors:
        print("ok: .noinit is NOBITS, after _ebss, before the heap/stack region in " + ", ".join(args.elf))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
