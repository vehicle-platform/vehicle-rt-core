#!/usr/bin/env python3
"""Fail unless each fault handler of a linked image is a stack-free naked entry.

HardFault_Handler, MemManage_Handler, BusFault_Handler and UsageFault_Handler must start with
`ldr rX, [pc, ...]` then `msr MSP, rX` (the stack is reloaded before anything else) and must not
contain a push, a push-like store (stmdb sp, str ... [sp, ...]) or a call (bl/blx) of their own.
The handlers are naked on purpose (cubemx/User/boot_checks.h): a compiled prologue would push onto
a possibly exhausted stack, and a fault in the handler is a lockup.
"""
import argparse
import re
import subprocess
import sys

HANDLERS = ("HardFault_Handler", "MemManage_Handler", "BusFault_Handler", "UsageFault_Handler")


def disassemble(objdump, elf, symbol):
    out = subprocess.run([objdump, "-d", f"--disassemble={symbol}", elf], check=True,
                         capture_output=True, text=True).stdout
    ins = []
    for line in out.splitlines():
        m = re.match(r"^\s*[0-9a-f]+:\s+(?:[0-9a-f]{2,4}\s+)+\s*([a-z][a-z0-9.]*)\s*(.*)$", line)
        if m:
            ins.append((m.group(1), m.group(2).split("@")[0].strip()))
    return ins


def check(objdump, elf):
    errors = []
    for h in HANDLERS:
        ins = disassemble(objdump, elf, h)
        if len(ins) < 2:
            errors.append(f"{elf}: {h}: not found or too short")
            continue
        if not ins[0][0].startswith("ldr"):
            errors.append(f"{elf}: {h}: first instruction is {ins[0][0]}, expected ldr")
        if not (ins[1][0] == "msr" and ins[1][1].upper().startswith("MSP")):
            errors.append(f"{elf}: {h}: second instruction is '{ins[1][0]} {ins[1][1]}', expected msr MSP")
        for mnem, ops in ins:
            if mnem.startswith(("push", "stmdb", "stm", "vpush", "bl")) and mnem not in ("bic", "bics", "bkpt"):
                errors.append(f"{elf}: {h}: stack/call instruction '{mnem} {ops}'")
            if mnem.startswith(("str", "vstr")) and re.search(r"\[sp[,\]]", ops):
                errors.append(f"{elf}: {h}: store to the stack '{mnem} {ops}'")
        errors += check_tail_and_stores(elf, h, ins)
    return errors


def check_tail_and_stores(elf, h, ins):
    """The only exit is a branch to boot_fault_safe_state (after it only padding or the
    generated self-loop), and every store goes through one base register (the record)."""
    errors = []
    idx = [i for i, (m, o) in enumerate(ins) if m.startswith("b") and "boot_fault_safe_state" in o]
    if len(idx) != 1:
        errors.append(f"{elf}: {h}: expected one branch to boot_fault_safe_state, found {len(idx)}")
    else:
        for m, o in ins[idx[0] + 1:]:
            if not (m.startswith("nop") or (m.startswith("b") and "<" + h in o)):
                errors.append(f"{elf}: {h}: instruction '{m} {o}' after the branch to boot_fault_safe_state")
    bases = set()
    stores = 0
    for m, o in ins:
        if m.startswith("str"):
            mm = re.search(r"\[(r\d+)\s*(?:,\s*#(\d+))?\]", o)
            if not mm:
                errors.append(f"{elf}: {h}: unexpected store '{m} {o}'")
                continue
            stores += 1
            bases.add(mm.group(1))
    if len(bases) != 1 or stores != 4:
        errors.append(f"{elf}: {h}: expected 4 stores through one base register, got {stores} through {sorted(bases)}")
    return errors


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("elf", nargs="+")
    ap.add_argument("--objdump", default="arm-none-eabi-objdump")
    args = ap.parse_args()
    errors = [e for elf in args.elf for e in check(args.objdump, elf)]
    for e in errors:
        print("error: " + e, file=sys.stderr)
    if not errors:
        print("ok: the four fault handlers are stack-free in " + ", ".join(args.elf))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
