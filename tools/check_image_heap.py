#!/usr/bin/env python3
"""Fail if a linked firmware image contains a heap (invariant 5: no dynamic memory on the MCU).

Runs `arm-none-eabi-nm` on the ELF and looks at the defined symbols.

Forbidden (exit 1): the C library heap (malloc, free, calloc, realloc, memalign, their
reentrant `_r` forms, `__malloc_*`) and the `_sbrk` / `_sbrk_r` / `sbrk` break functions. The
image links cubemx/User/sbrk_fail.c (an `_sbrk` that always fails); even that must not be
reachable, so any `_sbrk` symbol in the image means something calls into the C library heap.
The generated `__sbrk_heap_end` (Src/sysmem.c) must be absent too: it proves the generated
newlib heap is not linked.

Residual (reported, exit 0): the FreeRTOS heap_4 allocator (`pvPortMalloc`, `vPortFree`).
CubeMX cannot turn dynamic allocation off with CMSIS_V2 yet (cubemx/README.md, "Open"). Pass
`--forbid-freertos-heap` once T13 drops heap_4.c.
"""
import argparse
import re
import subprocess
import sys

FORBIDDEN = re.compile(
    r"^(malloc|free|calloc|realloc|reallocf|memalign|aligned_alloc|posix_memalign|valloc|pvalloc"
    r"|_malloc_r|_free_r|_calloc_r|_realloc_r|_memalign_r|_mallinfo_r|_malloc_trim_r"
    r"|_sbrk|_sbrk_r|sbrk|__sbrk_heap_end|__malloc_[A-Za-z_]+|_malloc_[A-Za-z_]+)$"
)
FREERTOS_HEAP = re.compile(r"^(pvPortMalloc|vPortFree|xPortGetFreeHeapSize|xPortGetMinimumEverFreeHeapSize)$")


def defined_symbols(nm, elf):
    out = subprocess.run([nm, "--defined-only", elf], check=True, capture_output=True, text=True).stdout
    names = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            names.append(parts[2])
    return names


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf")
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    ap.add_argument("--forbid-freertos-heap", action="store_true",
                    help="also fail on pvPortMalloc/vPortFree (heap_4 and friends)")
    args = ap.parse_args()

    names = defined_symbols(args.nm, args.elf)
    if not names:
        print(f"error: no symbols read from {args.elf}", file=sys.stderr)
        return 2
    bad = sorted({n for n in names if FORBIDDEN.match(n)})
    rtos = sorted({n for n in names if FREERTOS_HEAP.match(n)})
    for n in bad:
        print(f"error: heap symbol linked: {n}", file=sys.stderr)
    for n in rtos:
        level = "error" if args.forbid_freertos_heap else "note (residual, see cubemx/README.md Open)"
        print(f"{level}: FreeRTOS heap symbol linked: {n}", file=sys.stderr)
    if bad or (rtos and args.forbid_freertos_heap):
        return 1
    print(f"ok: no malloc/free/_sbrk in {args.elf} ({len(names)} symbols checked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
