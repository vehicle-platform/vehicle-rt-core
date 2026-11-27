/*
 * _sbrk that always fails (T77, invariant 5: no dynamic memory on the MCU). It replaces the
 * CubeMX-generated Src/sysmem.c, which grows a newlib heap up to the stack and is not editable
 * (no USER CODE block). cubemx/CMakeLists.txt drops sysmem.c from the target and builds this
 * file instead; see cubemx/README.md. A malloc that reaches the C library gets NULL (ENOMEM)
 * instead of heap memory, and tools/check_image_heap.py fails CI if malloc or _sbrk is linked at
 * all, so this is the second line of defence.
 */
#include <errno.h>
#include <stddef.h>

void *_sbrk(ptrdiff_t incr);

/* cppcheck-suppress misra-c2012-8.7 ; DEV-006: _sbrk is called by the C library, not by this project */
void *_sbrk(ptrdiff_t incr)
{
    (void)incr;
    errno = ENOMEM;
    /* cppcheck-suppress misra-c2012-11.6 ; DEV-006: (void *)-1 is the _sbrk error value of the C library ABI */
    return (void *)-1;
}
