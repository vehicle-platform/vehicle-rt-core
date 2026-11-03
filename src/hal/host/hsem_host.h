#ifndef HAL_HOST_HSEM_HOST_H
#define HAL_HOST_HSEM_HOST_H

/*
 * Host model of the STM32H7 HSEM (hal/hsem.h) for the mailbox tests (D-093 item 6): two
 * cores, one owner word per semaphore (compiler __atomic builtins, visible to
 * ThreadSanitizer: a take is an acquire, a release a release), the one-step same-core
 * re-take of the real HSEM, and injected contention:
 *   - hsem_host_force_busy(): the next n takes of an ID fail, whoever asks;
 *   - hsem_host_seize(): a core holds an ID until hsem_host_release_as();
 *   - a hook that runs after every successful take while the lock is held (to yield or
 *     sleep, widening the windows of the two-thread tests).
 * The calling thread's core is thread-local (hsem_host_set_core(), default M7).
 */

#include "hal/hsem.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HSEM_HOST_FREE = 0,
    HSEM_HOST_CORE_M7 = 1,
    HSEM_HOST_CORE_M4 = 2,
} hsem_host_core_t;

/* Frees every semaphore and clears the injections and the hook (single-threaded). */
void hsem_host_reset(void);

/* The core of the calling thread. */
void hsem_host_set_core(hsem_host_core_t core);

/* The next takes calls on id fail (any core). */
void hsem_host_force_busy(uint8_t id, uint32_t takes);

/* core takes id regardless of its owner (a peer holding the lock); release_as frees it. */
void hsem_host_seize(uint8_t id, hsem_host_core_t core);
void hsem_host_release_as(uint8_t id, hsem_host_core_t core);

/* The owner of id now. */
hsem_host_core_t hsem_host_owner(uint8_t id);

/* Called after each successful take, while held (NULL: none). Set before threads start. */
void hsem_host_set_held_hook(void (*hook)(uint8_t id));

/* Successful takes / refused takes since the last reset (all cores). */
uint32_t hsem_host_taken_count(void);
uint32_t hsem_host_refused_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_HSEM_HOST_H */
