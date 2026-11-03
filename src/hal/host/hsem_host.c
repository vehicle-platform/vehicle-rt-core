#include "hal/host/hsem_host.h"

#include <stddef.h>

/* Host model of the H7 HSEM, see hsem_host.h. Index = HSEM ID (0..31). */
#define HSEM_HOST_COUNT (HAL_HSEM_ID_MAX + 1u)

static uint32_t owner[HSEM_HOST_COUNT];
static uint32_t busy[HSEM_HOST_COUNT];
static uint32_t taken;
static uint32_t refused;
static void (*held_hook)(uint8_t id);
static _Thread_local uint32_t my_core = (uint32_t)HSEM_HOST_CORE_M7;

static bool id_ok(uint8_t id)
{
    return (id >= HAL_HSEM_ID_MIN) && (id <= HAL_HSEM_ID_MAX);
}

void hsem_host_reset(void)
{
    for (uint32_t i = 0u; i < HSEM_HOST_COUNT; i++) {
        __atomic_store_n(&owner[i], (uint32_t)HSEM_HOST_FREE, __ATOMIC_RELEASE);
        __atomic_store_n(&busy[i], 0u, __ATOMIC_RELAXED);
    }
    __atomic_store_n(&taken, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&refused, 0u, __ATOMIC_RELAXED);
    held_hook = NULL;
    my_core = (uint32_t)HSEM_HOST_CORE_M7;
}

void hsem_host_set_core(hsem_host_core_t core)
{
    my_core = (uint32_t)core;
}

void hsem_host_force_busy(uint8_t id, uint32_t takes)
{
    if (id < HSEM_HOST_COUNT) {
        __atomic_store_n(&busy[id], takes, __ATOMIC_RELAXED);
    }
}

void hsem_host_seize(uint8_t id, hsem_host_core_t core)
{
    if (id < HSEM_HOST_COUNT) {
        __atomic_store_n(&owner[id], (uint32_t)core, __ATOMIC_SEQ_CST);
    }
}

void hsem_host_release_as(uint8_t id, hsem_host_core_t core)
{
    if (id < HSEM_HOST_COUNT) {
        uint32_t expected = (uint32_t)core;
        (void)__atomic_compare_exchange_n(&owner[id], &expected, (uint32_t)HSEM_HOST_FREE, false,
                                          __ATOMIC_RELEASE, __ATOMIC_RELAXED);
    }
}

hsem_host_core_t hsem_host_owner(uint8_t id)
{
    if (id >= HSEM_HOST_COUNT) {
        return HSEM_HOST_FREE;
    }
    return (hsem_host_core_t)__atomic_load_n(&owner[id], __ATOMIC_ACQUIRE);
}

void hsem_host_set_held_hook(void (*hook)(uint8_t id))
{
    held_hook = hook;
}

uint32_t hsem_host_taken_count(void)
{
    return __atomic_load_n(&taken, __ATOMIC_RELAXED);
}

uint32_t hsem_host_refused_count(void)
{
    return __atomic_load_n(&refused, __ATOMIC_RELAXED);
}

/* A forced refusal consumes one injected busy take (bounded: one CAS attempt per call
 * would be enough for the tests; the loop ends when the count is 0 or decremented). */
static bool consume_busy(uint8_t id)
{
    uint32_t n = __atomic_load_n(&busy[id], __ATOMIC_RELAXED);
    while (n > 0u) {
        if (__atomic_compare_exchange_n(&busy[id], &n, n - 1u, false, __ATOMIC_RELAXED,
                                        __ATOMIC_RELAXED)) {
            return true;
        }
    }
    return false;
}

bool hal_hsem_take(uint8_t id, hal_hsem_key_t* key)
{
    if (!id_ok(id) || (key == NULL)) {
        return false;
    }
    key->irq_state = 0u;
    if (consume_busy(id)) {
        __atomic_fetch_add(&refused, 1u, __ATOMIC_RELAXED);
        return false;
    }
    uint32_t expected = (uint32_t)HSEM_HOST_FREE;
    bool ok = __atomic_compare_exchange_n(&owner[id], &expected, my_core, false, __ATOMIC_ACQUIRE,
                                          __ATOMIC_ACQUIRE);
    if (!ok && (expected == my_core)) {
        ok = true; /* the H7 one-step take succeeds again for the holding core */
    }
    if (!ok) {
        __atomic_fetch_add(&refused, 1u, __ATOMIC_RELAXED);
        return false;
    }
    __atomic_fetch_add(&taken, 1u, __ATOMIC_RELAXED);
    void (*hook)(uint8_t) = held_hook;
    if (hook != NULL) {
        hook(id);
    }
    return true;
}

void hal_hsem_release(uint8_t id, const hal_hsem_key_t* key)
{
    (void)key;
    if (id_ok(id)) {
        hsem_host_release_as(id, (hsem_host_core_t)my_core);
    }
}

void hal_hsem_barrier(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

void hal_hsem_release_own(uint8_t id)
{
    if (id_ok(id)) {
        hsem_host_release_as(id, (hsem_host_core_t)my_core);
    }
}
