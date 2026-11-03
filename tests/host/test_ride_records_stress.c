/*
 * Two-thread stress test of the ride-record hand-over (D-098 item 6): the derived-task side
 * (ride_records_publish) in one thread and the comms-task side (ride_records_take, then the
 * server's ride_records_read of every record) in another, sharing only the
 * services/ride_records snapshot, as on the H7. Run under ASan + UBSan (host-tests) and
 * ThreadSanitizer (host-tsan preset).
 *
 * A record set is never torn: publish p carries counter p in every header and the stop count
 * p % 1000 (its raw value), every other field NOT_AVAILABLE. Also checked: counters never go
 * backwards and the last publish arrives. Unity asserts only in the main thread.
 */
#define _POSIX_C_SOURCE 200809L
#include "services/ride_records.h"
#include "vehicle_cl250_derived.h"

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <string.h>
#include <unity.h>

#define PUBLISHES (60000u) /* below the uint16 wrap, so "backwards" is meaningful */

static atomic_bool producer_done;
static uint32_t producer_p; /* producer thread only */

typedef struct {
    uint32_t sets;
    uint32_t torn;
    uint32_t backwards;
    uint32_t last;
} consumer_result_t;

static consumer_result_t result;

void setUp(void) {}
void tearDown(void) {}

static bool value_of_p(uint32_t signal, float* value)
{
    *value = (float)(producer_p % 1000u);
    return signal == VEHICLE_CL250_DERIVED_RIDE_STOP_COUNT;
}

static void* producer(void* arg)
{
    (void)arg;
    for (uint32_t p = 1u; p <= PUBLISHES; p++) {
        producer_p = p;
        (void)ride_records_publish(value_of_p, true, (uint16_t)p);
        if ((p % 64u) == 0u) {
            (void)sched_yield();
        }
    }
    atomic_store(&producer_done, true);
    return NULL;
}

static uint32_t be(const uint8_t* p, uint32_t n)
{
    uint32_t v = 0u;
    for (uint32_t i = 0u; i < n; i++) {
        v = (v << 8u) | p[i];
    }
    return v;
}

/* One comms pass: take, then read every record the way the server does. */
static void check_set(consumer_result_t* r)
{
    ride_records_take();
    uint8_t rec[PLATFORM_UDS_MAX_DID_LENGTH];
    uint32_t counter = 0u;
    bool first = true;
    for (uint32_t idx = 0u; idx < PLATFORM_UDS_DID_COUNT; idx++) {
        if (!ride_records_is_record(idx)) {
            continue;
        }
        const uint16_t len = platform_uds_dids[idx].length;
        if (!ride_records_read(idx, rec, len)) {
            r->torn++;
            continue;
        }
        /* every served record has the same 4-byte header layout (test_ride_records_core) */
        const uint32_t c = be(&rec[PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_BYTE], PLATFORM_UDS_RIDE_USAGE_RIDE_COUNTER_LENGTH);
        if (first) {
            counter = c;
            first = false;
        } else if (c != counter) {
            r->torn++;
        }
    }
    uint8_t usage[PLATFORM_UDS_DID_RIDE_USAGE_LENGTH];
    (void)ride_records_read(PLATFORM_UDS_IDX_RIDE_USAGE, usage, sizeof usage);
    const uint32_t stops = be(&usage[PLATFORM_UDS_RIDE_USAGE_RIDE_STOP_COUNT_BYTE], PLATFORM_UDS_RIDE_USAGE_RIDE_STOP_COUNT_LENGTH);
    const bool closed = (usage[PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_BYTE] & PLATFORM_UDS_RIDE_USAGE_RIDE_CLOSED_MASK) != 0u;
    if (counter == 0u) {
        if (closed || (stops != PLATFORM_UDS_RIDE_USAGE_RIDE_STOP_COUNT_NA)) {
            r->torn++; /* the boot records */
        }
    } else if (!closed || (stops != (counter % 1000u))) {
        r->torn++;
    }
    if (counter < r->last) {
        r->backwards++;
    }
    r->last = counter;
    r->sets++;
}

static void* consumer(void* arg)
{
    (void)arg;
    bool finished = false;
    while (!finished) {
        finished = atomic_load(&producer_done);
        check_set(&result); /* after done: one more pass picks up the last publish */
        (void)sched_yield();
    }
    return NULL;
}

static void test_ride_records_are_never_torn(void)
{
    pthread_t tp;
    pthread_t tc;
    ride_records_init();
    memset(&result, 0, sizeof result);
    atomic_store(&producer_done, false);
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&tc, NULL, consumer, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&tp, NULL, producer, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(tp, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(tc, NULL));
    printf("ride_records stress: %u record sets read, last counter %u\n", (unsigned)result.sets,
           (unsigned)result.last);
    TEST_ASSERT_EQUAL_UINT32(0u, result.torn);
    TEST_ASSERT_EQUAL_UINT32(0u, result.backwards);
    TEST_ASSERT_EQUAL_UINT32(PUBLISHES, result.last);
    TEST_ASSERT_FALSE(ride_records_producer_fault());
    TEST_ASSERT_FALSE(ride_records_consumer_fault());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ride_records_are_never_torn);
    return UNITY_END();
}
