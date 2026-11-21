#ifndef FEATURES_DERIVED_DERIVED_WINDOWS_INTERNAL_H
#define FEATURES_DERIVED_DERIVED_WINDOWS_INTERNAL_H

/*
 * Private interface between derived_windows_setup.c (configuration, sizing, channel wiring, condition
 * checks), derived_windows.c (event engine) and derived_windows_value.c (summaries). Helpers moved out of
 * one file, same code (they lost their `static`). Only the files in features/derived may include it.
 */

#include "features/derived/derived_windows.h"
#include "vehicle_cl250_constants.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MS_PER_S (1000.0)

/* close reasons (the reference's CLOSE_*) */
#define CLOSE_CONDITION (1u)
#define CLOSE_STABLE (2u)
#define CLOSE_MAX_DURATION (3u)
#define CLOSE_ENGINE_ZERO (4u)
#define CLOSE_UNKNOWN (5u)
#define CLOSE_SESSION_END (6u)

bool at_or_after(uint32_t a, uint32_t b);
bool is_transition(uint8_t opens_on);
bool win_is_zero(double x);
bool wants_stat_row(uint32_t wi, uint8_t kind, uint32_t index);
bool ch_latest(const derived_win_chan_t* c, uint32_t t, double* v);
bool conds_hold(const derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t);
bool can_open_at(const derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t);
uint8_t stable_close(derived_win_t* w, const vehicle_cl250_window_t* spec, uint32_t t);
void add_mean(derived_win_stats_t* st, uint32_t slot, double v);
float cool_decode(uint32_t raw);
bool cool_encode(float v, uint16_t* raw);
void compute_rpm(const derived_windows_t* a, derived_win_stats_t* st, uint32_t open_ms, uint32_t close_ms,
                  uint8_t reason);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_DERIVED_DERIVED_WINDOWS_INTERNAL_H */
