#ifndef MIC_CLAP_H
#define MIC_CLAP_H

#include "mic.h"
#include <stdint.h>
#include <string.h>

/* ADC counts and samples at the existing 96 kHz sampling rate. */
#define MIC_CLAP_NOISE_BLOCKS       16u
#define MIC_CLAP_NOISE_SAMPLES      1536u
#define MIC_CLAP_ENV_SAMPLES        48u
#define MIC_CLAP_GATE_RATIO         4u
#define MIC_CLAP_GATE_MIN           6u
#define MIC_CLAP_PEAK_NOISE_RATIO    6u
#define MIC_CLAP_CLEAR_PEAK_RATIO    12u
#define MIC_CLAP_WEAK_MAX_WIDTH      (MIC_FS_HZ * 26u / 1000u)
#define MIC_CLAP_FAINT_MAX_WIDTH     (MIC_FS_HZ * 20u / 1000u)
#define MIC_CLAP_PEAK_TAIL_SAMPLES   (MIC_FS_HZ * 8u / 1000u)
#define MIC_CLAP_TRIGGER_CONFIRM    24u
#define MIC_CLAP_MAX_RISE_SAMPLES   (MIC_FS_HZ * 6u / 1000u)
#define MIC_CLAP_MAX_WIDTH_SAMPLES  (MIC_FS_HZ * 30u / 1000u)
#define MIC_CLAP_EARLY_SAMPLES      (MIC_FS_HZ * 20u / 1000u)
#define MIC_CLAP_ENERGY_SAMPLES     (MIC_FS_HZ * 40u / 1000u)
#define MIC_CLAP_MIN_HOT_SAMPLES    (MIC_FS_HZ / 4000u)
#define MIC_CLAP_MAX_CLIP_SAMPLES   (MIC_FS_HZ / 1000u)
/* Leave margin before the taskbook's minimum 1.5 s clap interval. */
#define MIC_CLAP_DEDUP_US           1200000u

enum {
  MIC_CLAP_OK = 0,
  MIC_CLAP_NO_EDGE = 1,
  MIC_CLAP_SLOW_RISE = 2,
  MIC_CLAP_LONG_SOUND = 3,
  MIC_CLAP_LONG_TAIL = 4,
  MIC_CLAP_CLIPPED = 5,
  MIC_CLAP_TOO_SHORT = 6,
  MIC_CLAP_CHIRP_LIKE = 7,
  MIC_CLAP_DUPLICATE = 8,
  MIC_CLAP_TRUNCATED = 9,
  MIC_CLAP_LOW_PEAK = 10,
  MIC_CLAP_UNCERTAIN_TIME = 11
};

typedef struct {
  uint32_t pos256;       /* Q8 position on the shared 24 kHz time grid. */
  uint32_t peak;
  uint32_t noise;
  uint32_t rise_us;
  uint32_t width_us;
  uint32_t early_percent;
  uint32_t clipped;
  uint8_t reason;
} mic_clap_result_t;

static uint32_t mic_clap_magnitude(int16_t sample, int32_t dc)
{
  int32_t value = (int32_t)sample - dc;
  return (uint32_t)(value < 0 ? -value : value);
}

/* Preserve the fractional DC baseline for timing weak signals. Rounding the
   baseline to an ADC code before abs() can create an entire false early lobe. */
static uint32_t mic_clap_magnitude_q8(int16_t sample, int32_t dc_q8)
{
  int32_t value = (int32_t)sample * 256 - dc_q8;
  return (uint32_t)(value < 0 ? -value : value);
}

static int32_t mic_clap_baseline_q8(const int16_t *raw, uint32_t *blocks)
{
  uint32_t i, j;
  /* Late triggers can put the beginning of the pulse into the last few
     baseline blocks. A median of block means resists this contamination. */
  for (i = 0u; i < MIC_CLAP_NOISE_BLOCKS; ++i) {
    uint32_t start = i * (MIC_CLAP_NOISE_SAMPLES / MIC_CLAP_NOISE_BLOCKS);
    uint32_t block_sum = 0u;
    for (j = 0u; j < MIC_CLAP_NOISE_SAMPLES / MIC_CLAP_NOISE_BLOCKS; ++j)
      block_sum += (uint32_t)raw[start + j];
    blocks[i] = block_sum * 256u / (MIC_CLAP_NOISE_SAMPLES / MIC_CLAP_NOISE_BLOCKS);
  }
  for (i = 1u; i < MIC_CLAP_NOISE_BLOCKS; ++i) {
    uint32_t value = blocks[i];
    j = i;
    while (j && blocks[j-1u] > value) { blocks[j] = blocks[j-1u]; --j; }
    blocks[j] = value;
  }
  return (int32_t)((blocks[7] + blocks[8]) / 2u);
}

/* Analyze the captured window without allocating another audio buffer. */
static uint8_t mic_clap_analyze(const int16_t *raw, uint32_t n,
                              mic_clap_result_t *out)
{
  uint32_t blocks[MIC_CLAP_NOISE_BLOCKS];
  uint32_t i, j, sum = 0u, peak = 0u, peak_index = 0u;
  uint32_t sum_q8 = 0u, peak_q8 = 0u, peak_q8_index = 0u;
  uint32_t env = 0u, previous = 0u, gate, low, energy_floor;
  uint32_t last_low = MIC_CLAP_NOISE_SAMPLES, onset = 0u, onset_q8 = 0u;
  uint32_t hot = 0u, positive = 0u, negative = 0u, width = 0u;
  uint64_t energy = 0u, early_energy = 0u;
  int32_t dc = 0, dc_q8;
  uint8_t have_edge = 0u;

  memset(out, 0, sizeof(*out));
  out->reason = MIC_CLAP_NO_EDGE;
  if (raw == 0 || n <= MIC_CLAP_NOISE_SAMPLES + MIC_CLAP_ENV_SAMPLES)
    return 0u;

  dc_q8 = mic_clap_baseline_q8(raw, blocks);
  dc = (dc_q8 + 128) / 256;
  for (i = 0u; i < MIC_CLAP_NOISE_BLOCKS; ++i) {
    uint32_t start = i * (MIC_CLAP_NOISE_SAMPLES / MIC_CLAP_NOISE_BLOCKS);
    uint32_t block_sum = 0u;
    for (j = 0u; j < MIC_CLAP_NOISE_SAMPLES / MIC_CLAP_NOISE_BLOCKS; ++j)
      block_sum += mic_clap_magnitude(raw[start + j], dc);
    blocks[i] = block_sum / (MIC_CLAP_NOISE_SAMPLES / MIC_CLAP_NOISE_BLOCKS);
  }
  for (i = 1u; i < MIC_CLAP_NOISE_BLOCKS; ++i) {
    uint32_t value = blocks[i];
    j = i;
    while (j != 0u && blocks[j - 1u] > value) {
      blocks[j] = blocks[j - 1u];
      --j;
    }
    blocks[j] = value;
  }
  out->noise = (blocks[7] + blocks[8]) / 2u;
  if (out->noise < 1u) out->noise = 1u;
  energy_floor = out->noise * 2u;

  for (i = 0u; i < n; ++i) {
    uint32_t magnitude = mic_clap_magnitude(raw[i], dc);
    sum += magnitude;
    if (i >= MIC_CLAP_ENV_SAMPLES)
      sum -= mic_clap_magnitude(raw[i - MIC_CLAP_ENV_SAMPLES], dc);
    env = sum / MIC_CLAP_ENV_SAMPLES;
    sum_q8 += mic_clap_magnitude_q8(raw[i], dc_q8);
    if (i >= MIC_CLAP_ENV_SAMPLES)
      sum_q8 -= mic_clap_magnitude_q8(raw[i - MIC_CLAP_ENV_SAMPLES], dc_q8);
    if (i >= MIC_CLAP_NOISE_SAMPLES) {
      if (env > peak) { peak = env; peak_index = i; }
      if (sum_q8 > peak_q8) { peak_q8 = sum_q8; peak_q8_index = i; }
      if (raw[i] <= 8 || raw[i] >= 4087) ++out->clipped;
      if (magnitude > energy_floor * 2u) {
        ++hot;
        if ((int32_t)raw[i] >= dc) ++positive;
        else ++negative;
      }
    }
  }
  out->peak = peak;
  if (peak < out->noise * MIC_CLAP_PEAK_NOISE_RATIO) {
    out->reason = MIC_CLAP_LOW_PEAK;
    return 0u;
  }
  if (n - peak_index <= MIC_CLAP_PEAK_TAIL_SAMPLES) {
    out->reason = MIC_CLAP_TRUNCATED;
    return 0u;
  }
  if (out->clipped > MIC_CLAP_MAX_CLIP_SAMPLES) {
    out->reason = MIC_CLAP_CLIPPED;
    return 0u;
  }
  if (hot < MIC_CLAP_MIN_HOT_SAMPLES || positive < 8u || negative < 8u) {
    out->reason = MIC_CLAP_TOO_SHORT;
    return 0u;
  }

  /* A relative level suppresses handling noise before the actual impulse.
     The noise gate still prevents a quiet window from defining its own event. */
  gate = peak / 5u;
  if (gate < out->noise * MIC_CLAP_GATE_RATIO)
    gate = out->noise * MIC_CLAP_GATE_RATIO;
  if (gate < MIC_CLAP_GATE_MIN) gate = MIC_CLAP_GATE_MIN;
  low = peak / 20u;
  if (low < energy_floor) low = energy_floor;
  sum = 0u;
  for (i = 0u; i < n; ++i) {
    sum += mic_clap_magnitude(raw[i], dc);
    if (i >= MIC_CLAP_ENV_SAMPLES)
      sum -= mic_clap_magnitude(raw[i - MIC_CLAP_ENV_SAMPLES], dc);
    env = sum / MIC_CLAP_ENV_SAMPLES;
    if (i >= MIC_CLAP_NOISE_SAMPLES && !have_edge) {
      if (env <= low) last_low = i;
      if (previous < gate && env >= gate) {
        uint32_t fraction = ((gate - previous) * 256u) / (env - previous);
        onset = i;
        onset_q8 = (i - 1u) * 256u + fraction;
        out->rise_us = (uint32_t)((uint64_t)(i - last_low) * 1000000u / MIC_FS_HZ);
        have_edge = 1u;
      }
    }
    if (have_edge) {
      uint32_t excess = env > energy_floor ? env - energy_floor : 0u;
      uint64_t square = (uint64_t)excess * excess;
      if (i - onset < MIC_CLAP_ENERGY_SAMPLES) energy += square;
      if (i - onset < MIC_CLAP_EARLY_SAMPLES) early_energy += square;
      if (env >= out->noise + (peak - out->noise) / 4u) ++width;
    }
    previous = env;
  }
  if (!have_edge) return 0u;
  out->pos256 = (onset_q8 + 2u) / 4u;
  out->width_us = (uint32_t)((uint64_t)width * 1000000u / MIC_FS_HZ);
  out->early_percent = energy ? (uint32_t)(early_energy * 100u / energy) : 0u;
  if (out->rise_us > MIC_CLAP_MAX_RISE_SAMPLES * 1000000u / MIC_FS_HZ) {
    out->reason = MIC_CLAP_SLOW_RISE;
    return 0u;
  }
  if (width > MIC_CLAP_MAX_WIDTH_SAMPLES ||
      (peak < out->noise * MIC_CLAP_CLEAR_PEAK_RATIO && width > MIC_CLAP_WEAK_MAX_WIDTH) ||
      (peak < out->noise * 8u && width > MIC_CLAP_FAINT_MAX_WIDTH)) {
    out->reason = MIC_CLAP_LONG_SOUND;
    return 0u;
  }
  if (out->early_percent < 60u) {
    out->reason = MIC_CLAP_LONG_TAIL;
    return 0u;
  }
  /* Classification uses a conservative noise gate. Timing uses a separate
     20% constant-fraction crossing so a quiet board does not timestamp at a
     higher fraction of its pulse than a loud board. Keep the full window sum
     here: rounding the envelope to ADC counts moves weak edges noticeably.
     The earliest crossing before the pulse peak preserves the direct path. */
  {
    uint32_t previous_sum = 0u;
    uint8_t timed = 0u;
    sum = 0u;
    for (i = 0u; i <= peak_q8_index; ++i) {
      sum += mic_clap_magnitude_q8(raw[i], dc_q8);
      if (i >= MIC_CLAP_ENV_SAMPLES)
        sum -= mic_clap_magnitude_q8(raw[i - MIC_CLAP_ENV_SAMPLES], dc_q8);
      /* A late trigger can leave the real edge inside the latter half of the
         robust baseline interval; do not force its timestamp past 16 ms. */
      if (i >= MIC_CLAP_NOISE_SAMPLES / 2u && previous_sum * 5u < peak_q8 && sum * 5u >= peak_q8) {
        uint32_t fraction = (uint32_t)(((uint64_t)(peak_q8 - previous_sum * 5u) * 256u) /
                                      ((sum - previous_sum) * 5u));
        out->pos256 = (((i - 1u) * 256u + fraction) + 2u) / 4u;
        timed = 1u;
        break;
      }
      previous_sum = sum;
    }
    if (!timed) {
      out->reason = MIC_CLAP_UNCERTAIN_TIME;
      return 0u;
    }
  }
  out->reason = MIC_CLAP_OK;
  return 1u;
}

#endif
