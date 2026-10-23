#ifndef HFI_H
#define HFI_H

#include "speed_pos_fdbk.h"

/*
 * 0: encoder feedback, HFI runs in parallel
 * 1: HFI feedback
 */
#define HFI_USE_FEEDBACK           0

/*
 * Set to 1 only after verifying PWM/ADC timing on hardware.
 */
#define HFI_TIMING_VERIFIED        0

/*
 * 0: current sample corresponds to the preceding PWM command
 * 1: one additional control-period delay
 */
#define HFI_SAMPLE_DELAY           0

#define HFI_DEMOD_SIGN             1.0f
#define HFI_PLL_SIGN               1.0f

/* Initial tuning values, not validated motor parameters. */
#define HFI_INJECTION_V            0.20f
#define HFI_PLL_KP                 120.0f
#define HFI_PLL_KI                 3600.0f
#define HFI_DEMOD_BW_HZ            200.0f
#define HFI_MIN_RESPONSE_A         0.03f

#define HFI_MAX_TARGET_RPM         300
#define HFI_MAX_EST_RPM            600.0f

#define HFI_ALIGN_CURRENT_A        2.0f
#define HFI_ALIGN_TIME_MS          700U
#define HFI_SETTLE_TIME_MS         500U
#define HFI_START_TIMEOUT_MS       2000U

#define HFI_IQ_LIMIT_A             2.0f
#define HFI_CURRENT_TRIP_A         8.0f
#define HFI_ENCODER_TRIP_RPM       450

#if HFI_USE_FEEDBACK && !HFI_TIMING_VERIFIED
#error Verify PWM/ADC timing before enabling HFI feedback
#endif

#if (HFI_SAMPLE_DELAY < 0) || (HFI_SAMPLE_DELAY > 1)
#error HFI_SAMPLE_DELAY must be 0 or 1
#endif

typedef struct
{
    SpeednPosFdbk_Handle_t sensor;

    float theta;
    float omega;
    float integral;
    float speed_filtered;

    float previous[2];
    float older[2];
    float high_previous[2];

    float response_alpha;
    float response_beta;
    float response_d;
    float response_q;
    float error;

    volatile float angle_error_deg;
    float injection_v;

    uint32_t samples;
    volatile uint32_t good_samples;

    int16_t injection_digit;
    int8_t next_sign;
    int8_t command_history[2];
    int8_t sample_sign;

    volatile bool active;
} HFI_Handle_t;

extern HFI_Handle_t HFI_M1;

void HFI_Reset(int16_t angle);
void HFI_Stop(void);

alphabeta_t HFI_Sample(alphabeta_t raw);

int16_t HFI_PrepareInjection(float bus_voltage);
void HFI_CommitInjection(void);

bool HFI_ResponsePresent(void);

#endif