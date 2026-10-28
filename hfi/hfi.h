#ifndef HFI_H
#define HFI_H

#include "speed_pos_fdbk.h"

/*
 * 0: encoder feedback, HFI runs in parallel.
 * 1: HFI angle and speed feedback.
 */

 /* 新增这个开关 */
#define HFI_ENABLE_INJECTION   0

#define HFI_USE_FEEDBACK          0

/* Set to 1 only after hardware timing verification. */
#define HFI_TIMING_VERIFIED       0

/*
 * 0: preceding submitted PWM command.
 * 1: one additional control-period delay.
 */
#define HFI_SAMPLE_DELAY          0

/* Synchronization polarity and detector direction. */
#define HFI_DEMOD_SIGN            1.0f
#define HFI_PLL_POLARITY          1.0f//1图片方向 -1参考代码方向

/* Starting values requiring hardware tuning. */
#define HFI_INJECTION_V           1.60f

/*
 * Direct-current-error PLL.
 * KI is a continuous-time gain: integral += KI * error * Ts.
 * These values are NOT copied from the normalized-error version.
 */
#define HFI_PLL_KP                20.0f
#define HFI_PLL_KI                2000.0f

#define HFI_SPEED_LPF_HZ          4.0f
#define HFI_MIN_RESPONSE_A        0.03f

/*
 * Fundamental-current branch:
 * always use the reference [1,2,1]/4 filter.
 *
 * 0: disable additional dq low-pass initially.
 * Reference alpha=0.01046 at 20 kHz is approximately 33.6 Hz.
 * Enabling that low-pass requires checking current-loop stability.
 */
#define HFI_CURRENT_LPF_HZ        0.0f

#define HFI_TARGET_LIMIT_RPM      300
#define HFI_EST_LIMIT_RPM         600.0f
#define HFI_FIRST_TARGET_RPM      300

#define HFI_ALIGN_CURRENT_A       2.0f
#define HFI_ALIGN_TIME_MS         700U

#define HFI_SETTLE_TIME_MS        500U
#define HFI_START_TIMEOUT_MS      3000U
#define HFI_ID_RELEASE_MS         200U

#define HFI_IQ_LIMIT_A            5.0f
#define HFI_CURRENT_TRIP_A        8.0f
#define HFI_ENCODER_TRIP_RPM      8000

#if HFI_USE_FEEDBACK && !HFI_TIMING_VERIFIED
#error Verify PWM/ADC timing before enabling HFI feedback
#endif

#if (HFI_SAMPLE_DELAY < 0) || (HFI_SAMPLE_DELAY > 1)
#error HFI_SAMPLE_DELAY must be 0 or 1
#endif

typedef struct
{
    SpeednPosFdbk_Handle_t sensor;

    /* MCSDK electrical angle convention, radians. */
    float theta;
    float omega;
    float integral;
    float omega_filtered;

    /* Runtime-adjustable PLL gains. */
    float kp;
    float ki;

    /* Raw alpha/beta current histories, amperes. */
    float i_last[2];
    float i_llast[2];
    float high_last[2];

    /* Synchronously demodulated responses, amperes. */
    float response_alpha;
    float response_beta;
    float response_d;

    /*
     * reference_x:
     * original reference PLL.x after coordinate conversion.
     *
     * error:
     * image detector direction, with polarity adjustment.
     */
    float reference_x;
    float error;

    /* Fundamental dq-current filter histories, MCSDK digits. */
    qd_t dq_last;
    qd_t dq_llast;
    float dq_filtered[2];

    float injection_v;
    int16_t injection_digit;

    int8_t next_sign;
    int8_t command_history[2];
    int8_t sample_sign;

    uint16_t warmup;
    volatile uint32_t good_samples;
    volatile float angle_error_deg;
    volatile bool active;

    //临时
    /* Temporary START diagnostics. */
    volatile uint32_t start_pass_count;
    volatile uint32_t start_response_fail;
    volatile uint32_t start_angle_fail;
    volatile uint32_t start_speed_fail;

    volatile uint16_t start_stable_peak;
    volatile uint8_t  start_fail_mask;
} HFI_Handle_t;

extern HFI_Handle_t HFI_M1;

void HFI_Motor1Init(void);

int32_t HFI_LimitCurrent(int32_t value, float limit_a);

/* Call with HF interrupt access excluded. */
void HFI_Reset(int16_t alignment_angle);

void HFI_Stop(void);

/* Input: raw MCSDK alpha/beta current digits. */
void HFI_Update(alphabeta_t raw);

/* Input/output: MCSDK dq current digits. */
qd_t HFI_FilterCurrent(qd_t raw);

/* Input: actual DC-bus voltage in volts. */
int16_t HFI_PrepareInjection(float bus_voltage);

/* Call after successful PWM submission. */
void HFI_CommitInjection(void);

/* Carrier-response check, not a saliency/angle-lock guarantee. */
bool HFI_ResponsePresent(void);

void EncoderSyncAfterAlignment(void);

#endif