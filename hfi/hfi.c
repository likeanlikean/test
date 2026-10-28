#include "hfi.h"
#include "parameters_conversion.h"
#include "mc_math.h"

#include <math.h>
#include <string.h>

#include "motorcontrol.h"
#include "mc_interface.h"

#define HFI_PI       3.14159265358979323846f
#define HFI_TWO_PI   (2.0f * HFI_PI)
#define HFI_TS       (1.0f / (float)TF_REGULATION_RATE)

HFI_Handle_t HFI_M1;

void HFI_Motor1Init(void)
{
    int16_t iq_limit;
    int16_t initial_speed;
    int32_t integral_limit;

    iq_limit =
        (int16_t)(
            HFI_IQ_LIMIT_A * CURRENT_CONV_FACTOR);

    initial_speed =
        (int16_t)(
            HFI_FIRST_TARGET_RPM
            * SPEED_UNIT / U_RPM);

    integral_limit =
        (int32_t)iq_limit
        * PID_GetKIDivisor(&PIDSpeedHandle_M1);

    /*
     * 1. Speed-command range.
     */
    pSTC[M1]->MaxAppPositiveMecSpeedUnit =
        (uint16_t)(
            HFI_TARGET_LIMIT_RPM
            * SPEED_UNIT / U_RPM);

    pSTC[M1]->MinAppNegativeMecSpeedUnit =
        -(int16_t)(
            HFI_TARGET_LIMIT_RPM
            * SPEED_UNIT / U_RPM);

    pSTC[M1]->MecSpeedRefUnitDefault =
        initial_speed;

    /*
     * 2. Alignment settings.
     */
    EncAlignCtrlM1.hFinalTorque =
        (int16_t)(
            HFI_ALIGN_CURRENT_A
            * CURRENT_CONV_FACTOR);

    EncAlignCtrlM1.hDurationms =
        HFI_ALIGN_TIME_MS;

    /*
     * 3. Speed PI output-current limits.
     */
    PID_SetUpperOutputLimit(
        &PIDSpeedHandle_M1,
        iq_limit);

    PID_SetLowerOutputLimit(
        &PIDSpeedHandle_M1,
        -iq_limit);

    PID_SetUpperIntegralTermLimit(
        &PIDSpeedHandle_M1,
        integral_limit);

    PID_SetLowerIntegralTermLimit(
        &PIDSpeedHandle_M1,
        -integral_limit);

    /*
     * 4. Replace the original buffered default speed command.
     * This queues a command; it does not start the motor.
     */
    MCI_ExecSpeedRamp(
        &Mci[M1],
        initial_speed,
        1500U);
}

int32_t HFI_LimitCurrent(int32_t value, float limit_a)
{
    int32_t limit =
        (int32_t)(limit_a * CURRENT_CONV_FACTOR);

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static float HFI_Limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static float HFI_LpfCoefficient(float frequency_hz)
{
    float value = HFI_TWO_PI * frequency_hz * HFI_TS;

    return value / (1.0f + value);
}

static int16_t HFI_ToAngleDigit(float theta)
{
    return (int16_t)(theta * (32768.0f / HFI_PI));
}

void HFI_Reset(int16_t alignment_angle)
{
    /*
     * Caller must exclude the HF interrupt during reset.
     */
    memset(&HFI_M1, 0, sizeof(HFI_M1));

    HFI_M1.theta =
        (float)alignment_angle * (HFI_PI / 32768.0f);

    HFI_M1.sensor.hElAngle = alignment_angle;

    HFI_M1.sensor.bElToMecRatio = POLE_PAIR_NUM;
    HFI_M1.sensor.SpeedUnit = SPEED_UNIT;

    HFI_M1.sensor.hMeasurementFrequency =
        TF_REGULATION_RATE_SCALED;

    HFI_M1.sensor.DPPConvFactor = DPP_CONV_FACTOR;

    HFI_M1.sensor.hMaxReliableMecSpeedUnit =
        (uint16_t)(
            HFI_EST_LIMIT_RPM * SPEED_UNIT / U_RPM);

    HFI_M1.sensor.hMinReliableMecSpeedUnit = 0U;

    HFI_M1.sensor.hMaxReliableMecAccelUnitP = 65535U;

    HFI_M1.sensor.bMaximumSpeedErrorsNumber =
        M1_SS_MEAS_ERRORS_BEFORE_FAULTS;

    HFI_M1.kp = HFI_PLL_KP;
    HFI_M1.ki = HFI_PLL_KI;

    HFI_M1.injection_v = HFI_INJECTION_V;
    HFI_M1.next_sign = 1;

    HFI_M1.active = true;
}

void HFI_Stop(void)
{
    HFI_M1.active = false;

    HFI_M1.injection_digit = 0;

    HFI_M1.command_history[0] = 0;
    HFI_M1.command_history[1] = 0;

    HFI_M1.good_samples = 0U;
}

void HFI_Update(alphabeta_t raw)
{
    float input[2];
    float high[2];
    float response[2];

    float sn;
    float cs;
    float omega_limit;
    float speed_alpha;

    Trig_Components trig;

    unsigned int axis;

    if (!HFI_M1.active)
    {
        return;
    }

    /*
     * 1. Convert raw currents to amperes.
     */
    input[0] =
        (float)raw.alpha * CURRENT_CONV_FACTOR_INV;

    input[1] =
        (float)raw.beta * CURRENT_CONV_FACTOR_INV;

    HFI_M1.sample_sign =
        HFI_M1.command_history[HFI_SAMPLE_DELAY];

    /*
     * 2. Reference HF extraction and synchronous demodulation.
     */
    for (axis = 0U; axis < 2U; ++axis)
    {
        high[axis] =
            (input[axis]
             - 2.0f * HFI_M1.i_last[axis]
             + HFI_M1.i_llast[axis]) * 0.25f;

        response[axis] =
            (high[axis] - HFI_M1.high_last[axis])
            * (float)HFI_M1.sample_sign
            * HFI_DEMOD_SIGN;

        HFI_M1.i_llast[axis] =
            HFI_M1.i_last[axis];

        HFI_M1.i_last[axis] =
            input[axis];

        HFI_M1.high_last[axis] =
            high[axis];
    }

    /*
     * Ignore initialization transients.
     */
    if (HFI_M1.warmup < 8U)
    {
        ++HFI_M1.warmup;
        return;
    }

    /*
     * No alpha/beta low-pass here.
     */
    HFI_M1.response_alpha = response[0];
    HFI_M1.response_beta = response[1];

    trig =
        MCM_Trig_Functions(
            HFI_M1.sensor.hElAngle);

    sn = (float)trig.hSin / 32768.0f;
    cs = (float)trig.hCos / 32768.0f;

    /*
     * Carrier-response monitor only.
     * It is NOT used to normalize the PLL error.
     */
    HFI_M1.response_d =
        response[0] * sn + response[1] * cs;

    /*
     * Coordinate mapping:
     *
     * alpha_ref = alpha_MC
     * beta_ref  = -beta_MC
     * theta_ref = theta_MC - pi/2
     *
     * Reference source:
     * x = alpha_ref*sin(theta_ref)
     *     - beta_ref*cos(theta_ref)
     *
     * Therefore:
     */
    HFI_M1.reference_x =
        -response[0] * cs + response[1] * sn;

    /*
     * The image uses the opposite cross-product direction:
     *
     * e = -alpha_ref*sin(theta_ref)
     *     + beta_ref*cos(theta_ref)
     *
     * No amplitude normalization and no error low-pass.
     */
    HFI_M1.error =
        -HFI_PLL_POLARITY * HFI_M1.reference_x;

    if (fabsf(HFI_M1.response_d) > HFI_MIN_RESPONSE_A)
    {
        if (HFI_M1.good_samples < TF_REGULATION_RATE)
        {
            ++HFI_M1.good_samples;
        }
    }
    else
    {
        HFI_M1.good_samples = 0U;
    }

    /*
     * 3. Direct-error PLL.
     *
     * KI is continuous-time:
     * integral += KI * error * Ts
     */
    omega_limit =
        HFI_EST_LIMIT_RPM
        * HFI_TWO_PI * POLE_PAIR_NUM / 60.0f;

    HFI_M1.integral +=
        HFI_M1.ki * HFI_M1.error * HFI_TS;

    HFI_M1.integral =
        HFI_Limit(HFI_M1.integral, omega_limit);

    HFI_M1.omega =
        HFI_M1.kp * HFI_M1.error
        + HFI_M1.integral;

    /*
     * Additional commissioning speed limit.
     */
    HFI_M1.omega =
        HFI_Limit(HFI_M1.omega, omega_limit);

    /*
     * 4. Electrical-angle integration.
     */
    HFI_M1.theta += HFI_M1.omega * HFI_TS;

    if (HFI_M1.theta >= HFI_PI)
    {
        HFI_M1.theta -= HFI_TWO_PI;
    }

    if (HFI_M1.theta < -HFI_PI)
    {
        HFI_M1.theta += HFI_TWO_PI;
    }

    HFI_M1.sensor.hElAngle =
        HFI_ToAngleDigit(HFI_M1.theta);

    HFI_M1.sensor.hElSpeedDpp =
        (int16_t)(
            HFI_M1.omega * HFI_TS
            * 65536.0f / HFI_TWO_PI);

    HFI_M1.sensor.InstantaneousElSpeedDpp =
        HFI_M1.sensor.hElSpeedDpp;

    /*
     * 5. Output-speed filtering.
     * This filter does not participate in angle integration.
     */
    speed_alpha =
        HFI_LpfCoefficient(HFI_SPEED_LPF_HZ);

    HFI_M1.omega_filtered +=
        speed_alpha
        * (HFI_M1.omega - HFI_M1.omega_filtered);

    HFI_M1.sensor.hAvrMecSpeedUnit =
        (int16_t)(
            HFI_M1.omega_filtered
            * SPEED_UNIT
            / (HFI_TWO_PI * POLE_PAIR_NUM));
}

qd_t HFI_FilterCurrent(qd_t raw)
{
    qd_t filtered;

    float q;
    float d;
    float alpha;

    /*
     * Same three-tap fundamental extraction as the reference:
     *
     * (i[k] + 2*i[k-1] + i[k-2]) / 4
     *
     * Work in the control dq frame, not the raw alpha/beta frame.
     */
    q =
        ((float)raw.q
         + 2.0f * HFI_M1.dq_last.q
         + HFI_M1.dq_llast.q) * 0.25f;

    d =
        ((float)raw.d
         + 2.0f * HFI_M1.dq_last.d
         + HFI_M1.dq_llast.d) * 0.25f;

    HFI_M1.dq_llast = HFI_M1.dq_last;
    HFI_M1.dq_last = raw;

    /*
     * Optional extra dq low-pass.
     * This is separate from the angle-estimation branch.
     */
    if (HFI_CURRENT_LPF_HZ > 0.0f)
    {
        alpha =
            HFI_LpfCoefficient(HFI_CURRENT_LPF_HZ);

        HFI_M1.dq_filtered[0] +=
            alpha * (q - HFI_M1.dq_filtered[0]);

        HFI_M1.dq_filtered[1] +=
            alpha * (d - HFI_M1.dq_filtered[1]);

        q = HFI_M1.dq_filtered[0];
        d = HFI_M1.dq_filtered[1];
    }

    filtered.q = (int16_t)q;
    filtered.d = (int16_t)d;

    return filtered;
}

int16_t HFI_PrepareInjection(float bus_voltage)
{
    float voltage;

    HFI_M1.injection_digit = 0;

    if (!HFI_M1.active || bus_voltage < 8.0f)
    {
        return 0;
    }

    voltage = HFI_M1.injection_v;

    if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }

    if (voltage > 1.0f)
    {
        voltage = 1.0f;
    }

    /*
     * MCSDK phase-voltage scaling.
     */
    HFI_M1.injection_digit =
        (int16_t)(
            voltage * 1.7320508f * 32767.0f
            / bus_voltage);

    return
        (int16_t)(
            HFI_M1.next_sign
            * HFI_M1.injection_digit);
}

void HFI_CommitInjection(void)
{
    HFI_M1.command_history[1] =
        HFI_M1.command_history[0];

    HFI_M1.command_history[0] =
        HFI_M1.next_sign;

    HFI_M1.next_sign =
        (int8_t)-HFI_M1.next_sign;
}

bool HFI_ResponsePresent(void)
{
    return
        HFI_M1.active
        && HFI_M1.good_samples
           > TF_REGULATION_RATE / 20U;
}

/*
 * Call only after stationary alignment,
 * with interrupts already disabled.
 */
void EncoderSyncAfterAlignment(void)
{
    uint32_t count;

    /* TIM4 is a 16-bit timer. Exclude the UIF remap bit. */
    count =
        LL_TIM_GetCounter(ENCODER_M1.TIMx)
        & 0xFFFFU;

    ENC_Clear(&ENCODER_M1);

    ENCODER_M1.PreviousCapture = count;
    ENCODER_M1.DeltaCapturesIndex = 0U;

    LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);

    ENCODER_M1.TimerOverflowNb = 0U;
    ENCODER_M1.TimerOverflowError = false;
    ENCODER_M1.SensorIsReliable = true;

    ENCODER_M1._Super.hAvrMecSpeedUnit = 0;
    ENCODER_M1._Super.hElSpeedDpp = 0;
    ENCODER_M1._Super.InstantaneousElSpeedDpp = 0;
    ENCODER_M1._Super.hMecAccelUnitP = 0;
    ENCODER_M1._Super.bSpeedErrorNumber = 0U;

    /* Keep the angle corresponding to the aligned counter. */
    (void)ENC_CalcAngle(&ENCODER_M1);
}

