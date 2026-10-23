#include "hfi.h"
#include "parameters_conversion.h"
#include "mc_math.h"

#include <math.h>
#include <string.h>

#define HFI_PI      3.14159265358979323846f
#define HFI_TWO_PI  (2.0f * HFI_PI)
#define HFI_DT      (1.0f / (float)TF_REGULATION_RATE)

HFI_Handle_t HFI_M1;

static float HFI_Clamp(float x, float limit)
{
    if (x > limit)
    {
        return limit;
    }

    if (x < -limit)
    {
        return -limit;
    }

    return x;
}

static int16_t HFI_AngleDigit(float theta)
{
    return (int16_t)(theta * (32768.0f / HFI_PI));
}

void HFI_Reset(int16_t angle)
{
    /*
     * Caller must prevent HF interrupt access during reset.
     */
    memset(&HFI_M1, 0, sizeof(HFI_M1));

    HFI_M1.theta =
        (float)angle * (HFI_PI / 32768.0f);

    HFI_M1.sensor.hElAngle = angle;

    HFI_M1.sensor.bElToMecRatio = POLE_PAIR_NUM;
    HFI_M1.sensor.SpeedUnit = SPEED_UNIT;

    HFI_M1.sensor.hMeasurementFrequency =
        TF_REGULATION_RATE_SCALED;

    HFI_M1.sensor.DPPConvFactor = DPP_CONV_FACTOR;

    HFI_M1.sensor.hMaxReliableMecSpeedUnit =
        (uint16_t)(HFI_MAX_EST_RPM * SPEED_UNIT / U_RPM);

    HFI_M1.sensor.hMinReliableMecSpeedUnit = 0U;

    HFI_M1.sensor.hMaxReliableMecAccelUnitP = 65535U;

    HFI_M1.sensor.bMaximumSpeedErrorsNumber =
        M1_SS_MEAS_ERRORS_BEFORE_FAULTS;

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

alphabeta_t HFI_Sample(alphabeta_t raw)
{
    alphabeta_t fundamental = raw;

    float value[2];
    float high[2];
    float demod[2];

    float filter;
    float sn;
    float cs;
    float error;
    float omega_limit;
    float current_scale;

    Trig_Components trig;

    unsigned int axis;

    if (!HFI_M1.active)
    {
        return raw;
    }

    /*
     * Convert MCSDK current digits to amperes.
     */
    value[0] =
        (float)raw.alpha * CURRENT_CONV_FACTOR_INV;

    value[1] =
        (float)raw.beta * CURRENT_CONV_FACTOR_INV;

    current_scale =
        1.0f / (float)CURRENT_CONV_FACTOR_INV;

    HFI_M1.sample_sign =
        HFI_M1.command_history[HFI_SAMPLE_DELAY];

    for (axis = 0U; axis < 2U; ++axis)
    {
        /*
         * HF extraction:
         * (i[k] - 2*i[k-1] + i[k-2]) / 4
         */
        high[axis] =
            (value[axis]
             - 2.0f * HFI_M1.previous[axis]
             + HFI_M1.older[axis]) * 0.25f;

        /*
         * Synchronous demodulation.
         */
        demod[axis] =
            (high[axis] - HFI_M1.high_previous[axis])
            * (float)HFI_M1.sample_sign
            * HFI_DEMOD_SIGN;

        /*
         * Two-tap average rejects the alternating carrier.
         * Group delay is approximately half a control period.
         */
        if (axis == 0U)
        {
            fundamental.alpha =
                (int16_t)((value[axis]
                           + HFI_M1.previous[axis])
                          * 0.5f * current_scale);
        }
        else
        {
            fundamental.beta =
                (int16_t)((value[axis]
                           + HFI_M1.previous[axis])
                          * 0.5f * current_scale);
        }

        HFI_M1.older[axis] =
            HFI_M1.previous[axis];

        HFI_M1.previous[axis] =
            value[axis];

        HFI_M1.high_previous[axis] =
            high[axis];
    }

    if (HFI_M1.samples < 8U)
    {
        ++HFI_M1.samples;
        return raw;
    }

    filter =
        HFI_TWO_PI * HFI_DEMOD_BW_HZ * HFI_DT;

    filter /= 1.0f + filter;

    HFI_M1.response_alpha +=
        filter * (demod[0] - HFI_M1.response_alpha);

    HFI_M1.response_beta +=
        filter * (demod[1] - HFI_M1.response_beta);

    trig =
        MCM_Trig_Functions(
            HFI_AngleDigit(HFI_M1.theta));

    sn = (float)trig.hSin / 32768.0f;
    cs = (float)trig.hCos / 32768.0f;

    /*
     * Existing MCSDK coordinate convention:
     *
     * d = alpha*sin(theta) + beta*cos(theta)
     * q = alpha*cos(theta) - beta*sin(theta)
     */
    HFI_M1.response_d =
        HFI_M1.response_alpha * sn
        + HFI_M1.response_beta * cs;

    HFI_M1.response_q =
        HFI_M1.response_alpha * cs
        - HFI_M1.response_beta * sn;

    if (fabsf(HFI_M1.response_d)
        > HFI_MIN_RESPONSE_A)
    {
        error =
            HFI_PLL_SIGN
            * HFI_M1.response_q
            / fabsf(HFI_M1.response_d);

        error = HFI_Clamp(error, 0.5f);

        if (HFI_M1.good_samples
            < TF_REGULATION_RATE)
        {
            ++HFI_M1.good_samples;
        }
    }
    else
    {
        error = 0.0f;

        HFI_M1.good_samples = 0U;
        HFI_M1.integral = 0.0f;
    }

    HFI_M1.error = error;

    omega_limit =
        HFI_MAX_EST_RPM
        * HFI_TWO_PI
        * POLE_PAIR_NUM
        / 60.0f;

    HFI_M1.integral =
        HFI_Clamp(
            HFI_M1.integral
            + HFI_PLL_KI * error * HFI_DT,
            omega_limit);

    HFI_M1.omega =
        HFI_Clamp(
            HFI_PLL_KP * error
            + HFI_M1.integral,
            omega_limit);

    HFI_M1.theta +=
        HFI_M1.omega * HFI_DT;

    if (HFI_M1.theta >= HFI_PI)
    {
        HFI_M1.theta -= HFI_TWO_PI;
    }

    if (HFI_M1.theta < -HFI_PI)
    {
        HFI_M1.theta += HFI_TWO_PI;
    }

    HFI_M1.sensor.hElAngle =
        HFI_AngleDigit(HFI_M1.theta);

    HFI_M1.sensor.hElSpeedDpp =
        (int16_t)(
            HFI_M1.omega
            * HFI_DT
            * 65536.0f
            / HFI_TWO_PI);

    HFI_M1.sensor.InstantaneousElSpeedDpp =
        HFI_M1.sensor.hElSpeedDpp;

    HFI_M1.speed_filtered +=
        0.01f
        * (HFI_M1.omega
           - HFI_M1.speed_filtered);

    HFI_M1.sensor.hAvrMecSpeedUnit =
        (int16_t)(
            HFI_M1.speed_filtered
            * SPEED_UNIT
            / (HFI_TWO_PI * POLE_PAIR_NUM));

    return fundamental;
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
     * MCSDK voltage conversion:
     *
     * phase voltage amplitude
     * = voltage_digit * Vbus / (sqrt(3) * 32767)
     */
    HFI_M1.injection_digit =
        (int16_t)(
            voltage
            * 1.7320508f
            * 32767.0f
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
    /*
     * This detects carrier response.
     * It does not prove correct rotor-angle lock.
     */
    return
        HFI_M1.active
        && HFI_M1.good_samples
           > TF_REGULATION_RATE / 20U;
}