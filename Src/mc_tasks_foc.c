
/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/* USER CODE BEGIN Includes */
#include "hfi.h"
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0.000 * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0 * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */
static uint16_t hfiStartTicks = 0U;
static uint16_t hfiStableTicks = 0U;
static uint16_t hfiBadTicks = 0U;
static uint16_t hfiIdReleaseTicks = 0U;
/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    STO_PLL_Init (&STO_PLL_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_PLL_M1._Super);

    /******************************************************/
    /*   Auxiliary speed sensor component initialization  */
    /******************************************************/
    ENC_Init (&ENCODER_M1);

    /***********************************************************/
    /*   Auxiliary encoder alignment component initialization  */
    /***********************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,&ENCODER_M1);
    pEAC[M1] = &EncAlignCtrlM1;

    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    STC_Clear(pSTC[M1]);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */
    HFI_Motor1Init();
    /* USER CODE END MCboot 2   */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  R3_2_SwitchOffPWM(pwmcHandle[motor]);
  
  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);

  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  (void)ENC_CalcAvrgMecSpeedUnit(&ENCODER_M1, &wAux);
  //(void)STO_PLL_CalcAvrgMecSpeedUnit(&STO_PLL_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
            pSTC[M1]->AUXSPD = (SpeednPosFdbk_Handle_t *) &ENCODER_M1;
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);

              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                STC_Clear(pSTC[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );

              STO_PLL_Clear(&STO_PLL_M1);
              ENC_Clear(&ENCODER_M1);

              FOC_Clear( M1 );

              //  if (EAC_IsAligned(&EncAlignCtrlM1) == false)
              // {
              //   EAC_StartAlignment(&EncAlignCtrlM1);
              //   Mci[M1].State = ALIGNMENT;
              // }
              // else
              // {
              //   VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
              //   FOC_Clear(M1);
              //   Mci[M1].State = START;
              // }  
              EncAlignCtrlM1.EncAligned = false;

              EAC_StartAlignment(&EncAlignCtrlM1);

              Mci[M1].State = ALIGNMENT;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

       case ALIGNMENT:
      {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
              TSK_MF_StopProcessing(M1);
          }
          else
          {
              bool done =
                  EAC_Exec(&EncAlignCtrlM1);

              FOCVars[M1].Iqdref.q = 0;

              FOCVars[M1].Iqdref.d =
                  STC_CalcTorqueReference(pSTC[M1]);

              if (done)
              {
                  uint32_t interruptState =
                      __get_PRIMASK();

                  __disable_irq();

                  EncoderSyncAfterAlignment();

      //             HFI_Reset(EncAlignCtrlM1.hElAngle);

      // #if HFI_USE_FEEDBACK
      //             STC_SetSpeedSensor(
      //                 pSTC[M1], &HFI_M1.sensor);
      // #else
      //             STC_SetSpeedSensor(
      //                 pSTC[M1], &ENCODER_M1._Super);
      // #endif
      //确保闭环正确
      /* 不启动 HFI */
      HFI_Stop();

      /* 电流环角度和速度环反馈均选择编码器 */
      STC_SetSpeedSensor( pSTC[M1], &ENCODER_M1._Super);

                  hfiStartTicks = 0U;
                  hfiStableTicks = 0U;
                  hfiBadTicks = 0U;
                  hfiIdReleaseTicks = 0U;

                  Mci[M1].State = START;

                  __set_PRIMASK(interruptState);
              }
          }

          break;
      }

      //确保闭环正确
      case START:
      {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
              TSK_MF_StopProcessing(M1);
              break;
          }

          /* 再次明确选择编码器反馈 */
          STC_SetSpeedSensor(
              pSTC[M1], &ENCODER_M1._Super);


          /* 清除速度 PI 积分 */
          PID_SetIntegralTerm(
              &PIDSpeedHandle_M1, 0);

          /* 切换到速度控制 */
          STC_SetControlMode(
              pSTC[M1], MCM_SPEED_MODE);

              (void)STC_ExecRamp(pSTC[M1], 0, 0U);
          // /* 用当前编码器速度初始化速度参考 */
          // STC_ForceSpeedReferenceToCurrentSpeed(
          //     pSTC[M1]);

          FOCVars[M1].bDriveInput = INTERNAL;
          FOCVars[M1].UserIdref = 0;

          /*
          * 保留对齐结束时的 Id，
          * 由此前 RUN 中的 200 ms 斜坡释放到零。
          */
          hfiIdReleaseTicks = 0U;
          hfiBadTicks = 0U;
           //确保编码器闭环
          /* 临时测试：明确重新缓存一个非零速度指令 */
MCI_ExecSpeedRamp(
    &Mci[M1],
    300 * SPEED_UNIT / U_RPM,
    1500U);
   

          /* 执行初始化时缓存的速度斜坡 */
          MCI_ExecBufferedCommands(&Mci[M1]);

          Mci[M1].State = RUN;

          break;
      }
       
//       case START:
//         {
//             if (MCI_STOP == Mci[M1].DirectCommand)
//             {
//                 TSK_MF_StopProcessing(M1);
//                 break;
//             }

//             ++hfiStartTicks;

//             FOCVars[M1].Iqdref.q = 0;

//             FOCVars[M1].Iqdref.d =
//                 (int16_t)(HFI_ALIGN_CURRENT_A * CURRENT_CONV_FACTOR);


//                 //临时
//                 {
//     bool response_ok;
//     bool angle_ok;
//     bool speed_ok;

//     uint8_t fail_mask = 0U;

//     response_ok = HFI_ResponsePresent();

//     angle_ok =
//         fabsf(HFI_M1.angle_error_deg) < 10.0f;

//     speed_ok =
//         abs(ENCODER_M1._Super.hAvrMecSpeedUnit)
//         <= 10 * SPEED_UNIT / U_RPM;

//     if (!response_ok)
//     {
//         fail_mask |= 0x01U;
//         ++HFI_M1.start_response_fail;
//     }

//     if (!angle_ok)
//     {
//         fail_mask |= 0x02U;
//         ++HFI_M1.start_angle_fail;
//     }

//     if (!speed_ok)
//     {
//         fail_mask |= 0x04U;
//         ++HFI_M1.start_speed_fail;
//     }

//     HFI_M1.start_fail_mask = fail_mask;

//     if (fail_mask == 0U)
//     {
//         ++hfiStableTicks;
//         ++HFI_M1.start_pass_count;

//         if (hfiStableTicks > HFI_M1.start_stable_peak)
//         {
//             HFI_M1.start_stable_peak = hfiStableTicks;
//         }
//     }
//     else
//     {
//         hfiStableTicks = 0U;
//     }
// }
//             /*
//             * Encoder-assisted commissioning check.
//             * Carrier response alone does not establish correct lock.
//             */
//             if (HFI_ResponsePresent()
//                 && fabsf(HFI_M1.angle_error_deg) < 10.0f
//                 && abs(ENCODER_M1._Super.hAvrMecSpeedUnit)
//                   <= 10 * SPEED_UNIT / U_RPM)
//             {
//                 ++hfiStableTicks;
//             }
//             else
//             {
//                 hfiStableTicks = 0U;
//             } 

//             if (
//                 hfiStartTicks
//                     >= HFI_SETTLE_TIME_MS
//                       * MEDIUM_FREQUENCY_TASK_RATE / 1000U
//                 &&
//                 hfiStableTicks
//                     >= 100U
//                       * MEDIUM_FREQUENCY_TASK_RATE / 1000U)
//             {
//                 PID_SetIntegralTerm(
//                     &PIDSpeedHandle_M1, 0);

//                 STC_SetControlMode(
//                     pSTC[M1], MCM_SPEED_MODE);

//                 STC_ForceSpeedReferenceToCurrentSpeed(
//                     pSTC[M1]);

//                 FOCVars[M1].bDriveInput = INTERNAL;
//                 FOCVars[M1].UserIdref = 0;

//                 /*
//                 * Keep Id continuous here.
//                 * RUN gradually releases it.
//                 */
//                 hfiIdReleaseTicks = 0U;

//                 MCI_ExecBufferedCommands(&Mci[M1]);

//                 Mci[M1].State = RUN;
//             }
//             else if (
//                 hfiStartTicks
//                     >= HFI_START_TIMEOUT_MS
//                       * MEDIUM_FREQUENCY_TASK_RATE / 1000U)
//             {
//                 HFI_Stop();

//                 PWMC_SwitchOffPWM(pwmcHandle[M1]);

//                 MCI_FaultProcessing(
//                     &Mci[M1], MC_START_UP, 0);
//             }

//             break;
//         }

        //case SWITCH_OVER:
//         {
//           if (MCI_STOP == Mci[M1].DirectCommand)
//           {
//             TSK_MF_StopProcessing(M1);
//           }
//           else
//           {
//             int16_t hForcedMecSpeedUnit;

//             /* Compute the virtual speed and positions of the rotor.
//                The function returns true if the virtual speed is in the reliability range */
//             bool FlagEnableClosedLoop = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
//             /* Check if the transition ramp has completed. */
//             bool FlagTransitionPhaseCompleted = VSS_TransitionEnded(&VirtualSpeedSensorM1);
//             FlagEnableClosedLoop = FlagEnableClosedLoop || FlagTransitionPhaseCompleted;

//             /* If any of the above conditions is true, the loop is considered closed.
//                The state machine transitions to the RUN state */
//             if (true == FlagEnableClosedLoop)
//             {
// #if PID_SPEED_INTEGRAL_INIT_DIV == 0
//               PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
// #else
//               PID_SetIntegralTerm(&PIDSpeedHandle_M1,
//                                   (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
//                                   / PID_SPEED_INTEGRAL_INIT_DIV));
// #endif
//               /* USER CODE BEGIN MediumFrequencyTask M1 1 */

//               /* USER CODE END MediumFrequencyTask M1 1 */
//               STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super); /* Observer has converged */
//               FOC_InitAdditionalMethods(M1);
//               FOC_CalcCurrRef(M1);
//               STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
//               MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
//               Mci[M1].State = RUN;
//             }
//             else if ((FlagTransitionPhaseCompleted == true) && (FlagEnableClosedLoop == false))
//             {
//               /* The transition time from Open-Loop to Close-Loop allowed has expired */
//               MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
//             }
//           }
//           break;
//         }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */
            /* Independent commissioning overspeed monitor. */
            if (abs(ENCODER_M1._Super.hAvrMecSpeedUnit)
                > HFI_ENCODER_TRIP_RPM * SPEED_UNIT / U_RPM)
            {
                HFI_Stop();

                PWMC_SwitchOffPWM(pwmcHandle[M1]);

                MCI_FaultProcessing(
                    &Mci[M1], MC_SPEED_FDBK, 0);

                break;
            }

            #if HFI_USE_FEEDBACK

            if (!HFI_ResponsePresent()
                || fabsf(HFI_M1.angle_error_deg) > 30.0f)
            {
                ++hfiBadTicks;
            }
            else
            {
                hfiBadTicks = 0U;
            }

            if (hfiBadTicks
                >= 50U * MEDIUM_FREQUENCY_TASK_RATE / 1000U)
            {
                HFI_Stop();

                PWMC_SwitchOffPWM(pwmcHandle[M1]);

                MCI_FaultProcessing(
                    &Mci[M1], MC_SPEED_FDBK, 0);

                break;
            }

            #endif
            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);
            /*
            * Release alignment Id to zero over 200 ms.
            * This avoids a reference step at START -> RUN.
            */
            {
                uint32_t release_ticks =
                    HFI_ID_RELEASE_MS
                    * MEDIUM_FREQUENCY_TASK_RATE / 1000U;

                float fraction = 0.0f;

                if (release_ticks > 0U
                    && hfiIdReleaseTicks < release_ticks)
                {
                    fraction =
                        1.0f
                        - (float)hfiIdReleaseTicks
                          / (float)release_ticks;

                    ++hfiIdReleaseTicks;
                }

                FOCVars[M1].Iqdref.d =
                    (int16_t)(HFI_ALIGN_CURRENT_A * CURRENT_CONV_FACTOR * fraction);
            }
              FOC_CalcCurrRef(M1);
              if(!SPD_Check(STC_GetSpeedSensor(pSTC[M1])))
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        case SWITCH_OVER:

        case WAIT_STOP_MOTOR:
        {
					  HFI_Stop();

            PWMC_SwitchOffPWM(pwmcHandle[M1]);

            MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);

            break;
//          if (MCI_STOP == Mci[M1].DirectCommand)
//          {
//            TSK_MF_StopProcessing(M1);
//          }
//          else
//          {
//            if (TSK_StopPermanencyTimeHasElapsedM1())
//            {
//              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
//              STO_PLL_Clear(&STO_PLL_M1);
//              ENC_Clear(&ENCODER_M1);
//              VSS_Clear(&VirtualSpeedSensorM1);
//              R3_2_SwitchOnPWM(pwmcHandle[M1]);
//              Mci[M1].State = START;
//            }
//            else
//            {
//              /* Nothing to do */
//            }
//          }
//          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */
  HFI_Stop();
  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;

  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  * @param bMotorNbr Motor reference number defined
  * @retval Number of the  motor instance which FOC loop was executed.
  */

  __weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
    uint16_t result;

    RCM_ReadOngoingConv();
    RCM_ExecNextConv();

    (void)ENC_CalcAngle(&ENCODER_M1);

    result = FOC_CurrControllerM1();

    if (result != MC_NO_FAULTS)
    {
        HFI_Stop();

        PWMC_SwitchOffPWM(pwmcHandle[M1]);

        MCI_FaultProcessing(
            &Mci[M1], result, 0);
    }

    if (HFI_M1.active)
    {
        int32_t delta =
            (int32_t)HFI_M1.sensor.hElAngle
            - ENCODER_M1._Super.hElAngle;

        if (delta > 32767)
        {
            delta -= 65536;
        }
        else if (delta < -32768)
        {
            delta += 65536;
        }

        HFI_M1.angle_error_deg =
            (float)delta * (180.0f / 32768.0f);

        FOCVars[M1].AuxDeltaAngle =
            (int16_t)delta;
    }

    return bMotorNbr;
}
// __weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
// {
//   uint16_t hFOCreturn;
//   /* USER CODE BEGIN HighFrequencyTask 0 */

//   /* USER CODE END HighFrequencyTask 0 */

//   RCM_ReadOngoingConv();
//   RCM_ExecNextConv();
//   Observer_Inputs_t STO_Inputs; /* Only if sensorless main */

//   (void)ENC_CalcAngle(&ENCODER_M1);   /* If not sensorless then 2nd parameter is MC_NULL */

//   STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
//   if (SWITCH_OVER == Mci[M1].State)
//   {
//     if (!REMNG_RampCompleted(pREMNG[M1]))
//     {
//       FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
//     }
//     else
//     {
//       /* Nothing to do */
//     }
//   }
//   else
//   {
//     /* Nothing to do */
//   }
//   /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

//   /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
//   hFOCreturn = FOC_CurrControllerM1();
//   /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

//   /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
//   if(hFOCreturn == MC_DURATION)
//   {
//     MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
//   }
//   else
//   {
//     bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
//     if ((IDLE != Mci[M1].State) && (FAULT_NOW != Mci[M1].State) && (FAULT_OVER != Mci[M1].State))
//     {
//       STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
//       STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
//       (void)STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);
//     }
//     else
//     {
//       /* Nothing to do */
//     }
//     STO_PLL_CalcAvrgElSpeedDpp(&STO_PLL_M1); /* Only in case of Sensor-less */
//     if (false == IsAccelerationStageReached)
//     {
//       STO_ResetPLL(&STO_PLL_M1);
//     }
//     else
//     {
//       /* Nothing to do */
//     }
//     /* Only for sensor-less */
//     if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State))
//     {
//       int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
//       (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
//     }
//       if (RUN == Mci[M1].State) {
//       FOCVars[M1].AuxDeltaAngle=SPD_GetElAngle(STC_GetSpeedSensor(pSTC[M1]))-SPD_GetElAngle(STC_GetAuxSensor(pSTC[M1]));
//     }
//     /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

//     /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
//   }

//   return (bMotorNbr);

// }

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */

 static uint16_t FOC_CurrControllerM1(void)
{
    ab_t phase;

    alphabeta_t raw;
    alphabeta_t output;
    alphabeta_t carrier;

    qd_t raw_dq;
    qd_t current;
    qd_t voltage;
    qd_t injected = {0, 0};

    CircleLimitation_Handle_t limit;

    SpeednPosFdbk_Handle_t *sensor;

    int16_t angle;
    int16_t injection = 0;

    int32_t iq;
    int32_t id;
    int32_t trip;

    uint16_t result = MC_NO_FAULTS;

    float bus_voltage;

    bool enabled;
    bool injecting;

    sensor = STC_GetSpeedSensor(pSTC[M1]);

    enabled =
        PWMC_GetPWMState(pwmcHandle[M1]);

    injecting =
        HFI_ENABLE_INJECTION//enabled
        && HFI_M1.active
        && ((START == Mci[M1].State)
            || (RUN == Mci[M1].State));

    /*
     * 1. Read original phase currents.
     */
    PWMC_GetPhaseCurrents(pwmcHandle[M1], &phase);

    trip = (int32_t)(HFI_CURRENT_TRIP_A * CURRENT_CONV_FACTOR);

    if (enabled && (abs((int32_t)phase.b) > trip || abs(-(int32_t)phase.a - phase.b) > trip
           || abs(-(int32_t)phase.a - phase.b) > trip))
    {
        return MC_OVER_CURR;
    }

    /*
     * 2. Raw alpha/beta currents feed the HFI angle estimator.
     */
    raw = MCM_Clarke(phase);

    if (injecting)
    {
        HFI_Update(raw);
    }

    /*
     * 3. Transform original currents into the active control frame.
     */
    angle = SPD_GetElAngle(sensor);

    raw_dq = MCM_Park(raw, angle);

    /*
     * 4. Fundamental-current extraction in dq frame.
     */
    if (injecting)
    {
        current = HFI_FilterCurrent(raw_dq);
    }
    else
    {
        current = raw_dq;
    }

    iq = HFI_LimitCurrent(FOCVars[M1].Iqdref.q, HFI_IQ_LIMIT_A);
  
    id = HFI_LimitCurrent(FOCVars[M1].Iqdref.d, HFI_ALIGN_CURRENT_A);

    if (enabled)
    {
        voltage.q = PI_Controller( pPIDIq[M1], iq - current.q);

        voltage.d = PI_Controller( pPIDId[M1], id - current.d);
    }
    else
    {
        voltage.q = 0;
        voltage.d = 0;
    }

    /*
     * 5. Reserve voltage-vector margin for injection.
     */
    limit = CircleLimitationM1;

    if (injecting)
    {
        bus_voltage = (float)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super)
            * (float)ADC_REFERENCE_VOLTAGE
            / ((float)VBUS_PARTITIONING_FACTOR
               * 65536.0f);

        injection = HFI_PrepareInjection(bus_voltage);

        limit.MaxModule -= (uint16_t)abs((int32_t)injection);

        if (limit.MaxVd > limit.MaxModule)
        {
            limit.MaxVd = limit.MaxModule;
        }
    }

    voltage = Circle_Limitation(&limit, voltage);

    output = MCM_Rev_Park(voltage, angle);

    /*
     * 6. Inject along HFI's own estimated d axis.
     *
     * With HFI feedback this equals:
     * Vd_total = Vd_fundamental +/- Vinj.
     *
     * With encoder feedback the separate vector addition allows
     * HFI to track independently of the encoder control angle.
     */
    if (injecting)
    {
        injected.q = 0;
        injected.d = injection;

        carrier = MCM_Rev_Park(injected, HFI_M1.sensor.hElAngle);

        output.alpha =
            (int16_t)((int32_t)output.alpha + carrier.alpha);

        output.beta =
            (int16_t)((int32_t)output.beta + carrier.beta);
    }

    /*
     * 7. Submit PWM and record the submitted injection polarity.
     */
    if (enabled)
    {
        result = PWMC_SetPhaseVoltage(pwmcHandle[M1], output);

        if (injecting && result == MC_NO_FAULTS)
        {
            HFI_CommitInjection();
        }
    }

    FOCVars[M1].Iab = phase;
    FOCVars[M1].Ialphabeta = raw;
    FOCVars[M1].Iqd = current;

    /*
     * Vqd: fundamental voltage.
     * Valphabeta: fundamental voltage + injection.
     */
    FOCVars[M1].Vqd = voltage;
    FOCVars[M1].Valphabeta = output;
    FOCVars[M1].hElAngle = angle;

    return result;
}
// inline uint16_t FOC_CurrControllerM1(void)
// {
//   qd_t Iqd, Vqd;
//   ab_t Iab;
//   alphabeta_t Ialphabeta, Valphabeta;
//   int16_t hElAngle;
//   uint16_t hCodeError = MC_NO_FAULTS;
//   SpeednPosFdbk_Handle_t *speedHandle;
//   speedHandle = STC_GetSpeedSensor(pSTC[M1]);
//   hElAngle = SPD_GetElAngle(speedHandle);
//   hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;
//   PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
//   Ialphabeta = MCM_Clarke(Iab);
//   Iqd = MCM_Park(Ialphabeta, hElAngle);
//   if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
//   {
//     Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
//     Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
//   }
//   else
//   {
//     Vqd.q = 0;
//     Vqd.d = 0;
//   }
//   Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
//   hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
//   Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

//   if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
//   {
//     hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
//   }
//   else
//   {
//     /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */

//   }

//   FOCVars[M1].Vqd = Vqd;
//   FOCVars[M1].Iab = Iab;
//   FOCVars[M1].Ialphabeta = Ialphabeta;
//   FOCVars[M1].Iqd = Iqd;
//   FOCVars[M1].Valphabeta = Valphabeta;
//   FOCVars[M1].hElAngle = hElAngle;

//   return (hCodeError);
// }

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

