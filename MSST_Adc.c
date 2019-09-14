/*
 * MSST_ADC.c
 *
 *  Created on: Dec 14, 2016
 *      Author: Yang Lei
 */

#include "MSST_PWM.h"
#include "F28x_Project.h"
#include "MSST_GlobalFunctions.h"
#include "Syncopation_SCI.h"
#include "Syncopation_Data.h"

Uint16 i_dc_1_adc;
float I_dc_1;

Uint16 led_count=0;

void Adc_A_Init();
void Adc_B_Init();
void ControlLoop();

Uint16 V_DC;
Uint16 I_DC_1;
Uint16 I_DC_2;
Uint16 I_AC_1;
Uint16 I_AC_2;
Uint16 LIGHT;

Uint16 TEMP_1;
Uint16 TEMP_2;
Uint16 TEMP_3;
Uint16 TEMP_4;

void AdcInit()
{
    Adc_A_Init();
    Adc_B_Init();

    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;  // ADC-B interrupt 1
    EALLOW;
    PieVectTable.ADCB1_INT = &ControlLoop;
    EDIS;
}

void Adc_A_Init()
{
    EALLOW;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // ADC-A power up
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt position at the end of conversion
    AdcaRegs.ADCCTL2.bit.PRESCALE = 2; // ADC Clock = SYSCLK / 2;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0; // Single-ended signal mode
    AdcaRegs.ADCBURSTCTL.bit.BURSTEN = 0; // Disable burst mode
    AdcaRegs.ADCINTFLGCLR.all = 15; // Clear all the interrupt flags
    AdcaRegs.ADCINTOVFCLR.all = 15; // Clear all the interrupt overflow flags
    AdcaRegs.ADCINTSEL1N2.all = 0; //  Enable ADCINT1 and it is triggered by EOC3, disable ADCINT2
    AdcaRegs.ADCINTSEL3N4.all = 0; //  Disable ADCINT3 and ADCINT4
    AdcaRegs.ADCSOCPRICTL.all = 0; // Round robin control. Conversion starts from SOC0.
    AdcaRegs.ADCINTSOCSEL1.all = 0; // ADC interrupt doesn't trigger any SOC
    AdcaRegs.ADCINTSOCSEL2.all = 0; // ADC interrupt doesn't trigger any SOC

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // SOC-0 software start
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // SOC-0 convert channel 2, which is ADC_A2 pin
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // SOC-1 software start
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; // SOC-1 convert channel 3, which is ADC_A3 pin
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // SOC-2 software start
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4; // SOC-2 convert channel 4, which is ADC_A4 pin
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // SOC-3 software start
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5; // SOC-3 convert channel 5, which is ADC_A5 pin
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles
    EDIS;
}

void Adc_B_Init()
{
    EALLOW;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // ADC-A power up
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt position at the end of conversion
    AdcbRegs.ADCCTL2.bit.PRESCALE = 2; // ADC Clock = SYSCLK / 2;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0; // Single-ended signal mode
    AdcbRegs.ADCBURSTCTL.bit.BURSTEN = 0; // Disable burst mode
    AdcbRegs.ADCINTFLGCLR.all = 15; // Clear all the interrupt flags
    AdcbRegs.ADCINTOVFCLR.all = 15; // Clear all the interrupt overflow flags
    AdcbRegs.ADCINTSEL1N2.all = 0x0025; //  Enable ADCINT1 and it is triggered by EOC5, disable ADCINT2
    AdcbRegs.ADCINTSEL3N4.all = 0; //  Disable ADCINT3 and ADCINT4
    AdcbRegs.ADCSOCPRICTL.all = 0; // Round robin control. Conversion starts from SOC0.
    AdcbRegs.ADCINTSOCSEL1.all = 0; // ADC interrupt doesn't trigger any SOC
    AdcbRegs.ADCINTSOCSEL2.all = 0; // ADC interrupt doesn't trigger any SOC

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // SOC-0 EPWM7 SOCA
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2; // SOC-0 convert channel 2, which is ADC_B2 pin
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // SOC-1 EPWM7 SOCA
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3; // SOC-1 convert channel 3, which is ADC_B3 pin
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // SOC-2 EPWM7 SOCA
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4; // SOC-2 convert channel 4, which is ADC_B4 pin
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // SOC-3 EPWM7 SOCA
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5; // SOC-3 convert channel 5, which is ADC_B5 pin
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 5; // SOC-4 EPWM7 SOCA
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 14; // SOC-3 convert channel 14, which is ADC_14 pin
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 5; // SOC-5 EPWM7 SOCA
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 15; // SOC-3 convert channel 15, which is ADC_15 pin
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = 19; // Sample window is 49+1 clock cycles

    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // ADCINT1 trips after AdcResults latch
    AdcbRegs.ADCINTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT  = 0;    // Disable ADCINT1 Continuous mode
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL   = 5;    // setup EOC3 to trigger ADCINT1 to fire

    EDIS;
}

// Feedback signals
Uint16 Prd = 0;
Uint16 Duty = 0;
Uint16 Duty_Vac_sec = 0;
Uint16 Duty_I_d_ref = 0;

#define FAULT_RELEASE   GpioDataRegs.GPASET.bit.GPIO18 = 1
#define FAULT_TRIP      GpioDataRegs.GPACLEAR.bit.GPIO18 = 1

float Iac = 0;
float Vdc = 0;

Uint16 dab_state = 0;
Uint16 Status = 0;

#define DAB_INIT     0
#define DAB_STANDBY  1
#define DAB_NORMAL   2
#define DAB_FAULT    3

void dab_init();
void dab_standby();
void dab_normal();
void dab_fault();

float Dab_Idc = 0;
int16 dab_phs = 0;

float Dab_Idc_ref = 0;
float Dab_Idc_buf = 0;
float Dab_Idc_error = 0;
float Dab_Idc_inte = 0;

float Dab_kp = 10;
float Dab_ki = 0.1;

//float Dab_kp = 3;
//float Dab_ki = 0.5;

extern Uint16 dab_prd;

extern void Dab_Update();

// I_loop_variables
#define TS 2e-5

//float i_kp = 40;
//float i_kr = 100;
float i_kp = 20;
float i_kr = 50;
float i_sogi_x1 = 0;
float i_sogi_x2 = 0;
float i_dc_offset_inte = 0;
float i_sogi_error;
float i_omega_h1_T;
// End of I_loop_variables

#pragma CODE_SECTION(current_loop, ".TI.ramfunc");
float current_loop(float Iac_ref, float Iac, float Freq)
{
    i_omega_h1_T = Freq * TS;

    float i_sogi_h1_x1_n;
    float i_sogi_h1_x2_n;

    i_sogi_error = -Iac_ref + Iac;

    i_sogi_h1_x1_n = __cospuf32(i_omega_h1_T) * i_sogi_x1 - __sinpuf32(i_omega_h1_T) * i_sogi_x2 + TS * 377 * i_sogi_error;
    i_sogi_h1_x2_n = __sinpuf32(i_omega_h1_T) * i_sogi_x1 + __cospuf32(i_omega_h1_T) * i_sogi_x2;

    i_sogi_x1 = i_sogi_h1_x1_n;
    i_sogi_x2 = i_sogi_h1_x2_n;

    float Vac_ref = i_kp * i_sogi_error + i_kr * i_sogi_x1;

    return Vac_ref;
}

// Variables for low voltage AC
Uint16 inverter_state = 0;
float Vac_sec = 0;
float Theta_sec = 0;
float Freq_sec = 60;
float Vac_sec_amp = 0;
float Vac_sec_ref = 0;
float Iac_sec_ref = 0;

extern float MSST_PLL(float Vac, float *Freq, float *Vac_amp);

Uint16 log_state = 0;
Uint16 log_index = 0;
Uint16 log_pos_index = 0;
Uint16 log_limit = 4000;
Uint16 event_index = 0;
Uint16 send_state = 0;
Uint16 send_start = 0;
Uint16 send_index = 0;

Uint16 inverter_state_buf = 0;

#pragma CODE_SECTION(ControlLoop, ".TI.ramfunc");
__interrupt void ControlLoop(void)
{
    V_DC = AdcbResultRegs.ADCRESULT1;
    I_DC_1 = AdcbResultRegs.ADCRESULT2;
    I_DC_2 = AdcbResultRegs.ADCRESULT0;
    I_AC_1 = AdcbResultRegs.ADCRESULT4;
    I_AC_2 = AdcbResultRegs.ADCRESULT5;
    LIGHT = AdcbResultRegs.ADCRESULT3;

    TEMP_1 = AdcaResultRegs.ADCRESULT0;
    TEMP_2 = AdcaResultRegs.ADCRESULT1;
    TEMP_3 = AdcaResultRegs.ADCRESULT2;
    TEMP_4 = AdcaResultRegs.ADCRESULT3;

    Prd = (ECap1Regs.CAP2 + ECap1Regs.CAP4) >> 1;
    Duty = (ECap1Regs.CAP1 + ECap1Regs.CAP3) >> 1;

    if(Duty)
    // L1_SEC
//    Dab_Idc = -0.0076785131 * I_DC_1 + 15.4710225296;
//    Vdc = 0.2965626611 * V_DC + 0.0219840284;
    // L2_SEC
//    Dab_Idc = -0.0075655198 * I_DC_1 + 15.5273564490;
//    Vdc = 0.2934196545 * V_DC - 0.5341295089;
    // L3_SEC
    Dab_Idc = -0.0075653312 * I_DC_1 + 15.4387459332;
    Vdc = 0.2954279377 * V_DC - 0.1407289198;

    if((dab_state > 0) && ((Iac < -15)||(Iac > 15)))
        Status |= (1 << 1);

    if((dab_state > 0) && (Vdc > 900))
        Status |= (1 << 2);

    if((dab_state > 0) && ((Dab_Idc < -20)||(Dab_Idc > 20)))
        Status |= (1 << 3);

    if((dab_state > 1) && (!((Duty > 50) && (Duty < 3950))))
        Status |= (1 << 4);

    if(Status)
    {
        Status |= 1;
        dab_state = DAB_FAULT;
    }

    switch(dab_state)
    {
    case DAB_INIT:    dab_init();           break;
    case DAB_STANDBY: dab_standby();        break;
    case DAB_NORMAL:  dab_normal();         break;
    case DAB_FAULT:   dab_fault();          break;
    default:          dab_fault();          break;
    }

    Dab_Update();


    Iac = -0.0253515732 * I_AC_2 + 52.1520863497;


    Duty_Vac_sec = ECap3Regs.CAP1;
    Duty_I_d_ref = ECap2Regs.CAP1;
    Vac_sec = ((float)Duty_Vac_sec - 2000.0) * 0.25;
    Theta_sec = MSST_PLL(Vac_sec, &Freq_sec, &Vac_sec_amp);

    inverter_state_buf = inverter_state;

    if((Duty_I_d_ref < 50) || (Vac_sec_amp < 100) || Status)
    {
        inverter_state = 0;
        Rectifier_DIS();
    }
    else
    {
        inverter_state = 1;
        Iac_sec_ref = ((float)Duty_I_d_ref - 2000.0) * 0.02 * __cospuf32(Theta_sec);
        Vac_sec_ref = current_loop(Iac_sec_ref, Iac, Freq_sec);
        float duty = Vac_sec_ref / Vdc;
        RectDuty_SET(duty);
        Rectifier_EN();
    }

    switch(log_state)
    {
    case 0: {
        DataLog_Logging(log_index, Vac_sec, Vac_sec_ref, Iac, Vdc);
        log_index++;
        if(log_index>=1000)
            log_index = 0;

        if((inverter_state == 1) && (inverter_state_buf == 0))
        {
            event_index = log_index;
            log_state = 1;
        }
        break;
    }
    case 1: {
        DataLog_Logging(log_index, Vac_sec, Vac_sec_ref, Iac, Vdc);
        log_index++;
        if(log_index>=1000)
            log_index = 0;

        log_pos_index++;
        if(log_pos_index >= 800)
        {
            log_state = 2;
            log_pos_index = 0;
        }
        break;
    }
    case 2: {
        break;
    }
    default: {
        break;
    }
    }


    led_count++;
    if(led_count>=20000)
    {
        led_count = 0;
        CPU_LED_TOGGLE = 1;
    }

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

Uint16 dab_init_count = 10;
void dab_init()
{
    Pwm_DIS();
    if(dab_init_count > 0)
        dab_init_count--;
    else
    {
        FAULT_RELEASE;
        dab_state = DAB_STANDBY;
    }
}

void dab_standby()
{
    Pwm_DIS();
    if(Duty < 50)
        dab_state = DAB_STANDBY;
    else
    {
        dab_phs = 0;
        DabPri_EN();
        dab_state = DAB_NORMAL;
    }
}

void dab_normal()
{
    if(Duty < 50)
    {
        Pwm_DIS();
        dab_state = DAB_STANDBY;
    }
    else
    {
        float Vdc_pri = 0.2965626611 * (Duty-50) + 0.0219840284;
        Dab_Idc_ref = 0.5 * (Vdc - Vdc_pri);    // Virtual resistance is 1 ohm.
        if(Dab_Idc_ref > 14)
            Dab_Idc_ref = 14;
        if(Dab_Idc_ref < -14)
            Dab_Idc_ref = -14;

        Dab_Idc_error = Dab_Idc_ref - Dab_Idc;
        Dab_Idc_inte += Dab_ki * Dab_Idc_error;
        dab_phs = -Dab_kp * (Dab_Idc_error + Dab_Idc_inte);

        if(dab_phs > 500)
            dab_phs = 500;
        if(dab_phs < -500)
            dab_phs = -500;
    }
}

void dab_fault()
{
    Pwm_DIS();
    FAULT_TRIP;
}

//void dab_pri_off()
//{
//    EPwm9Regs.CMPA.bit.CMPA = 20;
//}
//
//void dab_pri_ss()
//{
//    if(dab_duty < 985)
//    {
//        dab_duty++;
//        EPwm7Regs.CMPA.bit.CMPA = dab_duty;
//        EPwm7Regs.CMPB.bit.CMPB = 2000 - dab_duty;
//        EPwm8Regs.CMPA.bit.CMPA = dab_duty;
//        EPwm8Regs.CMPB.bit.CMPB = 2000 - dab_duty;
//    }
//    else
//        dab_pri_state = 2;
//}
//
//void dab_pri_on()
//{
//    EPwm9Regs.CMPA.bit.CMPA = V_DC + 50;
//}
//
//void dab_pri_fault()
//{
//    EPwm9Regs.CMPA.bit.CMPA = 10;
//}
