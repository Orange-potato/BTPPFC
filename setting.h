/*
 * setting.h
 *
 *  Created on: 2015. 6. 21.
 *      Author: RnA_SSH
 */

#ifndef SETTING_H_
#define SETTING_H_


// ADC 분해능 선택용 상수 정의 -------------------------------------------------------
#define RESOLUTION_12BIT	0		// 12비트
#define RESOLUTION_16BIT	1		// 16비트
// -----------------------------------------------------------------------------------

// ADC 신호 입력모드 선택용 상수 정의 ------------------------------------------------
#define SIGNAL_SINGLE		0		// 단동식(Single-ended) 단일 채널 변환
#define SIGNAL_DIFFERENTIAL	1		// 차동식(Differential) 채널 쌍 변환
// -----------------------------------------------------------------------------------

//definitions for selecting DAC reference
#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1
//============================================================================================

// Deadband 크기 정의 ----------------------------------------------------------------
#define EPWM1_MIN_DB   10
#define EPWM2_MIN_DB   50

// Control duty 관련
#define BI_STARTUP_RATE	2000

// Initial Phase 설정
#define INITIAL_PHASE	000

// Setting
#define SWITCH_FREQ		20000

#define TIME_INT_FREQ	167		// Time Interrupt Freq. 10 kHz
#define TIME_INT_PERIOD	100		// Time Interrupt Period 100 us
#define TS_INT_PERIOD	0.0001	// Time Interrupt Period 100 us

#define WAVE_FREQ		60
#define VOL_PEAK		80000	// 100,000 mV (100 V) -> 90 V
//#define CUR_PEAK		2650	// 2,000 mA (Peak Current) 500W(3100) 300W(1900) 200W(1300) 250W(1600) for MPPT
#define CUR_PEAK		1800	// 2,000 mA (Peak Current) 500W(3100) 300W(1900) 200W(1300) 250W(1600)
#define DEL_CUR_CON		1
// 380 --> 650 // 700 --> 380
#define VOL_IN			60000	//  60,000 mV (60 V)
#define AMP_RATIO		63.82	// Sensor Voltage Ratio	3000/47
#define AMP_RATIO_1		200		// Sensor Voltage Ratio	3000/15
#define OFFSET_1		99		// Current sensor offset
#define RC_STOP			150		// 52 optimal
//#define AMP_RATIO_2		0.875		// Sensor Current Ratio	1.25  5A/4V
#define AMP_RATIO_2		1.25		// Sensor Current Ratio	1.25  5A/4V
#define CUR_OFFSET		59.0
#define GAIN_RATIO		1000000	// 10^6 Gain Ratio
#define SW_PERIOD		5000	// 10000/200 MHz : 20 kHz / 5000/200 MHz : 40 kHz / 2500/200 MHz : 80 kHz
#define SW_PERIOD_1		5000	// 10000/200 MHz : 20 kHz / 5000/200 MHz : 40 kHz / 2500/200 MHz : 80 kHz
#define LAGGING_PF		0
#define LEADING_PF		1


#define	TWO_PI		6.283185307179586476925286766559
// -----------------------------------------------------------------------------------

#define CPU_FREQ    60E6
#define LSPCLK_FREQ CPU_FREQ/4
#define SCI_FREQ    100E3
#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1
#define P_R_CONST	30

#define INDUCTANCE_2	0.000693	// 693 uH Inductance
#define CAPACITANCE_2	0.00000047	// 0.47 uF Capacitance
//#define initial_phase	1000

// Initial 함수 정의 -----------------------------------------------------------------
void InitADC(void);			// ADC 초기화 함수
void InitEPwm(void);			// 발열제어 회로를 위한 EPWM 모듈 초기화 함수
void InitSCI(void);
void scia_fifo_init(void);
void error(void);

//extern int16 ab;

#endif /* SETTING_H_ */
