/*
 * setting.c
 *
 *  Created on: 2015. 6. 21.
 *      Author: RnA_SSH
 */
#include "F28x_Project.h"	// 칩-지원 헤더파일 및 기타 헤더파일들 일괄삽입
#include "setting.h"	// 칩-지원 헤더파일 및 기타 헤더파일들 일괄삽입

// 변수들 선언 -----------------------------------------------------------------------
Uint16 EPWM_PERIOD = SW_PERIOD;	// CPU_CLK = 200MHz, EPWM_PERIOD = CPU_CLK/PWM freq. 20 kHz
Uint16 EPWM_PERIOD1 = SW_PERIOD_1;	// CPU_CLK = 200MHz, EPWM_PERIOD = CPU_CLK/PWM freq. 20 kHz

Uint16 PWM_Duty = 0;		// EPWM_PERIOD/2

Uint16 phase = INITIAL_PHASE;




// DAC-A,B 초기화
void InitDAC(void)
{
	EALLOW;
	DacaRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VDAC;
	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VDAC;
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;		//Enable DAC output
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;		//Enable DAC output
	EDIS;
	DacaRegs.DACVALS.bit.DACVALS = 0;		// DAC 초기 출력 설정
	DacbRegs.DACVALS.bit.DACVALS = 0;		// DAC 초기 출력 설정
}


// ADC 환경설정 함수 -----------------------------------------------------------
void InitADC(void)
{
	EALLOW;
	AdcaRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-B 회로가 사용할 클럭 주파수 분주설정, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-B 분해능 옵션 설정
	AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-B 신호 입력모드 설정
	AdcbRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-B 회로가 사용할 클럭 주파수 분주설정, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-B 분해능 옵션 설정
	AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-B 신호 입력모드 설정
	AdcdRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-D 회로가 사용할 클럭 주파수 분주설정, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-D 분해능 옵션 설정
	AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-D 신호 입력모드 설정
	//AdcdRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-D 회로가 사용할 클럭 주파수 분주설정, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	//AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-D 분해능 옵션 설정
	//AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-D 신호 입력모드 설정

	// ADC 인터럽트 발생 시점 설정
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// 변환 완료 후 인터럽트 펄스 생성
	// ADC 회로 기동 (전원 공급)
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	// ADC 정상 기동을 위한 지연시간 (1 msec)
	DELAY_US(1000);

	// ADC 인터럽트 발생 시점 설정
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// 변환 완료 후 인터럽트 펄스 생성
	// ADC 회로 기동 (전원 공급)
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	// ADC 정상 기동을 위한 지연시간 (1 msec)
	DELAY_US(1000);

	// ADC 인터럽트 발생 시점 설정
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// 변환 완료 후 인터럽트 펄스 생성
	// ADC 회로 기동 (전원 공급)
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	// ADC 정상 기동을 위한 지연시간 (1 msec)
	DELAY_US(1000);

	// ADC 회로 기동 (전원 공급)
	//AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	// ADC 정상 기동을 위한 지연시간 (1 msec)
	//DELAY_US(1000);

	// SOC0 변환채널 설정 (A0, B0)
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;		// SOC0 - ADCINB2
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5;		// SOC0 - ADCINB1
	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;		// SOC0 - ADCIND0
	AdcdRegs.ADCSOC1CTL.bit.CHSEL = 2;		// SOC0 - ADCIND2
	// ADC 분해능에 따른 최소 Acquisition Window 크기 설정 (12비트일 경우)
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;	// 75nsec
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;	// 75nsec
	AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;	// 75nsec
	AdcdRegs.ADCSOC1CTL.bit.ACQPS = 14;	// 75nsec

	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1이 변환시작신호(SOC-A/C) 생성
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1이 변환시작신호(SOC-A/C) 생성
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1이 변환시작신호(SOC-A/C) 생성
	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1이 변환시작신호(SOC-A/C) 생성

	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC0의 변환이 완료되면 INT2 표지(Flag)비트를 1로 변경
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC0의 변환이 완료되면 INT2 표지(Flag)비트를 1로 변경
	AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC0의 변환이 완료되면 INT2 표지(Flag)비트를 1로 변경
	AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 1;	// SOC0의 변환이 완료되면 INT2 표지(Flag)비트를 1로 변경

	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;	// INT2 표지(Flag)비트가 1이 되면 인터럽트 생성 (Interrupt Enable)
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;	// INT2 표지(Flag)비트가 1이 되면 인터럽트 생성 (Interrupt Enable)
	AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;	// INT2 표지(Flag)비트가 1이 되면 인터럽트 생성 (Interrupt Enable)
	AdcdRegs.ADCINTSEL1N2.bit.INT2E = 1;	// INT2 표지(Flag)비트가 1이 되면 인터럽트 생성 (Interrupt Enable)

	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// INT2 표지(Flag)비트를 0으로 초기화(Clear)
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// INT2 표지(Flag)비트를 0으로 초기화(Clear)
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// INT2 표지(Flag)비트를 0으로 초기화(Clear)
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;	// INT2 표지(Flag)비트를 0으로 초기화(Clear)
	EDIS;
}
// -----------------------------------------------------------------------------------


void InitEPwm(void)
{
	// GPIO0번과 GPIO1번을 EPWM1 모듈과 연결하여 PWM 출력용 포트로 설정
		// GPIO2번과 GPIO3번을 EPWM2 모듈과 연결하여 PWM 출력용 포트로 설정

		//phase = 1000;

		EALLOW;
		GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;	// GPIO0번 핀의 내부 Pull-Up 해제 (EPWM1A)
		GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;	// GPIO1번 핀의 내부 Pull-Up 해제 (EPWM1B)
		GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;	// GPIO2번 핀의 내부 Pull-Up 해제 (EPWM2A)
		GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;	// GPIO3번 핀의 내부 Pull-Up 해제 (EPWM2B)
		GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;	// GPIO4번 핀의 내부 Pull-Up 해제 (EPWM3A)
		GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;	// GPIO5번 핀의 내부 Pull-Up 해제 (EPWM3B)
		//GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;  // GPIO6번 핀의 내부 Pull-Up 해제 (EPWM4A)
		//GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;  // GPIO7번 핀의 내부 Pull-Up 해제 (EPWM4B)
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;	// GPIO0번 핀을 EPWM1 모듈의 기능 핀(EPWM1A)으로 설정
		GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;	// GPIO1번 핀을 EPWM1 모듈의 기능 핀(EPWM1B)으로 설정
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;	// GPIO2번 핀을 EPWM2 모듈의 기능 핀(EPWM2A)으로 설정
		GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;	// GPIO3번 핀을 EPWM2 모듈의 기능 핀(EPWM2B)으로 설정
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;	// GPIO4번 핀을 EPWM2 모듈의 기능 핀(EPWM3A)으로 설정
		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;	// GPIO5번 핀을 EPWM2 모듈의 기능 핀(EPWM3B)으로 설정
		//GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; // GPIO6번 핀을 EPWM2 모듈의 기능 핀(EPWM4A)으로 설정
		//GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1; // GPIO7번 핀을 EPWM2 모듈의 기능 핀(EPWM4B)으로 설정

		CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;			// Disable TBCLKSYNC
		EDIS;

		// EPWM 모듈 타이머용 클럭 신호 주파수, 카운터 계수 모드, 주기 설정
		// 타이머 주기 레지스터 설정 (20kHz @ 200MHz)
		EPwm1Regs.TBPRD = (EPWM_PERIOD - 1);
		EPwm2Regs.TBPRD = (EPWM_PERIOD1 - 1);
		EPwm3Regs.TBPRD = (EPWM_PERIOD1 - 1);

		// 위상 레지스터 초기화 (Phase is 0)
		EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;
		EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;
		EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;

		// 타이머 카운터 레지스터 초기화 (Clear counter)
		EPwm1Regs.TBCTR = 0x0000;
		EPwm2Regs.TBCTR = 0x0000;
		EPwm3Regs.TBCTR = 0x0000;

		// 카운터 계수 모드 : 상승 계수	(Up-count Mode)
		EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
		EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
		EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;


		// Phase enable & disable setting
		EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // 위상 값 반영 기능 해제	(Disable phase loading)
		EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // 위상 값 반영 기능 설정	(Enable phase loading)
		EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // 위상 값 반영 기능 설정	(Enable phase loading)

		// 타이머 계수용 클럭 주파수를 시스템 클럭 주파수와 동일하게 설정 (200MHz)
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

		EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		//EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;



		// 비교 레지스터에 대한 쉐도우 기능 설정
		// 쉐도우 레지스터 및 기능 활성화 (Enable Shadow Register & Function)
		EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
		EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
		EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

		// 타이머 카운터가 0과 일치할 때, 쉐도우 레지스터의 값을 활성(Active) 레지스터에 반영
		EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
		EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
		EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

		// 비교 레지스터 초기값 설정
		EPwm1Regs.CMPA.bit.CMPA = PWM_Duty;
		EPwm2Regs.CMPA.bit.CMPA = PWM_Duty;
		EPwm3Regs.CMPA.bit.CMPA = PWM_Duty;

		EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
		EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
		EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

		EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
		EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
		EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;

		// Deadband 정의
		EPwm1Regs.DBRED = EPWM1_MIN_DB;
		EPwm1Regs.DBFED = EPWM1_MIN_DB;
		EPwm2Regs.DBRED = EPWM2_MIN_DB;
		EPwm2Regs.DBFED = EPWM2_MIN_DB;
		EPwm3Regs.DBRED = EPWM2_MIN_DB;
		EPwm3Regs.DBFED = EPWM2_MIN_DB;


		// 타이머 이벤트에 의한 PWM 출력 핀 상태 규정
		// 타이머 카운터가 0과 일치할 때, PWM A 포트 상태를 High로 변경
		EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // AQ_SET
		EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // AQ_SET
		EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // AQ_SET
		// 타이머 카운터가 상승계수 중 비교 레지스터 A와 일치할 때, PWM A 포트 상태를 Low로 변경
		EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;

		// EPWM1A와 EPWM1B가 서로 상보적으로 동작하도록 설정

		// ADC 변환시작신호(SOC) 설정
		// 변환시작신호 A(EPWMxSOCA) 생성 활성화(Enable)
		// 타이머 카운터가 0(Zero)과 일치할 때 변환시작신호 A(EPWMxSOCA) 생성
		EPwm1Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;
		EPwm2Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;
		EPwm3Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;
		EPwm1Regs.ETSEL.bit.SOCAEN	= 1;
		EPwm2Regs.ETSEL.bit.SOCAEN	= 1;
		EPwm3Regs.ETSEL.bit.SOCAEN	= 1;
		// 변환시작신호 생성 조건이 충족되면 매번 신호생성
		EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;
		EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;
		EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;

		EALLOW;
		CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;			// Enable TBCLKSYNC
		EDIS;
}

void InitSCI(void)
{
	   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
	   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
}

void scia_fifo_init()
{
   SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                  // No parity,8 char bits,
                                  // async mode, idle-line protocol
   SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                  // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA =1;
   SciaRegs.SCICTL2.bit.RXBKINTENA =1;
   SciaRegs.SCIHBAUD = 0x0000;
   SciaRegs.SCILBAUD = SCI_PRD;
   SciaRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
   SciaRegs.SCIFFTX.all=0xC022;
   SciaRegs.SCIFFRX.all=0x0022;
   SciaRegs.SCIFFCT.all=0x00;

   SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

}


void error(void)
{
   asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}
