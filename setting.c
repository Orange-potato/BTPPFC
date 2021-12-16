/*
 * setting.c
 *
 *  Created on: 2015. 6. 21.
 *      Author: RnA_SSH
 */
#include "F28x_Project.h"	// Ĩ-���� ������� �� ��Ÿ ������ϵ� �ϰ�����
#include "setting.h"	// Ĩ-���� ������� �� ��Ÿ ������ϵ� �ϰ�����

// ������ ���� -----------------------------------------------------------------------
Uint16 EPWM_PERIOD = SW_PERIOD;	// CPU_CLK = 200MHz, EPWM_PERIOD = CPU_CLK/PWM freq. 20 kHz
Uint16 EPWM_PERIOD1 = SW_PERIOD_1;	// CPU_CLK = 200MHz, EPWM_PERIOD = CPU_CLK/PWM freq. 20 kHz

Uint16 PWM_Duty = 0;		// EPWM_PERIOD/2

Uint16 phase = INITIAL_PHASE;




// DAC-A,B �ʱ�ȭ
void InitDAC(void)
{
	EALLOW;
	DacaRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VDAC;
	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VDAC;
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;		//Enable DAC output
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;		//Enable DAC output
	EDIS;
	DacaRegs.DACVALS.bit.DACVALS = 0;		// DAC �ʱ� ��� ����
	DacbRegs.DACVALS.bit.DACVALS = 0;		// DAC �ʱ� ��� ����
}


// ADC ȯ�漳�� �Լ� -----------------------------------------------------------
void InitADC(void)
{
	EALLOW;
	AdcaRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-B ȸ�ΰ� ����� Ŭ�� ���ļ� ���ּ���, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-B ���ش� �ɼ� ����
	AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-B ��ȣ �Է¸�� ����
	AdcbRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-B ȸ�ΰ� ����� Ŭ�� ���ļ� ���ּ���, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-B ���ش� �ɼ� ����
	AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-B ��ȣ �Է¸�� ����
	AdcdRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-D ȸ�ΰ� ����� Ŭ�� ���ļ� ���ּ���, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-D ���ش� �ɼ� ����
	AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-D ��ȣ �Է¸�� ����
	//AdcdRegs.ADCCTL2.bit.PRESCALE = 14;					// ADC-D ȸ�ΰ� ����� Ŭ�� ���ļ� ���ּ���, ADCCLK = SYSCLK /8 = 200MHz /8 = 25MHz
	//AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;	// ADC-D ���ش� �ɼ� ����
	//AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;	// ADC-D ��ȣ �Է¸�� ����

	// ADC ���ͷ�Ʈ �߻� ���� ����
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ��ȯ �Ϸ� �� ���ͷ�Ʈ �޽� ����
	// ADC ȸ�� �⵿ (���� ����)
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	// ADC ���� �⵿�� ���� �����ð� (1 msec)
	DELAY_US(1000);

	// ADC ���ͷ�Ʈ �߻� ���� ����
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ��ȯ �Ϸ� �� ���ͷ�Ʈ �޽� ����
	// ADC ȸ�� �⵿ (���� ����)
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	// ADC ���� �⵿�� ���� �����ð� (1 msec)
	DELAY_US(1000);

	// ADC ���ͷ�Ʈ �߻� ���� ����
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ��ȯ �Ϸ� �� ���ͷ�Ʈ �޽� ����
	// ADC ȸ�� �⵿ (���� ����)
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	// ADC ���� �⵿�� ���� �����ð� (1 msec)
	DELAY_US(1000);

	// ADC ȸ�� �⵿ (���� ����)
	//AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	// ADC ���� �⵿�� ���� �����ð� (1 msec)
	//DELAY_US(1000);

	// SOC0 ��ȯä�� ���� (A0, B0)
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;		// SOC0 - ADCINB2
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5;		// SOC0 - ADCINB1
	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;		// SOC0 - ADCIND0
	AdcdRegs.ADCSOC1CTL.bit.CHSEL = 2;		// SOC0 - ADCIND2
	// ADC ���شɿ� ���� �ּ� Acquisition Window ũ�� ���� (12��Ʈ�� ���)
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;	// 75nsec
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;	// 75nsec
	AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;	// 75nsec
	AdcdRegs.ADCSOC1CTL.bit.ACQPS = 14;	// 75nsec

	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1�� ��ȯ���۽�ȣ(SOC-A/C) ����
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1�� ��ȯ���۽�ȣ(SOC-A/C) ����
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1�� ��ȯ���۽�ȣ(SOC-A/C) ����
	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;	// CPU1 Timer 1�� ��ȯ���۽�ȣ(SOC-A/C) ����

	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC0�� ��ȯ�� �Ϸ�Ǹ� INT2 ǥ��(Flag)��Ʈ�� 1�� ����
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC0�� ��ȯ�� �Ϸ�Ǹ� INT2 ǥ��(Flag)��Ʈ�� 1�� ����
	AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC0�� ��ȯ�� �Ϸ�Ǹ� INT2 ǥ��(Flag)��Ʈ�� 1�� ����
	AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 1;	// SOC0�� ��ȯ�� �Ϸ�Ǹ� INT2 ǥ��(Flag)��Ʈ�� 1�� ����

	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;	// INT2 ǥ��(Flag)��Ʈ�� 1�� �Ǹ� ���ͷ�Ʈ ���� (Interrupt Enable)
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;	// INT2 ǥ��(Flag)��Ʈ�� 1�� �Ǹ� ���ͷ�Ʈ ���� (Interrupt Enable)
	AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;	// INT2 ǥ��(Flag)��Ʈ�� 1�� �Ǹ� ���ͷ�Ʈ ���� (Interrupt Enable)
	AdcdRegs.ADCINTSEL1N2.bit.INT2E = 1;	// INT2 ǥ��(Flag)��Ʈ�� 1�� �Ǹ� ���ͷ�Ʈ ���� (Interrupt Enable)

	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// INT2 ǥ��(Flag)��Ʈ�� 0���� �ʱ�ȭ(Clear)
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// INT2 ǥ��(Flag)��Ʈ�� 0���� �ʱ�ȭ(Clear)
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// INT2 ǥ��(Flag)��Ʈ�� 0���� �ʱ�ȭ(Clear)
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;	// INT2 ǥ��(Flag)��Ʈ�� 0���� �ʱ�ȭ(Clear)
	EDIS;
}
// -----------------------------------------------------------------------------------


void InitEPwm(void)
{
	// GPIO0���� GPIO1���� EPWM1 ���� �����Ͽ� PWM ��¿� ��Ʈ�� ����
		// GPIO2���� GPIO3���� EPWM2 ���� �����Ͽ� PWM ��¿� ��Ʈ�� ����

		//phase = 1000;

		EALLOW;
		GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;	// GPIO0�� ���� ���� Pull-Up ���� (EPWM1A)
		GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;	// GPIO1�� ���� ���� Pull-Up ���� (EPWM1B)
		GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;	// GPIO2�� ���� ���� Pull-Up ���� (EPWM2A)
		GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;	// GPIO3�� ���� ���� Pull-Up ���� (EPWM2B)
		GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;	// GPIO4�� ���� ���� Pull-Up ���� (EPWM3A)
		GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;	// GPIO5�� ���� ���� Pull-Up ���� (EPWM3B)
		//GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;  // GPIO6�� ���� ���� Pull-Up ���� (EPWM4A)
		//GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;  // GPIO7�� ���� ���� Pull-Up ���� (EPWM4B)
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;	// GPIO0�� ���� EPWM1 ����� ��� ��(EPWM1A)���� ����
		GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;	// GPIO1�� ���� EPWM1 ����� ��� ��(EPWM1B)���� ����
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;	// GPIO2�� ���� EPWM2 ����� ��� ��(EPWM2A)���� ����
		GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;	// GPIO3�� ���� EPWM2 ����� ��� ��(EPWM2B)���� ����
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;	// GPIO4�� ���� EPWM2 ����� ��� ��(EPWM3A)���� ����
		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;	// GPIO5�� ���� EPWM2 ����� ��� ��(EPWM3B)���� ����
		//GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; // GPIO6�� ���� EPWM2 ����� ��� ��(EPWM4A)���� ����
		//GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1; // GPIO7�� ���� EPWM2 ����� ��� ��(EPWM4B)���� ����

		CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;			// Disable TBCLKSYNC
		EDIS;

		// EPWM ��� Ÿ�̸ӿ� Ŭ�� ��ȣ ���ļ�, ī���� ��� ���, �ֱ� ����
		// Ÿ�̸� �ֱ� �������� ���� (20kHz @ 200MHz)
		EPwm1Regs.TBPRD = (EPWM_PERIOD - 1);
		EPwm2Regs.TBPRD = (EPWM_PERIOD1 - 1);
		EPwm3Regs.TBPRD = (EPWM_PERIOD1 - 1);

		// ���� �������� �ʱ�ȭ (Phase is 0)
		EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;
		EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;
		EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;

		// Ÿ�̸� ī���� �������� �ʱ�ȭ (Clear counter)
		EPwm1Regs.TBCTR = 0x0000;
		EPwm2Regs.TBCTR = 0x0000;
		EPwm3Regs.TBCTR = 0x0000;

		// ī���� ��� ��� : ��� ���	(Up-count Mode)
		EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
		EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
		EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;


		// Phase enable & disable setting
		EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // ���� �� �ݿ� ��� ����	(Disable phase loading)
		EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // ���� �� �ݿ� ��� ����	(Enable phase loading)
		EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // ���� �� �ݿ� ��� ����	(Enable phase loading)

		// Ÿ�̸� ����� Ŭ�� ���ļ��� �ý��� Ŭ�� ���ļ��� �����ϰ� ���� (200MHz)
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

		EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		//EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;



		// �� �������Ϳ� ���� ������ ��� ����
		// ������ �������� �� ��� Ȱ��ȭ (Enable Shadow Register & Function)
		EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
		EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
		EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

		// Ÿ�̸� ī���Ͱ� 0�� ��ġ�� ��, ������ ���������� ���� Ȱ��(Active) �������Ϳ� �ݿ�
		EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
		EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
		EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

		// �� �������� �ʱⰪ ����
		EPwm1Regs.CMPA.bit.CMPA = PWM_Duty;
		EPwm2Regs.CMPA.bit.CMPA = PWM_Duty;
		EPwm3Regs.CMPA.bit.CMPA = PWM_Duty;

		EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
		EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
		EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

		EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
		EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
		EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;

		// Deadband ����
		EPwm1Regs.DBRED = EPWM1_MIN_DB;
		EPwm1Regs.DBFED = EPWM1_MIN_DB;
		EPwm2Regs.DBRED = EPWM2_MIN_DB;
		EPwm2Regs.DBFED = EPWM2_MIN_DB;
		EPwm3Regs.DBRED = EPWM2_MIN_DB;
		EPwm3Regs.DBFED = EPWM2_MIN_DB;


		// Ÿ�̸� �̺�Ʈ�� ���� PWM ��� �� ���� ����
		// Ÿ�̸� ī���Ͱ� 0�� ��ġ�� ��, PWM A ��Ʈ ���¸� High�� ����
		EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // AQ_SET
		EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // AQ_SET
		EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // AQ_SET
		// Ÿ�̸� ī���Ͱ� ��°�� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� Low�� ����
		EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;

		// EPWM1A�� EPWM1B�� ���� �������� �����ϵ��� ����

		// ADC ��ȯ���۽�ȣ(SOC) ����
		// ��ȯ���۽�ȣ A(EPWMxSOCA) ���� Ȱ��ȭ(Enable)
		// Ÿ�̸� ī���Ͱ� 0(Zero)�� ��ġ�� �� ��ȯ���۽�ȣ A(EPWMxSOCA) ����
		EPwm1Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;
		EPwm2Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;
		EPwm3Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;
		EPwm1Regs.ETSEL.bit.SOCAEN	= 1;
		EPwm2Regs.ETSEL.bit.SOCAEN	= 1;
		EPwm3Regs.ETSEL.bit.SOCAEN	= 1;
		// ��ȯ���۽�ȣ ���� ������ �����Ǹ� �Ź� ��ȣ����
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
