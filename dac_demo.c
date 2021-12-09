//*****************************************************************************
//
// Copyright: 2019-2021, �Ϻ���ͨ��ѧ����ʵ����Ƽ�����III-A��ѧ��
// File name: dac_demo.c
// Description: 
//    1.����������ڳ������DAC6571оƬ�����Ƿ�������PL0������DAC6571֮SDA��PL1����SCL��
//    2.������λ��DAC������Ϊ��ʮ���ƣ�1023���װ����ұ�4λ�������ʾ����ֵ��
//    3.��1�ź�4�ż��ֱ���ƣ�ʮ���ƣ�DAC�������100�ͼ�100��
//    4.��2�ź�5�ż��ֱ���ƣ�ʮ���ƣ�DAC�������10�ͼ�10��
//    5.��3�ź�6�ż��ֱ���ƣ�ʮ���ƣ�DAC�������1�ͼ�1��
//    6.������̥�ڿγ̳�ʼDEMO�������Բ��ֱ��������Ĵ��빦�ܻ�ۼ���
// Author:	�Ϻ���ͨ��ѧ����ʵ����Ƽ�����III-A��ѧ�飨�ϡ�Ԭ��
// Version: 1.1.0.20210930 
// Date��2021-9-30
// History��2021-9-31�޸�����ע�ͣ�Ԭ��
//
//*****************************************************************************

//*****************************************************************************
//
// ͷ�ļ�
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // ��ַ�궨��
#include "inc/hw_types.h"         // �������ͺ궨�壬�Ĵ������ʺ���
#include "driverlib/debug.h"      // ������
#include "driverlib/gpio.h"       // ͨ��IO�ں궨��
#include "driverlib/pin_map.h"    // TM4Cϵ��MCU��Χ�豸�ܽź궨��
#include "driverlib/sysctl.h"     // ϵͳ���ƶ���
#include "driverlib/systick.h"    // SysTick Driver ԭ��
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver ԭ��
#include "driverlib/adc.h"        // ADC��ص�ģ�� 

#include "tm1638.h"               // �����TM1638оƬ�йصĺ���
#include "DAC6571.h"              // �����DAC6571оƬ�йصĺ���

//*****************************************************************************
//
// �궨��
//
//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTickƵ��Ϊ50Hz����ѭ����ʱ����20ms

#define V_T40ms 2 							 // 0.04s�����ʱ�����ֵ��2��20ms
#define V_T60ms 4
#define V_T100ms	5              // 0.1s�����ʱ�����ֵ��5��20ms
#define V_T500ms	25             // 0.5s�����ʱ�����ֵ��25��20ms
#define V_T240ms 12 
#define DAC_set_current_max 1001
#define current_display_error 5
//*****************************************************************************
//
// ����ԭ������
//
//*****************************************************************************
void GPIOInit(void);        // GPIO��ʼ��
void SysTickInit(void);     // ����SysTick�ж� 
void DevicesInit(void);     // MCU������ʼ����ע���������������
void ADCInit(void);			// ADC��ʼ��
void ADC_Sample(void);  // ADC����
void StateMachine(void); // ״̬��
//*****************************************************************************
//
// ��������
//
//*****************************************************************************

// �����ʱ������
uint8_t clock100ms = 0;
uint8_t clock500ms = 0;
uint8_t clock40ms = 0;
uint8_t clock250ms = 0;

// �����ʱ�������־
uint8_t clock100ms_flag = 0;
uint8_t clock500ms_flag = 0;
uint8_t clock40ms_flag = 0;
uint8_t clock250ms_flag = 0;

// �����ü�����
uint32_t test_counter = 0;

// 8λ�������ʾ�����ֻ���ĸ����
// ע����������λ�������������Ϊ4��5��6��7��0��1��2��3
uint8_t digit[8]={' ',' ',' ',' ','_',' ','_',' '};

// 8λС���� 1��  0��
// ע����������λС����������������Ϊ4��5��6��7��0��1��2��3
uint8_t pnt = 0x11;

// 8��LEDָʾ��״̬��0��1��
// ע������ָʾ�ƴ������������Ϊ7��6��5��4��3��2��1��0
//     ��ӦԪ��LED8��LED7��LED6��LED5��LED4��LED3��LED2��LED1
uint8_t led[] = {1, 1, 1, 1, 1, 1, 1, 0};

// ��ǰ����ֵ
uint8_t key_code = 0;
uint8_t key_cnt = 0;

// DAC6571
uint32_t DAC6571_code = 1023;
uint32_t DAC6571_voltage = 250;
uint8_t  DAC6571_flag = 0;


// ϵͳʱ��Ƶ�� 
uint32_t ui32SysClock;


// AIN2(PE1)  ADC???[0-4095]
uint32_t ui32ADC0Value[2]; 

//??30???????
uint32_t data_u[30];
uint32_t data_i[30];

//?????????????
//?30?????
uint8_t flag_u=0; 
uint8_t flag_i=0;

uint32_t sum_u=0;
uint32_t sum_i=0;
uint32_t mean_u=0;
uint32_t mean_i=0;

// AIN2???(???0.01V) [0.00-3.30]
uint32_t ui32ADC0Voltage; 


// DACת�����趨����
uint32_t DAC_set_current = 0;
uint32_t Last_DAC_set_current = 0;

// ѭ����ʾ
uint8_t display_toggle_cnt = 0;
uint8_t display_toggle_flag = 0;

// ״̬������
uint8_t status = 1;
int error = 0;
uint32_t DAC_RANGE_VALID_MAX = 0;
uint32_t DAC_RANGE_VALID_MIN = 0;
uint8_t valid_flag  = 0 ;
//*****************************************************************************
//
// ������
//
//*****************************************************************************
 int main(void)
{
	uint8_t temp,i;

	DevicesInit();            //  MCU������ʼ��
	
	while (clock100ms < 3);   // ��ʱ>60ms,�ȴ�TM1638�ϵ����
	TM1638_Init();	          // ��ʼ��TM1638
	
    DAC6571_flag = 1;
    
	while (1) // ������ѭ��
	{				
		StateMachine(); // ״̬��
    if (clock100ms_flag == 1)   // ���DAC��ѹ�Ƿ�Ҫ��
		{
			clock100ms_flag = 0;
			// DAC6571_code = 120;
			// DAC �� code �Ĺ�ϵ ��Ҫ������һ��
			//DAC6571_code = (DAC_set_current - 3.78) * 128. / 276.0;
			// �������ʾ	
			if (display_toggle_flag == 1)
			{
				pnt = 0x11;
				digit[4] = mean_u/ 1000; 	     // ??ADC??????
				digit[5] = mean_u / 100 % 10; 	 // ??ADC??????
				digit[6] = mean_u / 10 % 10; 	 // ??ADC??????
				digit[7] = mean_u % 10;           // ??ADC??????
			}
			if (display_toggle_flag == 2)
			{	
				pnt = 0x11;
				digit[4] = mean_i / 1000; 	     // ??ADC??????
				digit[5] = mean_i / 100 % 10; 	 // ??ADC??????
				digit[6] = mean_i / 10 % 10; 	 // ??ADC??????
				digit[7] = mean_i % 10;           // ??ADC??????
			}
			if (display_toggle_flag == 3)
			{
				pnt = 0x1;
				digit[4] = DAC6571_code / 1000 ; 	  // ����ǧλ��
				digit[5] = DAC6571_code / 100 % 10;   // �����λ��
				digit[6] = DAC6571_code / 10 % 10;    // ����ʮλ��
				digit[7] = DAC6571_code % 10;         // �����λ��
      } 
			// ��ʾDACоƬ�趨�ĵ�ѹ
			digit[0] = DAC_set_current / 1000. ; 	  // ����ǧλ��
			digit[1] = DAC_set_current / 100 % 10;   // �����λ��
			digit[2] = DAC_set_current / 10 % 10;    // ����ʮλ��
			digit[3] = DAC_set_current % 10;         // �����λ��
			
			DAC6571_Fastmode_Operation(DAC6571_code); //DACת�������
		}
		
		// ����DEBUG�������
		if (clock500ms_flag == 1)   // ���0.5�붨ʱ�Ƿ�
		{
			clock500ms_flag = 0;
			// 8��ָʾ��������Ʒ�ʽ��ÿ0.5�����ң�ѭ�����ƶ�һ��
			temp = led[0];
			for (i = 0; i < 7; i++) led[i] = led[i + 1];
			led[7] = temp;
		}
		
		// ADC����ת������
		if (clock40ms_flag == 1)        // ??40ms??????  ?80ms
    {
      clock40ms_flag = 0;
            
			sum_u-=data_u[flag_u];
			sum_i-=data_i[flag_i];
			ADC_Sample();
					
			data_u[flag_u]=ui32ADC0Value[0];
			sum_u+=data_u[flag_u];
			flag_u=(flag_u+1)%30;
					
			data_i[flag_i]=ui32ADC0Value[1];
			sum_i+=data_i[flag_i];
			flag_i=(flag_i+1)%30;
						
						
			mean_u=sum_u/30;
			mean_i=sum_i/30;
					
						/*mean_u=sum_u/27;
						mean_i=sum_i/56;*/
					
			mean_u/=1.2432;
			mean_i/=1.2340;
				
			mean_u*=2;
			mean_i/=15;
			mean_i/=0.1017;
						
    }
		
		//Last_DAC_set_current = DAC_set_current; //��¼����һ���趨�ĵ���
		
		
	}
	
}
//*****************************************************************************
//
// ����ԭ�ͣ�void StateMachine(void)
// �������ܣ�
// ������������
// ��������ֵ����
//
//*****************************************************************************
void StateMachine(void) {
	switch(status) {
		case 1: // ״̬һ
			DAC6571_code = (DAC_set_current - 3.78) * 128. / 276.0 ;
			Last_DAC_set_current = DAC_set_current;
		  status = 2;
		  clock250ms = 0;
			clock250ms_flag = 0;
			break;
		case 2: // ״̬�� ����systick������ʱ 
			if (clock250ms_flag) {
				clock250ms_flag = 0;
				status = 3;
			} else {
				status = 2;
			}
			break;
		case 3: // ״̬�� 
			// TODO:�����Ҫ���һ�����ޡ���ֹ����
			if (Last_DAC_set_current == DAC_set_current) {
				if (mean_i > DAC_set_current && mean_u < 5200){
					error = mean_i - DAC_set_current;
					
					if (error < 10) {
							DAC_RANGE_VALID_MAX = DAC6571_code; 
							if (DAC_RANGE_VALID_MAX - DAC_RANGE_VALID_MIN > 0 
								&& DAC_RANGE_VALID_MAX - DAC_RANGE_VALID_MIN < 10)
								valid_flag = 1;
					}
					
					if (valid_flag) {
						DAC6571_code = (DAC_RANGE_VALID_MAX+DAC_RANGE_VALID_MIN) / 2 + 1;
					} else {
						if (error > 20)
							DAC6571_code -= 5;
						else
							DAC6571_code -= 1;
					}
					if (DAC6571_code <= 80 && DAC6571_code >= 72 ) DAC6571_code = 75; // ǿ������
					if (DAC6571_code <= 125 && DAC6571_code >= 118 ) DAC6571_code = 120;
					clock250ms = 0;
					clock250ms_flag = 0;
					status = 2;
				} else if (mean_i < DAC_set_current  && mean_u < 5200){
					error = DAC_set_current - mean_i;
					
					if (error < 10) {
							DAC_RANGE_VALID_MIN = DAC6571_code;
							if (DAC_RANGE_VALID_MAX - DAC_RANGE_VALID_MIN > 0 
								&& DAC_RANGE_VALID_MAX - DAC_RANGE_VALID_MIN < 10)
								valid_flag = 1; 						
					}
					if (valid_flag) {
						DAC6571_code = (DAC_RANGE_VALID_MAX+DAC_RANGE_VALID_MIN) / 2 + 1;
					} else {
						if (error > 20)
							DAC6571_code += 5;
						else
							DAC6571_code += 1;
					}
					if (DAC6571_code <= 80 && DAC6571_code >= 72 ) DAC6571_code = 75;
					if (DAC6571_code <= 125 && DAC6571_code >= 118 ) DAC6571_code = 120;
					clock250ms = 0;
					clock250ms_flag = 0;
					status = 2;
				} else if (mean_i < DAC_set_current && mean_u > 5200) {
					status = 4;
				}
			} else{
				status = 1;
			}
			if (Last_DAC_set_current != DAC_set_current) status = 1;
			Last_DAC_set_current = DAC_set_current;
			break;
		case 4: // ״̬�� ��ѹ����״̬
			if (DAC_set_current > mean_i && mean_u > 5200){
				status = 4;
			} else {
				status = 3;
			}
			break;
		}
		valid_flag = 0;
}


//*****************************************************************************
//
// ����ԭ�ͣ�void GPIOInit(void)
// �������ܣ�GPIO��ʼ����ʹ��PortK������PK4,PK5Ϊ�����ʹ��PortM������PM0Ϊ�����
//          ���װ������ϣ�PK4������TM1638��STB��PK5����TM1638��DIO��PM0����TM1638��CLK��
//          ��ͨ���˹����ӣ�PL0������DAC6571֮SDA��PL1����SCL��
// ������������
// ��������ֵ����
//
//*****************************************************************************
void GPIOInit(void)
{
	//�������ڿ���TM1638оƬ��DAC6571оƬ�Ĺܽ�
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// ʹ�ܶ˿� K	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// �ȴ��˿� K׼�����		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// ʹ�ܶ˿� M	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// �ȴ��˿� M׼�����		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);				// ʹ�ܶ˿� L	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};		// �ȴ��˿� L׼�����		
    
    // ���ö˿� K�ĵ�4,5λ��PK4,PK5��Ϊ�������		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// ���ö˿� M�ĵ�0λ��PM0��Ϊ�������   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);	


    // ���ö˿� L�ĵ�0,1λ��PL0,PL1��Ϊ�������		PL0-SDA  PL1-SCL (DAC6571)
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1);
        
}

//*****************************************************************************
// 
// ����ԭ�ͣ�SysTickInit(void)
// �������ܣ�����SysTick�ж�
// ������������
// ��������ֵ����
//
//*****************************************************************************
void SysTickInit(void)
{
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // ������������,��ʱ����20ms
	SysTickEnable();  			// SysTickʹ��
	SysTickIntEnable();			// SysTick�ж�����
}

//*****************************************************************************
// 
// ����ԭ�ͣ�void DevicesInit(void)
// �������ܣ�CU������ʼ��������ϵͳʱ�����á�GPIO��ʼ����SysTick�ж�����
// ������������
// ��������ֵ����
//
//*****************************************************************************
void DevicesInit(void)
{
	// ʹ���ⲿ25MHz��ʱ��Դ������PLL��Ȼ���ƵΪ20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | 
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 
	                                   1000000);

	GPIOInit();             // GPIO��ʼ��
	ADCInit();
	SysTickInit();          // ����SysTick�ж�
    IntMasterEnable();			// ���ж�����
}

//*****************************************************************************
//
// ����ԭ��:void ADCInit(void)
// ��������:ADC0???? ??AIN2/PE1??ADC??????,?????????????
// ��������:��
// ��������ֵ:��
//
//*****************************************************************************
void ADCInit(void)
{	   
    // ??ADC0??
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // ʹ��AIN2/PE1 AIN1/PE2��ΪADC����,ʹ��E1 E2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2);

    // ????(sample sequence 3??)
   ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	
	
	  //
		ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH2);
	  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_IE |
                             ADC_CTL_END);
	 // ADCSequenceStepConfigure(ADC0_BASE,3,1,ADC_CTL_CH1|ADC_CTL_END|ADC_CTL_IE);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //


    // ʹ�ܵ��β�����ʽ(sample sequence 3)
    ADCSequenceEnable(ADC0_BASE, 1);

    // ����ǰ����жϱ�־λ
    ADCIntClear(ADC0_BASE, 1);		
}

//*****************************************************************************
//
// ����ԭ��:uint32_t ADC_Sample(void)
// ���������:??ADC????
// ��������:��
// ��������ֵ:ADC???[0-4095]
//
//*****************************************************************************
void ADC_Sample(void)
{

    //
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    //
    uint32_t pui32ADC0Value[2];
	
    // ??ADC??
    ADCProcessorTrigger(ADC0_BASE, 1);

    // �ȴ�����ת�����
    while(!ADCIntStatus(ADC0_BASE, 1, false))
    {
    }

    // ���ADC�жϱ�־λ
    ADCIntClear(ADC0_BASE, 1);

    // ��ȡADC����ֵ
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);

    return ;
}

//*****************************************************************************
// 
// ����ԭ�ͣ�void SysTick_Handler(void)
// �������ܣ�SysTick�жϷ������
// ������������
// ��������ֵ����
//
//*****************************************************************************
void SysTick_Handler(void)       // ��ʱ����Ϊ20ms
{
	// 0.1������ʱ������
	if (++clock100ms >= V_T100ms)
	{
		clock100ms_flag = 1; // ��0.1�뵽ʱ�������־��1
		clock100ms = 0;
	}
	// 40ms��ʱ������
	if (++clock40ms >= V_T40ms)
	{
		clock40ms_flag = 1;
		clock40ms = 0;
	}
	
 	// 0.5������ʱ������
	if (++clock500ms >= V_T500ms)
	{
		clock500ms_flag = 1; // ��0.5�뵽ʱ�������־��1
		clock500ms = 0;
	}
	
	if (++clock250ms >= V_T240ms) {
		clock250ms_flag = 1;
		clock250ms = 0;
	}
	// ˢ��ȫ������ܺ�LEDָʾ��
	TM1638_RefreshDIGIandLED(digit, pnt, led);

	// ��鵱ǰ�������룬0�����޼�������1-9��ʾ�ж�Ӧ����
	// ������ʾ��һλ�������
	key_code = TM1638_Readkeyboard();

//	if (key_code != 0)
//	{
//		if (key_cnt < 4) key_cnt++;   // ����������4*20ms
//		else if (key_cnt == 4)
//		{
//			if (key_code == 1)      // ��1
//			{
//				if (DAC6571_code < DAC6571_code_max) 
//				{
//					DAC6571_code++;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 2)  // ��1
//			{
//				if (DAC6571_code > 0) 
//				{
//					DAC6571_code--;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 3)  // ��10
//			{
//				if (DAC6571_code < DAC6571_code_max - 10) 
//				{
//					DAC6571_code += 10;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 4)   // ��10
//			{
//				if (DAC6571_code > 10) 
//				{
//					DAC6571_code -= 10;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 5)   // ��100
//			{
//				if (DAC6571_code < DAC6571_code_max - 100) 
//				{
//					DAC6571_code += 100;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 6)   // ��100
//			{
//				if (DAC6571_code > 100) 
//				{
//					DAC6571_code -= 100;
//					DAC6571_flag = 1;
//				}
//			}

//			key_cnt = 5;   // ����һֱ���ţ�ֻ�ı�һ��
//		}
//	}
//	else key_cnt = 0;
if (key_code != 0)
	{
		if (key_cnt < 4) key_cnt++;   // ����������4*20ms
		else if (key_cnt == 4)
		{
			
            switch(key_code)
            {
                case 1:     // ��100
									if (DAC_set_current < DAC_set_current_max - 100) 
				    {
					     DAC_set_current += 100;
					     DAC6571_flag = 1;
				    }
                    break;
                case 4:    // ��100
                    if (DAC_set_current >= 100) 
				    {
					    DAC_set_current -= 100;
					    DAC6571_flag = 1;
				    }
                    break;
                case 2:    // ��10
                   if (DAC_set_current < DAC_set_current_max - 10) 
				    {
					     DAC_set_current += 10;
					     DAC6571_flag = 1;
				    }                    
                    break;
                case 5:    // ��10
                   if (DAC_set_current >= 10) 
				    {
					    DAC_set_current -= 10;
					    DAC6571_flag = 1;
				    }
                    break;
                case 3:    // ��1
                   if (DAC_set_current < DAC_set_current_max - 1) 
				    {
					     DAC_set_current += 1;
					     DAC6571_flag = 1;
				    }
                    break;
                case 6:    // ��1
                   if (DAC_set_current >= 1) 
				    {
					    DAC_set_current -= 1;
					    DAC6571_flag = 1;
				    }
                    break;
								case 7:
									display_toggle_flag = 1;
									break;
								case 8:
									display_toggle_flag = 2;
									break;
								case 9:
									display_toggle_flag = 3;
									break;
								default:
                    break;
            }
            
			key_cnt = 5;   // ����һֱ���ţ�ֻ�ı�һ��
		}
	}
	else key_cnt = 0;
       
	//digit[5] = key_code;   // ����ֵ

}
