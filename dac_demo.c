//*****************************************************************************
//
// Copyright: 2019-2021, ÉÏº£½»Í¨´óÑ§¹¤³ÌÊµ¼ùÓë¿Æ¼¼´´ĞÂIII-A½ÌÑ§×é
// File name: dac_demo.c
// Description: 
//    1.±¾´úÂë¿ÉÓÃÓÚ³õ²½¼ì²éDAC6571Ğ¾Æ¬¹¦ÄÜÊÇ·ñÕı³££¬PL0ĞëÁ¬ÏßDAC6571Ö®SDA£¬PL1Á¬ÏßSCL£»
//    2.¿ª»ú»ò¸´Î»ºó£¬DAC±àÂëÖÃÎª£¨Ê®½øÖÆ£©1023£¬µ×°åÉÏÓÒ±ß4Î»ÊıÂë¹ÜÏÔÊ¾¸ÃÊıÖµ£»
//    3.ÓÉ1ºÅºÍ4ºÅ¼ü·Ö±ğ¿ØÖÆ£¨Ê®½øÖÆ£©DAC±àÂëÆä¼Ó100ºÍ¼õ100£»
//    4.ÓÉ2ºÅºÍ5ºÅ¼ü·Ö±ğ¿ØÖÆ£¨Ê®½øÖÆ£©DAC±àÂëÆä¼Ó10ºÍ¼õ10£»
//    5.ÓÉ3ºÅºÍ6ºÅ¼ü·Ö±ğ¿ØÖÆ£¨Ê®½øÖÆ£©DAC±àÂëÆä¼Ó1ºÍ¼õ1£»
//    6.´úÂëÍÑÌ¥ÓÚ¿Î³Ì³õÊ¼DEMO³ÌĞò£¬ËùÒÔ²¿·Ö±£ÁôÁËËüµÄ´úÂë¹¦ÄÜ»òºÛ¼££»
// Author:	ÉÏº£½»Í¨´óÑ§¹¤³ÌÊµ¼ùÓë¿Æ¼¼´´ĞÂIII-A½ÌÑ§×é£¨ÃÏ¡¢Ô¬£©
// Version: 1.1.0.20210930 
// Date£º2021-9-30
// History£º2021-9-31ĞŞ¸ÄÍêÉÆ×¢ÊÍ£¨Ô¬£©
//
//*****************************************************************************

//*****************************************************************************
//
// Í·ÎÄ¼ş
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // »ùÖ·ºê¶¨Òå
#include "inc/hw_types.h"         // Êı¾İÀàĞÍºê¶¨Òå£¬¼Ä´æÆ÷·ÃÎÊº¯Êı
#include "driverlib/debug.h"      // µ÷ÊÔÓÃ
#include "driverlib/gpio.h"       // Í¨ÓÃIO¿Úºê¶¨Òå
#include "driverlib/pin_map.h"    // TM4CÏµÁĞMCUÍâÎ§Éè±¸¹Ü½Åºê¶¨Òå
#include "driverlib/sysctl.h"     // ÏµÍ³¿ØÖÆ¶¨Òå
#include "driverlib/systick.h"    // SysTick Driver Ô­ĞÍ
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver Ô­ĞÍ
#include "driverlib/adc.h"        // ADCÏà¹ØµÄÄ£¿é 

#include "tm1638.h"               // Óë¿ØÖÆTM1638Ğ¾Æ¬ÓĞ¹ØµÄº¯Êı
#include "DAC6571.h"              // Óë¿ØÖÆDAC6571Ğ¾Æ¬ÓĞ¹ØµÄº¯Êı

//*****************************************************************************
//
// ºê¶¨Òå
//
//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTickÆµÂÊÎª50Hz£¬¼´Ñ­»·¶¨Ê±ÖÜÆÚ20ms

#define V_T40ms 2 							 // 0.04sÈí¼ş¶¨Ê±Æ÷Òç³öÖµ£¬2¸ö20ms
#define V_T60ms 4
#define V_T100ms	5              // 0.1sÈí¼ş¶¨Ê±Æ÷Òç³öÖµ£¬5¸ö20ms
#define V_T500ms	25             // 0.5sÈí¼ş¶¨Ê±Æ÷Òç³öÖµ£¬25¸ö20ms
#define V_T240ms 12 
#define DAC_set_current_max 1001
#define current_display_error 5
//*****************************************************************************
//
// º¯ÊıÔ­ĞÍÉùÃ÷
//
//*****************************************************************************
void GPIOInit(void);        // GPIO³õÊ¼»¯
void SysTickInit(void);     // ÉèÖÃSysTickÖĞ¶Ï 
void DevicesInit(void);     // MCUÆ÷¼ş³õÊ¼»¯£¬×¢£º»áµ÷ÓÃÉÏÊöº¯Êı
void ADCInit(void);			// ADC³õÊ¼»¯
void ADC_Sample(void);  // ADC²ÉÑù
void StateMachine(void); // ×´Ì¬»ú
//*****************************************************************************
//
// ±äÁ¿¶¨Òå
//
//*****************************************************************************

// Èí¼ş¶¨Ê±Æ÷¼ÆÊı
uint8_t clock100ms = 0;
uint8_t clock500ms = 0;
uint8_t clock40ms = 0;
uint8_t clock250ms = 0;

// Èí¼ş¶¨Ê±Æ÷Òç³ö±êÖ¾
uint8_t clock100ms_flag = 0;
uint8_t clock500ms_flag = 0;
uint8_t clock40ms_flag = 0;
uint8_t clock250ms_flag = 0;

// ²âÊÔÓÃ¼ÆÊıÆ÷
uint32_t test_counter = 0;

// 8Î»ÊıÂë¹ÜÏÔÊ¾µÄÊı×Ö»ò×ÖÄ¸·ûºÅ
// ×¢£º°åÉÏÊıÂëÎ»´Ó×óµ½ÓÒĞòºÅÅÅÁĞÎª4¡¢5¡¢6¡¢7¡¢0¡¢1¡¢2¡¢3
uint8_t digit[8]={' ',' ',' ',' ','_',' ','_',' '};

// 8Î»Ğ¡Êıµã 1ÁÁ  0Ãğ
// ×¢£º°åÉÏÊıÂëÎ»Ğ¡Êıµã´Ó×óµ½ÓÒĞòºÅÅÅÁĞÎª4¡¢5¡¢6¡¢7¡¢0¡¢1¡¢2¡¢3
uint8_t pnt = 0x11;

// 8¸öLEDÖ¸Ê¾µÆ×´Ì¬£¬0Ãğ£¬1ÁÁ
// ×¢£º°åÉÏÖ¸Ê¾µÆ´Ó×óµ½ÓÒĞòºÅÅÅÁĞÎª7¡¢6¡¢5¡¢4¡¢3¡¢2¡¢1¡¢0
//     ¶ÔÓ¦Ôª¼şLED8¡¢LED7¡¢LED6¡¢LED5¡¢LED4¡¢LED3¡¢LED2¡¢LED1
uint8_t led[] = {1, 1, 1, 1, 1, 1, 1, 0};

// µ±Ç°°´¼üÖµ
uint8_t key_code = 0;
uint8_t key_cnt = 0;

// DAC6571
uint32_t DAC6571_code = 1023;
uint32_t DAC6571_voltage = 250;
uint8_t  DAC6571_flag = 0;


// ÏµÍ³Ê±ÖÓÆµÂÊ 
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


// DAC×ªÂëºóµÄÉè¶¨µçÁ÷
uint32_t DAC_set_current = 0;
uint32_t Last_DAC_set_current = 0;

// Ñ­»·ÏÔÊ¾
uint8_t display_toggle_cnt = 0;
uint8_t display_toggle_flag = 0;

// ×´Ì¬»ú±äÁ¿
uint8_t status = 1;
int error = 0;
uint32_t DAC_RANGE_VALID_MAX = 0;
uint32_t DAC_RANGE_VALID_MIN = 0;
uint8_t valid_flag  = 0 ;
//*****************************************************************************
//
// Ö÷³ÌĞò
//
//*****************************************************************************
 int main(void)
{
	uint8_t temp,i;

	DevicesInit();            //  MCUÆ÷¼ş³õÊ¼»¯
	
	while (clock100ms < 3);   // ÑÓÊ±>60ms,µÈ´ıTM1638ÉÏµçÍê³É
	TM1638_Init();	          // ³õÊ¼»¯TM1638
	
    DAC6571_flag = 1;
    
	while (1) // ³ÌĞòÖ÷Ñ­»·
	{				
		StateMachine(); // ×´Ì¬»ú
    if (clock100ms_flag == 1)   // ¼ì²éDACµçÑ¹ÊÇ·ñÒª±ä
		{
			clock100ms_flag = 0;
			// DAC6571_code = 120;
			// DAC Óë code µÄ¹ØÏµ »¹ÒªÔÙĞŞÕıÒ»ÏÂ
			//DAC6571_code = (DAC_set_current - 3.78) * 128. / 276.0;
			// ÊıÂë¹ÜÏÔÊ¾	
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
				digit[4] = DAC6571_code / 1000 ; 	  // ¼ÆËãÇ§Î»Êı
				digit[5] = DAC6571_code / 100 % 10;   // ¼ÆËã°ÙÎ»Êı
				digit[6] = DAC6571_code / 10 % 10;    // ¼ÆËãÊ®Î»Êı
				digit[7] = DAC6571_code % 10;         // ¼ÆËã¸öÎ»Êı
      } 
			// ÏÔÊ¾DACĞ¾Æ¬Éè¶¨µÄµçÑ¹
			digit[0] = DAC_set_current / 1000. ; 	  // ¼ÆËãÇ§Î»Êı
			digit[1] = DAC_set_current / 100 % 10;   // ¼ÆËã°ÙÎ»Êı
			digit[2] = DAC_set_current / 10 % 10;    // ¼ÆËãÊ®Î»Êı
			digit[3] = DAC_set_current % 10;         // ¼ÆËã¸öÎ»Êı
			
			DAC6571_Fastmode_Operation(DAC6571_code); //DAC×ª»»ºóÊä³ö
		}
		
		// ÓÃÓÚDEBUGµÄ×ßÂíµÆ
		if (clock500ms_flag == 1)   // ¼ì²é0.5Ãë¶¨Ê±ÊÇ·ñµ½
		{
			clock500ms_flag = 0;
			// 8¸öÖ¸Ê¾µÆÒÔ×ßÂíµÆ·½Ê½£¬Ã¿0.5ÃëÏòÓÒ£¨Ñ­»·£©ÒÆ¶¯Ò»¸ñ
			temp = led[0];
			for (i = 0; i < 7; i++) led[i] = led[i + 1];
			led[7] = temp;
		}
		
		// ADC±àÂë×ª»»²¿·Ö
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
		
		//Last_DAC_set_current = DAC_set_current; //¼ÇÂ¼ÏÂÉÏÒ»´ÎÉè¶¨µÄµçÁ÷
		
		
	}
	
}
//*****************************************************************************
//
// º¯ÊıÔ­ĞÍ£ºvoid StateMachine(void)
// º¯Êı¹¦ÄÜ£º
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void StateMachine(void) {
	switch(status) {
		case 1: // ×´Ì¬Ò»
			DAC6571_code = (DAC_set_current - 3.78) * 128. / 276.0 ;
			Last_DAC_set_current = DAC_set_current;
		  status = 2;
		  clock250ms = 0;
			clock250ms_flag = 0;
			break;
		case 2: // ×´Ì¬¶ş ÀûÓÃsystick½øĞĞÑÓÊ± 
			if (clock250ms_flag) {
				clock250ms_flag = 0;
				status = 3;
			} else {
				status = 2;
			}
			break;
		case 3: // ×´Ì¬Èı 
			// TODO:Õâ±ßĞèÒªÌí¼ÓÒ»¸öÃÅÏŞ¡£·ÀÖ¹¶¶¶¯
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
					if (DAC6571_code <= 80 && DAC6571_code >= 72 ) DAC6571_code = 75; // Ç¿ĞĞĞŞÕı
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
		case 4: // ×´Ì¬ËÄ ÏŞÑ¹±£»¤×´Ì¬
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
// º¯ÊıÔ­ĞÍ£ºvoid GPIOInit(void)
// º¯Êı¹¦ÄÜ£ºGPIO³õÊ¼»¯¡£Ê¹ÄÜPortK£¬ÉèÖÃPK4,PK5ÎªÊä³ö£»Ê¹ÄÜPortM£¬ÉèÖÃPM0ÎªÊä³ö¡£
//          £¨µ×°å×ßÏßÉÏ£¬PK4Á¬½Ó×ÅTM1638µÄSTB£¬PK5Á¬½ÓTM1638µÄDIO£¬PM0Á¬½ÓTM1638µÄCLK£©
//          £¨Í¨¹ıÈË¹¤Ìø½Ó£¬PL0ĞëÁ¬ÏßDAC6571Ö®SDA£¬PL1Á¬ÏßSCL£©
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void GPIOInit(void)
{
	//ÅäÖÃÓÃÓÚ¿ØÖÆTM1638Ğ¾Æ¬¡¢DAC6571Ğ¾Æ¬µÄ¹Ü½Å
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);				// Ê¹ÄÜ¶Ë¿Ú K	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};		// µÈ´ı¶Ë¿Ú K×¼±¸Íê±Ï		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);				// Ê¹ÄÜ¶Ë¿Ú M	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};		// µÈ´ı¶Ë¿Ú M×¼±¸Íê±Ï		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);				// Ê¹ÄÜ¶Ë¿Ú L	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};		// µÈ´ı¶Ë¿Ú L×¼±¸Íê±Ï		
    
    // ÉèÖÃ¶Ë¿Ú KµÄµÚ4,5Î»£¨PK4,PK5£©ÎªÊä³öÒı½Å		PK4-STB  PK5-DIO
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4|GPIO_PIN_5);
	// ÉèÖÃ¶Ë¿Ú MµÄµÚ0Î»£¨PM0£©ÎªÊä³öÒı½Å   PM0-CLK
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);	


    // ÉèÖÃ¶Ë¿Ú LµÄµÚ0,1Î»£¨PL0,PL1£©ÎªÊä³öÒı½Å		PL0-SDA  PL1-SCL (DAC6571)
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1);
        
}

//*****************************************************************************
// 
// º¯ÊıÔ­ĞÍ£ºSysTickInit(void)
// º¯Êı¹¦ÄÜ£ºÉèÖÃSysTickÖĞ¶Ï
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void SysTickInit(void)
{
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // ÉèÖÃĞÄÌø½ÚÅÄ,¶¨Ê±ÖÜÆÚ20ms
	SysTickEnable();  			// SysTickÊ¹ÄÜ
	SysTickIntEnable();			// SysTickÖĞ¶ÏÔÊĞí
}

//*****************************************************************************
// 
// º¯ÊıÔ­ĞÍ£ºvoid DevicesInit(void)
// º¯Êı¹¦ÄÜ£ºCUÆ÷¼ş³õÊ¼»¯£¬°üÀ¨ÏµÍ³Ê±ÖÓÉèÖÃ¡¢GPIO³õÊ¼»¯ºÍSysTickÖĞ¶ÏÉèÖÃ
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void DevicesInit(void)
{
	// Ê¹ÓÃÍâ²¿25MHzÖ÷Ê±ÖÓÔ´£¬¾­¹ıPLL£¬È»ºó·ÖÆµÎª20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | 
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 
	                                   1000000);

	GPIOInit();             // GPIO³õÊ¼»¯
	ADCInit();
	SysTickInit();          // ÉèÖÃSysTickÖĞ¶Ï
    IntMasterEnable();			// ×ÜÖĞ¶ÏÔÊĞí
}

//*****************************************************************************
//
// º¯ÊıÔ­ĞÍ:void ADCInit(void)
// º¯Êı¹¦ÄÜ:ADC0???? ??AIN2/PE1??ADC??????,?????????????
// º¯Êı²ÎÊı:ÎŞ
// º¯Êı·µ»ØÖµ:ÎŞ
//
//*****************************************************************************
void ADCInit(void)
{	   
    // ??ADC0??
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Ê¹ÓÃAIN2/PE1 AIN1/PE2×÷ÎªADCÊäÈë,Ê¹ÄÜE1 E2
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


    // Ê¹ÄÜµ¥´Î²ÉÑù·½Ê½(sample sequence 3)
    ADCSequenceEnable(ADC0_BASE, 1);

    // ²ÉÑùÇ°Çå¿ÕÖĞ¶Ï±êÖ¾Î»
    ADCIntClear(ADC0_BASE, 1);		
}

//*****************************************************************************
//
// º¯ÊıÔ­ĞÍ:uint32_t ADC_Sample(void)
// º¯Êı¹¦ÄÜÄ:??ADC????
// º¯Êı²ÎÊı:ÎŞ
// º¯Êı·µ»ØÖµ:ADC???[0-4095]
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

    // µÈ´ı²ÉÑù×ª»»Íê³É
    while(!ADCIntStatus(ADC0_BASE, 1, false))
    {
    }

    // Çå¿ÕADCÖĞ¶Ï±êÖ¾Î»
    ADCIntClear(ADC0_BASE, 1);

    // ¶ÁÈ¡ADC²ÉÑùÖµ
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);

    return ;
}

//*****************************************************************************
// 
// º¯ÊıÔ­ĞÍ£ºvoid SysTick_Handler(void)
// º¯Êı¹¦ÄÜ£ºSysTickÖĞ¶Ï·şÎñ³ÌĞò
// º¯Êı²ÎÊı£ºÎŞ
// º¯Êı·µ»ØÖµ£ºÎŞ
//
//*****************************************************************************
void SysTick_Handler(void)       // ¶¨Ê±ÖÜÆÚÎª20ms
{
	// 0.1ÃëÖÓÈí¶¨Ê±Æ÷¼ÆÊı
	if (++clock100ms >= V_T100ms)
	{
		clock100ms_flag = 1; // µ±0.1Ãëµ½Ê±£¬Òç³ö±êÖ¾ÖÃ1
		clock100ms = 0;
	}
	// 40msÈí¶¨Ê±Æ÷¼ÆÊı
	if (++clock40ms >= V_T40ms)
	{
		clock40ms_flag = 1;
		clock40ms = 0;
	}
	
 	// 0.5ÃëÖÓÈí¶¨Ê±Æ÷¼ÆÊı
	if (++clock500ms >= V_T500ms)
	{
		clock500ms_flag = 1; // µ±0.5Ãëµ½Ê±£¬Òç³ö±êÖ¾ÖÃ1
		clock500ms = 0;
	}
	
	if (++clock250ms >= V_T240ms) {
		clock250ms_flag = 1;
		clock250ms = 0;
	}
	// Ë¢ĞÂÈ«²¿ÊıÂë¹ÜºÍLEDÖ¸Ê¾µÆ
	TM1638_RefreshDIGIandLED(digit, pnt, led);

	// ¼ì²éµ±Ç°¼üÅÌÊäÈë£¬0´ú±íÎŞ¼ü²Ù×÷£¬1-9±íÊ¾ÓĞ¶ÔÓ¦°´¼ü
	// ¼üºÅÏÔÊ¾ÔÚÒ»Î»ÊıÂë¹ÜÉÏ
	key_code = TM1638_Readkeyboard();

//	if (key_code != 0)
//	{
//		if (key_cnt < 4) key_cnt++;   // °´¼üÏû¶¶£¬4*20ms
//		else if (key_cnt == 4)
//		{
//			if (key_code == 1)      // ¼Ó1
//			{
//				if (DAC6571_code < DAC6571_code_max) 
//				{
//					DAC6571_code++;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 2)  // ¼õ1
//			{
//				if (DAC6571_code > 0) 
//				{
//					DAC6571_code--;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 3)  // ¼Ó10
//			{
//				if (DAC6571_code < DAC6571_code_max - 10) 
//				{
//					DAC6571_code += 10;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 4)   // ¼õ10
//			{
//				if (DAC6571_code > 10) 
//				{
//					DAC6571_code -= 10;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 5)   // ¼Ó100
//			{
//				if (DAC6571_code < DAC6571_code_max - 100) 
//				{
//					DAC6571_code += 100;
//					DAC6571_flag = 1;
//				}
//			}
//			else if (key_code == 6)   // ¼õ100
//			{
//				if (DAC6571_code > 100) 
//				{
//					DAC6571_code -= 100;
//					DAC6571_flag = 1;
//				}
//			}

//			key_cnt = 5;   // °´¼üÒ»Ö±°´×Å£¬Ö»¸Ä±äÒ»´Î
//		}
//	}
//	else key_cnt = 0;
if (key_code != 0)
	{
		if (key_cnt < 4) key_cnt++;   // °´¼üÏû¶¶£¬4*20ms
		else if (key_cnt == 4)
		{
			
            switch(key_code)
            {
                case 1:     // ¼Ó100
									if (DAC_set_current < DAC_set_current_max - 100) 
				    {
					     DAC_set_current += 100;
					     DAC6571_flag = 1;
				    }
                    break;
                case 4:    // ¼õ100
                    if (DAC_set_current >= 100) 
				    {
					    DAC_set_current -= 100;
					    DAC6571_flag = 1;
				    }
                    break;
                case 2:    // ¼Ó10
                   if (DAC_set_current < DAC_set_current_max - 10) 
				    {
					     DAC_set_current += 10;
					     DAC6571_flag = 1;
				    }                    
                    break;
                case 5:    // ¼õ10
                   if (DAC_set_current >= 10) 
				    {
					    DAC_set_current -= 10;
					    DAC6571_flag = 1;
				    }
                    break;
                case 3:    // ¼Ó1
                   if (DAC_set_current < DAC_set_current_max - 1) 
				    {
					     DAC_set_current += 1;
					     DAC6571_flag = 1;
				    }
                    break;
                case 6:    // ¼õ1
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
            
			key_cnt = 5;   // °´¼üÒ»Ö±°´×Å£¬Ö»¸Ä±äÒ»´Î
		}
	}
	else key_cnt = 0;
       
	//digit[5] = key_code;   // °´¼üÖµ

}
