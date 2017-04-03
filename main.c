#include "MDR32Fx.h"                    // Device header
#include "system_MDR32F9Qx.h"           // Keil::Device:Startup_MDR1986BE9x
#include "MDR32F9Qx_uart.h"             // Keil::Drivers:UART
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "S25FK216K.h"

#define PORT_Pin_0                  0x0001U  /*!< Pin 0 selected */
#define PORT_Pin_1                  0x0002U  /*!< Pin 1 selected */
#define PORT_Pin_2                  0x0004U  /*!< Pin 2 selected */
#define PORT_Pin_3                  0x0008U  /*!< Pin 3 selected */
#define PORT_Pin_4                  0x0010U  /*!< Pin 4 selected */
#define PORT_Pin_5                  0x0020U  /*!< Pin 5 selected */
#define PORT_Pin_6                  0x0040U  /*!< Pin 6 selected */
#define PORT_Pin_7                  0x0080U  /*!< Pin 7 selected */
#define PORT_Pin_8                  0x0100U  /*!< Pin 8 selected */
#define PORT_Pin_9                  0x0200U  /*!< Pin 9 selected */
#define PORT_Pin_10                 0x0400U  /*!< Pin 10 selected */
#define PORT_Pin_11                 0x0800U  /*!< Pin 11 selected */
#define PORT_Pin_12                 0x1000U  /*!< Pin 12 selected */
#define PORT_Pin_13                 0x2000U  /*!< Pin 13 selected */
#define PORT_Pin_14                 0x4000U  /*!< Pin 14 selected */
#define PORT_Pin_15                 0x8000U  /*!< Pin 15 selected */

#define ADDRESS 0x01

uint8_t fbyte;
uint8_t Byte_Numb;
uint8_t ReadData[35];
uint32_t ADRESS;
uint16_t i, i1;;
uint8_t comand = 0;
uint8_t DatS[256];
uint8_t DatO[256];
/*Регистр флагов*/
/*bit №31 - Флаг приёма стартового байта по UART1*/
/*bit №30 - Флаг идентификации посылки для себя по UART1*/
//uint32_t FLAGS = 0; 
FunctionalState StartByte, ADR, F_Comand;

void PortE_Pin6_init( void  ){
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 25); //разрешение тактирования порта E

    MDR_PORTE->OE     |= ((1 << 6)); //направление передачи данных = Выход
    MDR_PORTE->ANALOG |= ((1 << 6)); //режим работы контроллера = Цифровой
    MDR_PORTE->FUNC   |= ((0 << 6*2)); //режим работы вывода порта = Порт
    MDR_PORTE->PULL   |= ((0 << 6) | (0 << (6 + 16))); //запрещение подтяжки к GND и запрещение подтяжки к VCC
    MDR_PORTE->PD     |= ((0 << 6)); //режим работы выхода = Управл. драйвер
    MDR_PORTE->PWR    |= ((1 << 6*2)); //скорость фронта вывода = медленный
}//void PortE_Pin6_init

void Wait(int value)
{
	int v;
	for (v = 1; v <= value; v++)
	{
		__asm
		{
			NOP;
		}
	}
}



void Timer3_Init( void  ){
    MDR_RST_CLK->PER_CLOCK |= 1 << 16;//разрешение тактирования Таймера 3
    MDR_RST_CLK->TIM_CLOCK = ((3 << 16) /* делитель тактовой частоты Таймера 3 */
                            | (1 << 26));  /* разешение тактирования Таймера 3 */
    MDR_TIMER3->PSG = 0x4;

    MDR_TIMER3->IE = (1 << 1);//разрешение прерывания по совпадению 

    NVIC_EnableIRQ(Timer3_IRQn);
}//void Timer_Init


void PortB_Pin4_init( void  ){
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 22); //разрешение тактирования порта B

    MDR_PORTB->OE     |= ((1 << 4)); //направление передачи данных = Выход
    MDR_PORTB->ANALOG |= ((1 << 4)); //режим работы контроллера = Цифровой
    MDR_PORTB->FUNC   |= ((0 << 4*2)); //режим работы вывода порта = Порт
    MDR_PORTB->PULL   |= ((0 << 4) | (0 << (4 + 16))); //запрещение подтяжки к GND и запрещение подтяжки к VCC
    MDR_PORTB->PD     |= ((0 << 4)); //режим работы выхода = Управл. драйвер
    MDR_PORTB->PWR    |= ((3 << 4*2)); //скорость фронта вывода = максимальный
}//void PortB_Pin4_init

void CPU_init_fromPLIS ( void ){
     //Необходимая пауза для работы Flash-памяти программ
     MDR_EEPROM->CMD |= (3 << 3);

     MDR_RST_CLK->HS_CONTROL = 0x01;// вкл. HSE осцилятора (частота кварца 10 МГц)
     while ((MDR_RST_CLK->CLOCK_STATUS & (1 << 2)) == 0x00);// ждем пока HSE выйдет в рабочий режим

     MDR_RST_CLK->CPU_CLOCK = ((2 << 0));//подача частоты на блок PLL
     MDR_RST_CLK->PLL_CONTROL = ((1 << 2) | (7 << 8));//вкл. PLL  | коэф. умножения = 8
     while ((MDR_RST_CLK->CLOCK_STATUS & 0x02) != 0x02);// ждем когда PLL выйдет в раб. режим

     MDR_RST_CLK->CPU_CLOCK = ((2 << 0)//источник для CPU_C1
                             | (1 << 2)//источник для CPU_C2
                             | (0 << 4)//предделитель для CPU_C3
                             | (1 << 8));//источник для HCLK

     MDR_BKP->REG_0E |= (7 << 0); //режим встроенного регулятора напряжения DUcc
     MDR_BKP->REG_0E |= (7 << 3); //выбор доп.стабилизирующей нагрузки
}//void CPU_init
void Uart1_Init921( void  ){
 MDR_RST_CLK->PER_CLOCK |= (1UL << 21); //тактирование порта A

   MDR_PORTA->FUNC   |= ((3 << 7*2) | (3 << 6*2)); //режим работы порта 
   MDR_PORTA->ANALOG |= ((1 << 7)   | (1 << 6)); //цифровой
   MDR_PORTA->PWR    |= ((3 << 7*2) | (3 << 6*2)); //максимально быcтрый

   MDR_RST_CLK->PER_CLOCK |= (1UL << 6); //тактирование UART1
   MDR_RST_CLK->UART_CLOCK = ((0) /* установка делителя для UART_CLK = HCLK/1 */
                            |(1 << 24));  /* разрешение тактовой частоты UART1 */

   //Параметры делителя при частоте = 80 MHz Гц и скорости = 921 600 бит/с
   MDR_UART1->IBRD = 0x5; //целая часть делителя скорости
   MDR_UART1->FBRD = 0x1B; //дробная часть делителя скорости
   MDR_UART1->LCR_H = ((0 << 1) /* разрешение проверки четности */
                      |(0 << 2) /* четность/нечетность (Нет контроля) */
                      |(0 << 3) /* стоп-бит = 1 стоп-бит */
											|(0 << 4) /* запрет буфера FIFO */
                      |(3 << 5) /* длина слова = 8 бит */
                      |(0 << 7)); /* передача бита четности */
	//	MDR_UART1->IMSC = (1 << 7);
   MDR_UART1->CR = ((1 << 8)|(1 << 9)|1); //передачик и приемник разрешен, разрешение приемопередатчика

		NVIC_EnableIRQ(UART1_IRQn);
	

	UART_ITConfig(MDR_UART1, UART_IT_RT, ENABLE);
	UART_ITConfig(MDR_UART1, UART_IT_TX, DISABLE);
}//void Uart_Init

void PortA_Pin5_init( void  ){
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 21); //разрешение тактирования порта A

    MDR_PORTA->OE     |= ((1 << 5)); //направление передачи данных = Выход
    MDR_PORTA->ANALOG |= ((1 << 5)); //режим работы контроллера = Цифровой
    MDR_PORTA->FUNC   |= ((0 << 5*2)); //режим работы вывода порта = Порт
    MDR_PORTA->PULL   |= ((0 << 5) | (0 << (5 + 16))); //запрещение подтяжки к GND и запрещение подтяжки к VCC
    MDR_PORTA->PD     |= ((0 << 5)); //режим работы выхода = Управл. драйвер
    MDR_PORTA->PWR    |= ((2 << 5*2)); //скорость фронта вывода = быстрый
}//void PortA_Pin5_init

void PortA_Pin4_init( void  ){
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 21); //разрешение тактирования порта A

    MDR_PORTA->OE     |= ((1 << 4)); //направление передачи данных = Выход
    MDR_PORTA->ANALOG |= ((1 << 4)); //режим работы контроллера = Цифровой
    MDR_PORTA->FUNC   |= ((0 << 4*2)); //режим работы вывода порта = Порт
    MDR_PORTA->PULL   |= ((0 << 4) | (0 << (4 + 16))); //запрещение подтяжки к GND и запрещение подтяжки к VCC
    MDR_PORTA->PD     |= ((0 << 4)); //режим работы выхода = Управл. драйвер
    MDR_PORTA->PWR    |= ((1 << 4*2)); //скорость фронта вывода = медленный
}//void PortA_Pin4_init

void SSP1_init ( void  ){
    MDR_RST_CLK->PER_CLOCK |= 1 << 29;//разрешение тактирование порта F

    MDR_PORTF->FUNC   |= (/*(2 << 2*2) |*/ (2 << 1*2) | (2 << 3*2) | (2 << 0*2));//режим работы порта 
    MDR_PORTF->ANALOG |= (/*(1 << 2  ) |*/ (1 << 1  ) | (1 << 3  ) | (1 << 0  ));//цифровой 
    MDR_PORTF->PWR    |= (/*(3 << 2*2) |*/ (3 << 1*2) | (3 << 3*2) | (3 << 0*2));//максимально быcтрый 

    MDR_RST_CLK->PER_CLOCK |= (1 << 8);//тактирование SSP1 

    MDR_RST_CLK->SSP_CLOCK = ((1 << 0)|(1 << 24));//предделитель = /2, разрешение тактирования SSP1

    MDR_SSP1->CPSR = 2; //делитель тактовой частоты = 2, скорость = 5000 kHz
    MDR_SSP1->CR0 = ((0 << 6) //полярность сигнала 
                   | (0 << 7) //фаза сигнала 
                   | (3 << 8) //коэффициент скорости = 4
                   | (0 << 4) //формат кадра = SPI
                   | (7 << 0)); //длина слова = 8 бит 
    MDR_SSP1->CR1 =  ((0 << 2)|(1 << 1)); //режим работы и включение приемопередатчика SSP1
}//void SSP1_init

void Set_ADR_SS(uint8_t addr)
{
	MDR_PORTB->RXTX &= 0xF0;//обнуляем разряды для адреса не меняя остальные
	MDR_PORTB->RXTX = addr | MDR_PORTB->RXTX;
}

void PortB_Pin03_56_init( void  ){
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 22); //разрешение тактирования порта B

    MDR_PORTB->OE     |= 0x6f; //направление передачи данных = Выход
    MDR_PORTB->ANALOG |= 0x6f; //режим работы контроллера = Цифровой
    MDR_PORTB->FUNC   |= 0; //режим работы вывода порта = Порт
    MDR_PORTB->PULL   |= 0xff; //запрещение подтяжки к GND и разрешение подтяжки к VCC
    MDR_PORTB->PD     |= 0; //режим работы выхода = Управл. драйвер
    MDR_PORTB->PWR    |= 0xffff; //скорость фронта вывода = максимальный
}//void PortB_Pin0_init

void PortF_Pin2_init( void  ){
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 29); //разрешение тактирования порта F

    MDR_PORTF->OE     |= ((1 << 2)); //направление передачи данных = Выход
    MDR_PORTF->ANALOG |= ((1 << 2)); //режим работы контроллера = Цифровой
    MDR_PORTF->FUNC   |= ((0 << 2*2)); //режим работы вывода порта = Порт
    MDR_PORTF->PULL   |= ((0 << 2) | (0 << (2 + 16))); //запрещение подтяжки к GND и запрещение подтяжки к VCC
    MDR_PORTF->PD     |= ((0 << 2)); //режим работы выхода = Управл. драйвер
    MDR_PORTF->PWR    |= ((2 << 2*2)); //скорость фронта вывода = быстрый
}//void PortF_Pin2_init

void PortB_Pin12_init( void  ){//прерывание от ПЛИС
    MDR_RST_CLK->PER_CLOCK   |= (1UL << 22); //разрешение тактирования порта B

    MDR_PORTB->OE     |= ((0 << 10)); //направление передачи данных = Вход
    MDR_PORTB->ANALOG |= ((1 << 10)); //режим работы контроллера = Цифровой
    MDR_PORTB->FUNC   |= ((2 << 10*2)); //режим работы вывода порта = Альтернативная
    MDR_PORTB->PULL   |= ((0 << 10) | (0 << (10 + 16))); //запрещение подтяжки к GND и запрещение подтяжки к VCC
    MDR_PORTB->PD     |= ((0 << (10 + 16))); //режим работы входа = выкл. триг. Шмитта
    MDR_PORTB->PWR    |= ((0 << 10*2)); //скорость фронта вывода = 
    MDR_PORTB->GFEN   |= ((0 << 10)); //входной фильтр = выключен
	

}//void PortB_Pin12_init

int main()
{
	__enable_irq();
	CPU_init_fromPLIS();
	PortB_Pin4_init();

	Timer3_Init();
	PortE_Pin6_init();
	PortA_Pin5_init();
	PortA_Pin4_init();
	PortB_Pin03_56_init();
	SSP1_init();
	PortF_Pin2_init();
	PortB_Pin12_init();
	//ВАЖНО!!! 
	//1. Включить разрешение на приём
	//2. Инициализировать UART
	MDR_PORTA->RXTX &= ~PORT_Pin_4;
	Uart1_Init921();
	MDR_PORTF->RXTX = PORT_Pin_2 | MDR_PORTF->RXTX;		
	i1 = 0;
	StartByte = DISABLE;
	ADR = DISABLE;
	UART_ITConfig(MDR_UART1, UART_IT_RX, ENABLE);
	while (1)
	{
	}
}

void EXT_INT2_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(EXT_INT2_IRQn);
	
	Set_ADR_SS(0x00);//выставляем адрес передачи по UART
	MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
	MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
	
	MDR_PORTA->RXTX = PORT_Pin_5 | MDR_PORTA->RXTX;
	UART_SendData(MDR_UART1, 0x0A);
	Wait(1200);	
	MDR_PORTA->RXTX &= ~PORT_Pin_5;
}	

void Timer3_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(Timer3_IRQn);
	MDR_PORTA->RXTX = PORT_Pin_5 | MDR_PORTA->RXTX;
	UART_SendData(MDR_UART1, 0xF1);
	Wait(1200);	
	fbyte = 0;
	Byte_Numb = 0;
	
	StartByte = DISABLE;
	ADR = DISABLE;	
  MDR_TIMER3->CNT = 0x0000;
	MDR_TIMER3->CNTRL = 0x00;
  MDR_TIMER3->STATUS &= ~(1 << 1);
}

void UART1_IRQHandler (void)//ОБМЕН С УУА
{
	if (UART_GetITStatus(MDR_UART1, UART_IT_RX) != RESET)//обработка прерывания на приём
	{
		uint8_t buffer_read/*буфер для приёма данных по UART*/, streg;
		buffer_read = UART_ReceiveData(MDR_UART1);
		MDR_TIMER3->ARR = 0x17;//аргумент счёта таймера	
		if ((buffer_read == 0x55) && (StartByte == DISABLE))//обнуление флага для UART 1	
		{				
			StartByte = ENABLE;//флаг принятия стартового слова				
			comand = 0;
			UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
			MDR_TIMER3->CNT = 0x00;//обнуление буфера счёта счётчика	
			MDR_TIMER3->CNTRL = 0x01;//счет вверх по TIM_CLK, таймер вкл.
			return;
		}
		
		if ((StartByte == ENABLE) && (ADR == DISABLE) && (buffer_read == ADDRESS))//проверка адреса
		{
			MDR_TIMER3->CNT = 0x0000;//сброс буфера счёта счётчика
			ADR =  ENABLE;//установка флага приёма адреса	
			UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);			
			return;
		}
		
		if (ADR == ENABLE) //обработка команд
		{
			MDR_TIMER3->CNT = 0x0000;
			if (comand == 0)
			{
				MDR_TIMER3->CNT = 0x0000;
				comand = buffer_read;//запоминаем команду
				UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
				if ((comand == 0xAA)	| (comand == 0x10) | (comand == 0x22))				
				{
					i = 0;
					return;
				}	
			}
			switch (comand)//выполенение команд
			{
					case 0xAA://команда на приём таблицы ФВ
					{		
						if ((i <= 3) && (i >= 0))//формируем адрес
						{
							ADRESS = F_ADDR_N(buffer_read, i, ADRESS);
							i++;
							MDR_TIMER3->CNT = 0x0000;	
							break;
						}
						if (i > 3)
						{
							DatS[(i - 4)] = buffer_read;				
							i++;
							UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
						}
						if (i == 260)
						{
							//Сброс таймера
							MDR_TIMER3->CNT = 0x0000;
							MDR_TIMER3->CNTRL = 0x00;
							MDR_TIMER3->STATUS &= ~(1 << 1);
					
							Set_ADR_SS(0x05);//выставляем адрес передачи по UART
							MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
							MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
							
							//Стираем адресумый сектор перед записью
							EraseSector(MDR_SSP1, ADRESS, MDR_PORTF, PORT_Pin_2);	
							streg = ReadStatusReg(MDR_SSP1, MDR_PORTF, PORT_Pin_2);	
							while ((streg & 0x01) != 0x00)
							{								
								streg = ReadStatusReg(MDR_SSP1, MDR_PORTF, PORT_Pin_2);	
							}							
				
							//Наичинаем запись в сектор
							ProgrData(MDR_SSP1, ADRESS, DatS, MDR_PORTF, PORT_Pin_2);						
							streg = ReadStatusReg(MDR_SSP1, MDR_PORTF, PORT_Pin_2);	
							while ((streg & 0x01) != 0x00)
							{
								streg = ReadStatusReg(MDR_SSP1, MDR_PORTF, PORT_Pin_2);	
							}							
							
							Set_ADR_SS(0x00);//выставляем адрес передачи по UART
							MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
							MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
							
							UART_SendData(MDR_UART1, streg);
							Wait(1200);								
							MDR_PORTA->RXTX &= ~PORT_Pin_5;
							
							//Сброс флагов
							StartByte = DISABLE;
							ADR = DISABLE;			
							ADRESS = 0;							
							break;
						}
						break;		
					}
					case 0x11:
					{
						MDR_TIMER3->CNT = 0x0000;
						MDR_TIMER3->CNTRL = 0x00;
						MDR_TIMER3->STATUS &= ~(1 << 1);
						
						Set_ADR_SS(0x05);//выставляем адрес передачи по UART
						MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
						MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
						
						ReadStatusReg(MDR_SSP1, MDR_PORTF, PORT_Pin_2);			

						StartByte = DISABLE;
						ADR = DISABLE;							
						break;
					}
					case 0x10://чтение буфера 256 по индексу
					{
						MDR_TIMER3->CNT = 0x0000;
						MDR_TIMER3->CNTRL = 0x00;
						MDR_TIMER3->STATUS &= ~(1 << 1);
						
						MDR_PORTA->RXTX = PORT_Pin_5 | MDR_PORTA->RXTX;
						UART_SendData(MDR_UART1, DatS[buffer_read]);
						Wait(1200);								
						MDR_PORTA->RXTX &= ~PORT_Pin_5;												
						
						StartByte = DISABLE;
						ADR = DISABLE;	
					}
					case 0x22://чтение 256 байт по адресу
					{
						MDR_TIMER3->CNT = 0x0000;

						
						Set_ADR_SS(0x03);//выставляем адрес передачи по UART
						MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
						MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
						
						if ((i <= 3) && (i >= 0))//формируем адрес
						{
							ADRESS = F_ADDR_N(buffer_read, i, ADRESS);
							i++;
							MDR_TIMER3->CNT = 0x0000;	
							if (i == 4)
							{
								MDR_TIMER3->CNT = 0x0000;	
								MDR_TIMER3->CNTRL = 0x00;
								MDR_TIMER3->STATUS &= ~(1 << 1);				
								
								ReadDataF(MDR_SSP1, ADRESS, DatO, MDR_PORTF, PORT_Pin_2);
								
								Set_ADR_SS(0x00);//выставляем адрес передачи по UART
								MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
								MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
										
								StartByte = DISABLE;
								ADR = DISABLE;
								ADRESS = 0;		
							/*	UART_ITConfig(MDR_UART1, UART_IT_TX, ENABLE);
								i1 = 0;
								MDR_PORTA->RXTX = PORT_Pin_5 | MDR_PORTA->RXTX;
								UART_SendData(MDR_UART1, DatO[i1]);
								i1++;*/
								break;
							}
							else
							{
								break;
							}							
						}
					}
					case 0x33://отправка данных из ОЗУ ПЛИС на ППМ
					{
						MDR_TIMER3->CNT = 0x0000;
						MDR_TIMER3->CNTRL = 0x00;
						MDR_TIMER3->STATUS &= ~(1 << 1);
						
						Set_ADR_SS(0x02);//выставляем адрес передачи по UART
						MDR_PORTB->RXTX = PORT_Pin_4 | MDR_PORTB->RXTX;//защёлкиваем адрес фронтом
						MDR_PORTB->RXTX &= ~PORT_Pin_4;//сброс защёлки	
						
						MDR_PORTF->RXTX &= ~PORT_Pin_2;
						
						RW_SPI(MDR_SSP1, 0x07);
						
						Wait(450);
						
						RW_SPI(MDR_SSP1, 0x09);
						
						Wait(450);
						
						RW_SPI(MDR_SSP1, 0xf0);
						
						MDR_PORTF->RXTX = MDR_PORTF->RXTX | PORT_Pin_2;
						
						StartByte = DISABLE;
						ADR = DISABLE;	
						break;
					}
			}
		}
	}
	if (UART_GetITStatus(MDR_UART1, UART_IT_TX) != RESET)//обработка прерывания на передачу
	{
		switch (comand)//выполенение команд
		{
			case 0x22://чтение 256 байт по адресу
			{
				UART_SendData(MDR_UART1, DatO[i1]);
				UART_ClearITPendingBit(MDR_UART1, UART_IT_TX);
				i1++;
				if (i1 == 256)
				{
					i1 = 0;
					Wait(1200);	
					MDR_PORTA->RXTX &= ~PORT_Pin_5;
				
					UART_ITConfig(MDR_UART1, UART_IT_TX, DISABLE);
					
					StartByte = DISABLE;
					ADR = DISABLE;	
				}
				break;
			}
		}
	}
}
