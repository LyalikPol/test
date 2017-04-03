#include "MDR32Fx.h"
#include "S25FK216K.h"


/*Функция отправки и приёма байта по SPI*/
uint8_t RW_SPI(/*Идентефикатор SPI*/MDR_SSP_TypeDef* SSPx, /*Данные для отправки*/uint8_t VALUE)
{
	uint8_t DATA;
	while (SSPx->SR & (uint16_t)0x0010) {}//ждём освобождения FIFO передатчика	
	SSPx->DR = VALUE;//передача
		
	while ((SSPx->SR & (uint16_t)0x0004) != (uint16_t)0x0004){}//ждём появления данных в FIFO приёмника
	DATA = SSPx->DR;//приём
	return DATA;		
}
/*Установка бита разрешения записи*/
void WriteEnable(/*Идентефикатор SPI*/MDR_SSP_TypeDef* SSPx, /*Порт CS*/MDR_PORT_TypeDef* PORTx, /*Пин CS*/uint16_t PINx)
{
	PORTx->RXTX &= ~PINx;//Устанвка сигнала CS в "0"
	
	RW_SPI(SSPx, 0x06);//Передача команды
	
	PORTx->RXTX = PORTx->RXTX | PINx;//Установка сигнала CS в "1"
}
/*Чтение статус регистра*/
uint16_t ReadStatusReg(/*Идентефикатор SPI*/MDR_SSP_TypeDef* SSPx, /*Порт CS*/MDR_PORT_TypeDef* PORTx, /*Пин CS*/uint16_t PINx)
{
	uint16_t DATA = 0x0000;
	
	PORTx->RXTX &= ~PINx;//Устанвка сигнала CS в "0"
	
	RW_SPI(SSPx, 0x05);//Передача команжы
	DATA |= (RW_SPI(SSPx, 0xFF) << 8);//Чтение 1-й части регистра статуса
	DATA |= RW_SPI(SSPx, 0xFF);//Чтение 2-й части регистра статуса 
	
	PORTx->RXTX = PORTx->RXTX | PINx;//Установка сигнала CS в "1"
	
	return DATA;
}
/*Очистка сектора*/
void EraseSector(/*Идентефикатор SPI*/MDR_SSP_TypeDef* SSPx, uint32_t ADRES, /*Порт CS*/MDR_PORT_TypeDef* PORTx, /*Пин CS*/uint16_t PINx)
{
	uint8_t BUF = 0x00;
	PORTx->RXTX &= ~PINx;//Устанвка сигнала CS в "0"
	
	RW_SPI(SSPx, 0x20);//Передача команжы	
//	BUF = (ADR >> 24);
	//RW_SPI(SSPx, (ADRES >> 24));//Передача адреса
	RW_SPI(SSPx, (ADRES >> 16));//Передача адреса
	RW_SPI(SSPx, (ADRES >> 8));//Передача адреса
	RW_SPI(SSPx, (ADRES >> 0));//Передача адреса
	//RW_SPI(SSPx, ADRES );//Передача команжы
		PORTx->RXTX = PORTx->RXTX | PINx;//Установка сигнала CS в "1"
}
