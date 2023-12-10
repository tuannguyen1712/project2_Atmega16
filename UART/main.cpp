/*
 * UART.cpp
 *
 * Created: 12/3/2023 7:37:33 AM
 * Author : tuann
 */ 
/*******************************
led:xxxxxxxx			// set PORTD = xxxxxxxx(bin);
led:1					// turn off led 
*******************************/
#define FRE				8
#define F_CPU			8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "avr/interrupt.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "MQ135.h"

#define cbi(PORTx, Pxi)			(PORTx) &= ~(1 << (Pxi))
#define sbi(PORTx, Pxi)			(PORTx) |= (1 << (Pxi))	

#define AVCC_MODE		(1 << REFS0)
#define UART_TIMEOUT	2							//2ms

#define TWI_R			1 
#define TWI_W			0
#define TWI_START		(1 << TWINT) | (1 << TWEN) | (1 << TWSTA)
#define TWI_STOP		(1 << TWINT) | (1 << TWSTO) | (1 << TWEN)
#define DS1307_I2C_ADDR    	 	0x68
#define DS1307_REG_SECOND    	0x00
#define DS1307_REG_MINUTE    	0x01
#define DS1307_REG_HOUR      	0x02
#define DS1307_REG_DOW       	0x03
#define DS1307_REG_DATE      	0x04
#define DS1307_REG_MONTH     	0x05
#define DS1307_REG_YEAR      	0x06
#define DS1307_REG_CONTROL   	0x07
#define DS1307_REG_UTC_HR    	0x08
#define DS1307_REG_UTC_MIN   	0x09
#define DS1307_REG_CENT      	0x10
#define DS1307_TIMEOUT       	1000

#define START_SENT				0x08
#define RE_SENT					0x10
#define SLA_ADD_W				0x18
#define DATA_SENT				0x28

#define SLA_ADD_R				0x40
#define DATA_RECV_ACK			0x50
#define DATA_RECV_NACK			0x58

#define DHT11_PIN				(PB0)

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dow;
	uint8_t date;
	uint8_t month;
	uint16_t year;
} DS1307_param_t;

typedef struct {
	uint8_t hum;
	uint8_t tem;
} DHT11_param_t;

//timer0
volatile uint32_t sys_tick = 0;								//1us
volatile uint32_t g_sys_tick = 0;							//1ms
volatile uint32_t last_tick = 0;
volatile uint32_t time_tick = 0;

//timer2
volatile uint32_t tim2_tick = 0;
volatile uint32_t last_tim2_tick = 0;

//UART
uint8_t test[] = "Hello\n";
uint8_t rx[100];
uint8_t tx[100];
uint8_t uart_cnt = 0;
uint32_t uart_last_rcv = 0;

//ADC
uint8_t channel = 7;										// PA7
uint16_t ADC_val;
uint8_t ADC_tx[100];
Air_param_t aqi;

uint8_t irh, drh, it, dt, checksum;

DHT11_param_t dht11;
DS1307_param_t ds1307;

void TIM0_Init();

void UART_Init();
void UARTTransmitByte(uint8_t tx_byte);
void UARTTransmit(uint8_t *tx);
void HandleCommand();

void ADC_Init();
uint16_t readADC();							// Single Ended Input

void TWI_Init();
void TWI_Master_Transmit(uint8_t add, uint8_t* data, uint8_t len);
void TWI_Master_Receive(uint8_t add, uint8_t* data, uint8_t len);

uint8_t DS1307_DecodeBCD(uint8_t bin);
uint8_t DS1307_EncodeBCD(uint8_t dec);
void DS1307_SetClockHalt(uint8_t halt);
void DS1307_SetRegByte(uint8_t regAddr, uint8_t val);
void DS1307_SetTimeZone(uint8_t hr, uint8_t min);
uint8_t DS1307_GetClockHalt();
uint8_t DS1307_GetRegByte(uint8_t regAddr);
void DS1307_config();
void DS1307_Gettime(DS1307_param_t *ds1307);
void DS1307_Settime(uint8_t sec, uint8_t min, uint8_t hour_24mode, uint8_t dayOfWeek, uint8_t date, uint8_t month, uint16_t year);

void DHT11_Init();									
void DHT11_GetValue(DHT11_param_t* dht11);

int main(void)
{
    /* Replace with your application code */
	UART_Init();
	TIM0_Init();
	ADC_Init();
	DHT11_Init();
	TWI_Init();
	DS1307_SetClockHalt(0);
	DS1307_Settime(0, 18, 22, 5, 12, 7, 2023);
	DDRD = 0xFC;
	PORTD = 0x00;
    while (1) 
    {
		HandleCommand();
		if (g_sys_tick - last_tick > 5000) { 
			last_tick = g_sys_tick;
			PORTD = PORTD ^ 0xFC;
			ADC_val = readADC();
			DHT11_GetValue(&dht11);
			DS1307_Gettime(&ds1307);
			getAQI_Correctval(&aqi, dht11.tem, dht11.hum, ADC_val);
//			sprintf((char*) ADC_tx, "ADC value: %d\nAcetone: %d Alcohol: %d CO: %d CO2: %d NH4: %d Toluene: %d\n", 
//			(int) ADC_val, (int) aqi.Acetone, (int) aqi.Alcohol, (int) aqi.CO, (int) aqi.CO2, (int) aqi.NH4, (int) (12.5/6));
			sprintf((char*) ADC_tx, "%04d%02d%02d%02d%02d%02d\tAcetone:%d\tAlcohol:%d\tCO:%d\tCO2:%d\tAmmoniac:%d\tToluene:%d\tTem:%d\tHum:%d", 
			ds1307.year, ds1307.month, ds1307.date, ds1307.hour, ds1307.min, ds1307.sec,
			(int) aqi.Acetone, (int) aqi.Alcohol, (int) aqi.CO, (int) aqi.CO2, (int) aqi.NH4, (int) aqi.Toluene,
			(int) dht11.tem, (int) dht11.hum);
			UARTTransmit(ADC_tx);
			memset(ADC_tx, 0, sizeof(ADC_tx));
		}

//		PORTD = PORTD ^ 0xFC;
//		_delay_ms(1000);
    }
}

void TIM0_Init() {
	TCCR0 = (1 << CS01) | (1 << CS00);				// prescaler = 64 
	TCNT0 = 131;									// 64 / 8MHz * (256 - 131) = 1ms
	TIMSK = (1 << TOIE0);							// enable Timer interrupt
	sei();
}

void UART_Init() {
	// Set baud rate 9600 with FRE = 8MHz
	UBRRH = 0;
	UBRRL = 51;
	
	UCSRA = 0x00;
	
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	
	sei();
}

void UARTTransmitByte(uint8_t tx_byte) {
	while(!(UCSRA & (1 << UDRE)));
	UDR = tx_byte;
}

void UARTTransmit(uint8_t *tx) {
	for(int i = 0; i < (int) strlen((char*) tx); i++) {
		UARTTransmitByte(tx[i]);
	}	
}

void HandleCommand() {
	if (g_sys_tick - uart_last_rcv > 1 && uart_cnt >= 2) {
		if (strncmp((char*) rx, "led:", 4) == 0) {
			if (strlen((char*)rx) == 12) {
				PORTD = (uint8_t) strtoul((char*) (rx + 4), NULL, 2);
				sprintf((char*) tx, "Set PORTD = %s\n", rx + 4);
				UARTTransmit(tx);
			}
			else if (strcmp((char*) rx, "led:1") == 0) {
				PORTD = 0xFF;
				sprintf((char*) tx, "Turn off led\n");
				UARTTransmit(tx);
			}
			// reset rx buffer for the next command
			memset(rx, 0, strlen((char*) rx));
			uart_cnt = 0;
		}
		else if (strncmp((char*) rx, "time:", 5) == 0 && strlen((char*) rx) == 19) {				//time:YYYYMMddhhmmss
			int y, M, d, h, m, s;
			sscanf((char*) (rx + 5), "%4d%2d%2d%2d%2d%2d", &y, &M, &d, &h, &m, &s);
			DS1307_Settime(s, m, h, 1, d, M, y);
			memset(rx, 0, strlen((char*) rx));
			uart_cnt = 0;
		}
		else {
			sprintf((char*) tx, "Invalid Command\n");
			UARTTransmit(tx);
			memset(rx, 0, strlen((char*) rx));
			uart_cnt = 0;
		}
	}
}

void ADC_Init() {
	ADMUX |= AVCC_MODE;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);		// Enable ADC and choose ADC prescaler = 128 and Left Adjust Result
} 

uint16_t readADC() {
	ADMUX = channel | AVCC_MODE;
	ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADIF)));
//	uint16_t value = (uint16_t) (ADCH << 8) + (uint16_t) (ADCL);
	uint16_t value = ADCW;
	return value;
}

void TWI_Init() {
	TWSR = 0x00;
	TWBR = 0x32;					// SCL fre = 100MHz with clock = 8MHz
	TWCR = (1 << TWEN);
}

void TWI_Master_Transmit(uint8_t add, uint8_t* data, uint8_t len) {
	TWCR = TWI_START;
//	while (!(TWCR & (1 << TWINT)));
	while ((TWCR & (1 << TWINT)) == 0);
	
 	TWDR = (add << 1) | TWI_W;
	TWCR = (1 << TWINT) | (1 << TWEN);				// clear bit TWINT
	while ((TWCR & (1 << TWINT)) == 0);
	
	for (int i = 0; i < len; i++) {
		TWDR = data[i];
		TWCR = (1 << TWINT) | (1 << TWEN);				// clear bit TWINT
		while ((TWCR & (1 << TWINT)) == 0);
	}
	
	TWCR = TWI_STOP;
	TWI_Init();
}

void TWI_Master_Receive(uint8_t add, uint8_t word_add, uint8_t* data, uint8_t len) {
	TWCR = TWI_START;
	while ((TWCR & (1 << TWINT)) == 0);
	
	TWDR = (add << 1) | TWI_W;
	TWCR = (1 << TWINT) | (1 << TWEN);				// clear bit TWINT
	while ((TWCR & (1 << TWINT)) == 0);
	
	TWDR = word_add;
	TWCR = (1 << TWINT) | (1 << TWEN);				// clear bit TWINT
	while ((TWCR & (1 << TWINT)) == 0);
	
	TWCR = TWI_START;
	while ((TWCR & (1 << TWINT)) == 0);
	
	TWDR = (add << 1) | TWI_R;
	TWCR = (1 << TWINT) | (1 << TWEN);				// clear bit TWINT
	while ((TWCR & (1 << TWINT)) == 0);
	
	//first len - 1 byte, response ACK
	for (int i = 1; i < len - 1; i++) {
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);				//clear TWINT, send ACK when receive msg from slave and enable TW
		while ((TWCR & (1 << TWINT)) == 0);
		data[i] = TWDR;
	}
	// last byte response NACK
	TWCR = (1 << TWINT) | (1 << TWEN);		
	while (!(TWCR & (1 << TWINT)));
	data[len - 1] = TWDR;
	
	TWCR = TWI_STOP;
	TWI_Init();
}

uint8_t DS1307_DecodeBCD(uint8_t bin) {				// bcd to dec
	return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}
uint8_t DS1307_EncodeBCD(uint8_t dec) {				// dec to bcd
	return (dec % 10 + ((dec / 10) << 4));
}

uint8_t DS1307_GetRegByte(uint8_t regAddr) {
	uint8_t val;
	TWI_Master_Receive(DS1307_I2C_ADDR, regAddr, &val, 1);
	return val;
}

void DS1307_SetRegByte(uint8_t regAddr, uint8_t val) {
	uint8_t bytes[2] = {regAddr, val};
	TWI_Master_Transmit(DS1307_I2C_ADDR, bytes, 2);
}

void DS1307_Gettime(DS1307_param_t *ds1307) {
	uint16_t cen;
	ds1307->sec = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_SECOND) & 0x7f);
	ds1307->min = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_MINUTE));
	ds1307->hour = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_HOUR) & 0x3f);
	ds1307->dow = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_DOW));
	ds1307->date = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_DATE));
	ds1307->month = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_MONTH));
	cen = DS1307_GetRegByte(DS1307_REG_CENT) * 100;
	ds1307->year = DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_YEAR)) + cen;
}

void DS1307_Settime(uint8_t sec, uint8_t min, uint8_t hour_24mode, uint8_t dayOfWeek, uint8_t date, uint8_t month, uint16_t year) {
	DS1307_SetRegByte(DS1307_REG_SECOND,
	DS1307_EncodeBCD(sec | DS1307_GetClockHalt()));
	DS1307_SetRegByte(DS1307_REG_MINUTE, DS1307_EncodeBCD(min));
	DS1307_SetRegByte(DS1307_REG_HOUR, DS1307_EncodeBCD(hour_24mode & 0x3f)); //hour_24mode Hour in 24h format, 0 to 23.
	DS1307_SetRegByte(DS1307_REG_DOW, DS1307_EncodeBCD(dayOfWeek)); //dayOfWeek Days since last Sunday, 0 to 6.
	DS1307_SetRegByte(DS1307_REG_DATE, DS1307_EncodeBCD(date)); //date Day of month, 1 to 31.
	DS1307_SetRegByte(DS1307_REG_MONTH, DS1307_EncodeBCD(month)); //month Month, 1 to 12.
	DS1307_SetRegByte(DS1307_REG_CENT, year / 100);
	DS1307_SetRegByte(DS1307_REG_YEAR, DS1307_EncodeBCD(year % 100)); //2000 to 2099.
}

uint8_t DS1307_GetClockHalt() {
	return (DS1307_GetRegByte(DS1307_REG_SECOND) & 0x80) >> 7;
}

void DS1307_SetClockHalt(uint8_t halt) {
	uint8_t ch = (halt ? 1 << 7 : 0);
	DS1307_SetRegByte(DS1307_REG_SECOND, ch | (DS1307_GetRegByte(DS1307_REG_SECOND) & 0x7f));
}

void DS1307_SetTimeZone(uint8_t hr, uint8_t min) {
	DS1307_SetRegByte(DS1307_REG_UTC_HR, hr);
	DS1307_SetRegByte(DS1307_REG_UTC_MIN, min);
}

void DS1307_config() {
	DS1307_SetClockHalt(0);
//	DS1307_SetTimeZone(+7, 00);
}

void DHT11_Init() {
	DDRB |= (1 << DHT11_PIN);						// DHT11 pin is PA4
	PORTB |= (1 << DHT11_PIN);
}

void DHT11_GetValue(DHT11_param_t* dht11) {
	uint8_t byte[5];
		
	PORTB &= ~(1 << DHT11_PIN);						// mcu start signal
	_delay_ms(18);
	PORTB |= (1 << DHT11_PIN);
	DDRB &= ~(1 << DHT11_PIN);
	_delay_us(30);
	
	if(PINB & (1 << DHT11_PIN)) {
		sprintf((char*) rx, "Err1\n");
		UARTTransmit(rx);
		return;
	}
	_delay_us(80);
	
	if(!(PINB & (1 << DHT11_PIN))) {
		sprintf((char*) rx, "Err2\n");
		UARTTransmit(rx);
		return;
	}
	_delay_us(80);									// wait dht response
	for (int j = 0; j < 5; j++) { //for each byte (5 total)
		uint8_t result = 0;
		for(int i = 0; i < 8; i++) {//for each bit in each byte (8 total)
			while(!(PINB & (1 << DHT11_PIN)));
			_delay_us(30);
			if(PINB & (1 << DHT11_PIN))
				//result |= (1 << (7-i));
				result = (result << 1) | 0x01;
			else 
				result = result << 1;
			while(PINB & (1 << DHT11_PIN));
		}
		byte[j] = result;
	}
	
	DHT11_Init();
	
	dht11->hum = byte[0];
	dht11->tem = byte[2];
}

ISR(USART_RXC_vect) {
	rx[uart_cnt] = UDR;
	uart_cnt++;
	uart_last_rcv = g_sys_tick;
}

ISR(TIMER0_OVF_vect) {
	g_sys_tick++;
	TCNT0 = 131;				
}