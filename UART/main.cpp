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

#define DURATION		(10 * 1000)

#define AVCC_MODE		(1 << REFS0)
#define UART_TIMEOUT	20							//2ms

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

#define DHT11_PIN				(PB4)

#define LCD_Cmd_Port PORTD
#define LCD_Cmd_Dir DDRD
#define LCD_Dir  DDRC			/* Define LCD data port direction */
#define LCD_Port PORTC			/* Define LCD data port */
#define RS PD6				/* Define Register Select pin */
#define RW PD5				/* Define Read/Write signal pin */
#define EN PD7 				/* Define Enable signal pin */

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
volatile uint32_t lcd_tick = 0;

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

// LCD
uint8_t fst = 1;
char lcd_l1[16] = "";
char lcd_l2[16] = "";
char name[] = "Nguyen Minh Tuan, 20203631, DT05-K65";
char sch[] = "Truong Dien - Dien tu, DHBK HN";
uint8_t mode = 0;

uint8_t led_state = 0;

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

void LCD_Command( unsigned char cmnd );
void LCD_Char( unsigned char data );
void LCD_Init (void);
void LCD_String (char *str);
void LCD_String_xy (char row, char pos, char *str);
void LCD_Clear();
void LCD_Display(uint8_t mode);
void LCD_Hanlde();

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
	_delay_ms(1000);
	LCD_Init();
	LCD_String((char*)"Starting...");
	while (1) 
    {
		HandleCommand();
		if (g_sys_tick - last_tick > DURATION) { 
			last_tick = g_sys_tick;
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
			//PORTD = ~PORTD;
		}
		LCD_Hanlde();
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
	if (g_sys_tick - uart_last_rcv > UART_TIMEOUT && uart_cnt >= 2) {
		/*
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
			memset(rx, 0, strlen((char*) rx));
			uart_cnt = 0;
		}
		*/
		if (strncmp((char*) rx, "time:", 5) == 0 && strlen((char*) rx) == 19) {				//time:YYYYMMddhhmmss
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
		//sprintf((char*) rx, "Err1\n");
		//UARTTransmit(rx);
		return;
	}
	_delay_us(80);
	
	if(!(PINB & (1 << DHT11_PIN))) {
		//sprintf((char*) rx, "Err2\n");
		//UARTTransmit(rx);
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


void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Cmd_Port &= ~ (1<<RS);		/* RS=0, command reg. */
	LCD_Cmd_Port |= (1<<EN);		/* Enable pulse */
	_delay_us(1);
	LCD_Cmd_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	LCD_Cmd_Port |= (1<<EN);
	_delay_us(1);
	LCD_Cmd_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Cmd_Port |= (1<<RS);		/* RS=1, data reg. */
	LCD_Cmd_Port |= (1<<EN);
	_delay_us(1);
	LCD_Cmd_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	LCD_Cmd_Port |= (1<<EN);
	_delay_us(1);
	LCD_Cmd_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Dir = 0xFF;			/* Make LCD port direction as o/p */
	LCD_Cmd_Dir = 0xFF;
	LCD_Cmd_Port &= ~(1 << RW);
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x02);		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0c);              /* Display on cursor off*/
	LCD_Command(0x06);              /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              /* Clear display screen*/
	_delay_ms(2);
}

void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
	LCD_String(str);		/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);		/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);		/* Cursor at home position */
}

void LCD_Display(uint8_t mode) {
	if (mode == 0) {
		LCD_Clear();
		sprintf(lcd_l1, "Temperature: %d", dht11.tem);
		sprintf(lcd_l2, "Humidity: %d", dht11.hum);
		LCD_String_xy(0, 0, lcd_l1);
		LCD_String_xy(1, 0, lcd_l2);
	}
	else if (mode == 1) {
		LCD_Clear();
		sprintf(lcd_l1, "Acetone: %d ppm", aqi.Acetone);
		sprintf(lcd_l2, "Alcohol: %d ppm", aqi.Alcohol);
		LCD_String_xy(0, 0, lcd_l1);
		LCD_String_xy(1, 0, lcd_l2);
	}
	else if (mode == 2) {
		LCD_Clear();
		sprintf(lcd_l1, "CO: %d ppm", aqi.CO);
		sprintf(lcd_l2, "CO2: %d ppm", aqi.CO2);
		LCD_String_xy(0, 0, lcd_l1);
		LCD_String_xy(1, 0, lcd_l2);
	}
	else if (mode == 3) {
		LCD_Clear();
		sprintf(lcd_l1, "NH4: %d ppm", aqi.NH4);
		sprintf(lcd_l2, "Toluene: %d ppm", aqi.Toluene);
		LCD_String_xy(0, 0, lcd_l1);
		LCD_String_xy(1, 0, lcd_l2);
	}
	else if (mode == 4) {
		LCD_Clear();
		sprintf(lcd_l1, "%02d : %02d : %04d", ds1307.date, ds1307.month, ds1307.year);
		sprintf(lcd_l2, "%02d : %02d : %02d", ds1307.hour, ds1307.min, ds1307.sec);
		LCD_String_xy(0, 0, lcd_l1);
		LCD_String_xy(1, 0, lcd_l2);
	}
}

void LCD_Hanlde() {
	if (fst && g_sys_tick > 11000) {
		fst = 0;
		lcd_tick = g_sys_tick;
	}
	if (!fst && g_sys_tick - lcd_tick >= 1700) {
		LCD_Display(mode);
		mode++;
		if (mode == 5) 
			mode = 0;
		lcd_tick = g_sys_tick;
	}
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