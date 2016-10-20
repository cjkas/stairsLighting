/*
 * Main.c
 *
 *  Created on: 20-03-2012
 *      Author: slaw
 *      8mhz mega8
 *
 *      PORTB4 i PORTD5 nie swieci
 */
//#define DEBUG 1
#if defined DEBUG
#define UART_BAUD_RATE      96000
#endif
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#if defined DEBUG
#include <string.h>
#include "uart.c"
#endif
#define TSOP_PORT DDRD
#define TSOP_TOP_PIN PORTD2
#define TSOP_BTM_PIN PORTD3
#define TSOP_TOP_SET (PIND & (1<<TSOP_TOP_PIN))
#define TSOP_BTM_SET (PIND & (1<<TSOP_BTM_PIN))
#define TST_BTN_PORT PORTC
#define TST_BTN_PIN PORTC3
#define testBtn (PINC & (1<<TST_BTN_PIN))
#define IR1 PORTC5
#define IR2 PORTD0
#define IR1_PORT PORTC
#define IR2_PORT PORTD
#define PHOTO_PORT DDRC
#define PHOTO_PIN PORTC2


char testMode = 0;

void _uart_puts(const char* c){
#if defined DEBUG
	uart_puts(c);
#endif
}
void initTimer(void) {
	//nosna dla ir
	TIMSK |= (1 << OCIE1A); //inicjalizacja przerwania od CTC1A
	OCR1A = 111; // f=36kHz // 8000000/(2+(2*36000))
	TCCR1B |= (1 << WGM12) | (1 << CS10); // CTC mode i preskaler 1

	sei();
}
void initAdc() {

	ADMUX |= _BV(REFS0); //Wybranie wewnętrznego żródła napięcia odniesienia

	//ADMUX|=	_BV(ADLAR);		//Wybranie sposobu zapisu wyniku z wyrównaniem do lewej (osiem starszych bitów wyniku w rejestrze ADCH)
	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADPS1);
	ADCSRA |= _BV(ADPS2); 	//125khz
}
int getADC(uint8_t ch) {
	//Select ADC Channel ch must be 0-7
	ch = ch & 0b00000111;
	ADMUX |= ch;

	ADCSRA |= _BV(ADSC); 	//start pojedynczej konwersji
	while (bit_is_set(ADCSRA, ADSC)) {
	}; 	//czekanie az konwersja zakonczona, wtedy ADSC=0;
	ADCSRA |= _BV(ADIF); 	//Clear ADIF by writing one to it

	return (ADC);
}

void delay_between() {
	if (testMode) {
		_delay_ms(1);
	} else {
		_delay_ms(150);
	}
}
void delay_end() {
	if (testMode) {
		_delay_ms(5);
	} else {
		_delay_ms(5000);
		_delay_ms(5000);
		_delay_ms(5000);
		_delay_ms(5000);
	}
}
void topLightDisable(void) {
	_uart_puts("top disable\r\n");
	PORTB &= ~_BV(PORTB2);
	delay_between();
	PORTB &= ~_BV(PORTB1);
	delay_between();
	PORTD &= ~_BV(PORTD6);
	delay_between();
	PORTB &= ~_BV(PORTB6);
	delay_between();
	PORTB &= ~_BV(PORTB3);
	delay_between();
	PORTB &= ~_BV(PORTB0);
	delay_between();
	PORTD &= ~_BV(PORTD7);
	delay_between();
	PORTB &= ~_BV(PORTB7);
	delay_between();
	PORTD &= ~_BV(PORTD5);
	delay_between();
	PORTB &= ~_BV(PORTB4);

}
void btmLightDisable(void) {
	_uart_puts("btm disable\r\n");
	PORTB &= ~_BV(PORTB7);
	delay_between();
	PORTD &= ~_BV(PORTD7);
	delay_between();
	PORTB &= ~_BV(PORTB0);
	delay_between();
	PORTB &= ~_BV(PORTB3);
	delay_between();
	PORTB &= ~_BV(PORTB6);
	delay_between();
	PORTD &= ~_BV(PORTD6);
	delay_between();
	PORTB &= ~_BV(PORTB1);
	delay_between();
	PORTB &= ~_BV(PORTB2);
	delay_between();
	PORTD &= ~_BV(PORTD5);
	delay_between();
	PORTB &= ~_BV(PORTB4);
}
void topLightEnable(void) {
	_uart_puts("top enable\r\n");
	PORTB |= _BV(PORTB7); //1
	delay_between();
	PORTD |= _BV(PORTD7); //2
	delay_between();
	PORTB |= _BV(PORTB0); //3
	delay_between();
	PORTB |= _BV(PORTB3); //4
	delay_between();
	PORTB |= _BV(PORTB6); //5
	delay_between();
	PORTD |= _BV(PORTD6); //6 podest
	delay_between();
	PORTB |= _BV(PORTB1); //8
	delay_between();
	PORTB |= _BV(PORTB2); //9
	delay_between();
	PORTD |= _BV(PORTD5); //
	delay_between();
	PORTB |= _BV(PORTB4); //
	delay_between();
	delay_end();
	topLightDisable();
}
void btmLightEnable(void) {
	_uart_puts("btm enable\r\n");
	PORTB |= _BV(PORTB2);
	delay_between();
	PORTB |= _BV(PORTB1); //??
	delay_between();
	PORTD |= _BV(PORTD6); //9
	delay_between();
	PORTB |= _BV(PORTB6); //34
	delay_between();
	PORTB |= _BV(PORTB3); //6
	delay_between();
	PORTB |= _BV(PORTB0);
	delay_between();
	PORTD |= _BV(PORTD7);
	delay_between();
	PORTB |= _BV(PORTB7); //??
	delay_between();
	PORTD |= _BV(PORTD5); //??
	delay_between();
	PORTB |= _BV(PORTB4); //??
	delay_end();
	btmLightDisable();
}
int main(void) {
	initTimer();
	initAdc();
	DDRB |= _BV(PORTB0)|_BV(PORTB1)|_BV(PORTB2)|_BV(PORTB3)|_BV(PORTB4)|_BV(PORTB6)|_BV(PORTB7); //lampki
	DDRD |= _BV(PORTD5) | _BV(PORTD6) | _BV(PORTD7);	//lampki
	TSOP_PORT &= ~_BV(TSOP_TOP_PIN);	//TSOP T
	TSOP_PORT &= ~_BV(TSOP_BTM_PIN);	//TSOP D
	IR1_PORT &= ~_BV(IR1);	//IR
	IR2_PORT &= ~_BV(IR2);	//IR
	PHOTO_PORT &= ~_BV(PHOTO_PIN);	//LTR PHOTO
	TST_BTN_PORT &= _BV(TST_BTN_PIN);	//TST BTN

	IR1_PORT |= _BV(IR1);	//IR
	IR2_PORT |= _BV(IR2);	//IR
	TST_BTN_PORT |= _BV(TST_BTN_PORT);	//TST BTN
	PORTB &= ~(_BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2) | _BV(PORTB3)| _BV(PORTB4) | _BV(PORTB6)|_BV(PORTB7));	//lampki
	PORTD &= ~(_BV(PORTD5) | _BV(PORTD6) | _BV(PORTD7));	//lampki
	/*ADC*/
	int brightness = 0;
	int darkLimit = 1023;
#if defined DEBUG
	char buffer[25];

	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
#endif
	topLightDisable();

	while (1) {

		if (testBtn && testMode == 0) {
			testMode = 1;
			darkLimit = 0;
		} else if (!testBtn && testMode == 1) {
			testMode = 0;
			darkLimit = 1023;
		}

		brightness = getADC(PHOTO_PIN); //zaladuj do zmiennej pomiar wartosc z PC2
#if defined DEBUG
		itoa(brightness, buffer, 10);
		_uart_puts(buffer);
		_uart_puts("\r\n");
#endif

		if (brightness >= darkLimit) {
			_uart_puts("it's dark\r\n");
			if (TSOP_TOP_SET) {
				_uart_puts("top ir\r\n");
				topLightEnable();
			}
			if (TSOP_BTM_SET) {
				_uart_puts("btm ir\r\n");
				btmLightEnable();
			}
		} else {
			_uart_puts("too bright\r\n");
		}
		if(testMode){
			_uart_puts("test mode\r\n");

			if(TSOP_TOP_SET){
				topLightEnable();
			}
			if(TSOP_BTM_SET){
				btmLightEnable();
			}
		}
		
		//oszczedzamy prad i sparwdzamy ~10 razy na sekunde
		IR1_PORT&=~_BV(IR1);//wylacz ir
		IR2_PORT&=~_BV(IR2);//wylacz ir
		TCCR1B &= 0xF8; //zamieniasz 3 ostatnie bity na 0 czyli wyłączasz preskaler /nosna off
		_delay_ms(50);//poczekaj chwile na inicjalizacje
		TCCR1B |= (1<<WGM12) | (1<<CS10); // wlacz nosna ponownie
		_delay_ms(25);//poczekaj chwile na inicjalizacje

	};
	return (0);
}
ISR(TIMER1_COMPA_vect) {

	//nosna 36khz na ir
	IR1_PORT ^= _BV(IR1); //odwraca wartość
	IR2_PORT ^= _BV(IR2);
}
