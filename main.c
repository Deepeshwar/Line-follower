#define F_CPU 8000000UL
#include <avr/io.h>

#include "USART_128.h"
#include <avr/sfr_defs.h>
int a[7]={0},last_err,cnt1=0;
float left_rpm,right_rpm,pidcalc=0;
float err;

#define ki 0

#define kp 7000.0
#define kd 0

void read_sensor() {
	int i;
	for(i=0;i<7;i++)
		{
		if(bit_is_set(PINC,i))
			{
			a[i]=1;
			
			}
			else
			{
			a[i]=0;
			}
		}
	}

void error() {

	if(((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]))>=4)
	err=4*(float)((-3*a[0])+(-2*a[1])+(-1*a[2])+(1*a[4])+(2*a[5])+(3*a[6]))/((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]));
	
	if(((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]))<4)
	err=(float)((-3*a[0])+(-2*a[1])+(-1*a[2])+(1*a[4])+(2*a[5])+(3*a[6]))/((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]));
	 
	
	//if(((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]))==0)
	//err=last_err;
	

}

void pwm() {
	DDRB=0xFF;
	TCCR1A |= 1<<WGM11 | 1<<COM1A1| 1<<COM1B1;
	TCCR1B |= 1<<WGM12  | 1<<WGM13 | 1<<CS11;
	ICR1=20000;
}

void pid(float err) {
	
	pidcalc=(kp*err+kd*(err-last_err));

	last_err=err;
}


int main(void)
{
	DDRC=0b00000000;
	DDRD|=(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7) | 1<<PD0;
	DDRA=0xFF;
	PORTA|= (1<<PA3) | (1<<PA1) | (0<<PA0) | (0<<PA2);
	TIMSK|=(1<<OCIE0);
	TIFR|=(1<<OCF0);
	
	TCCR0|=(1<<CS01)|(1<<CS00)|(1<<COM01)|(1<<WGM01);
	USART_Init(51,0);
	pwm();
	sei();
	OCR0=25;
 	PORTD=0b01010000;

		while(1)
		{
			//USART_Transmitchar(" ");
			

			//USART_TransmitNumber(OCR1A,0);
			//USART_Transmitchar('B',0);
			//USART_TransmitNumber(OCR1B,0);
			//USART_Transmitchar(" ");*/
		}
}

ISR(TIMER0_COMP_vect) {
	read_sensor();
	error();
	pid(err);
	long left_tx, right_tx;
	if(err>=-0.2&&err<=0.2)
	{
		PORTD|=1<<PD4;
		PORTD&=~(1<<PD5);
		PORTD|=1<<PD6;
		PORTD&=~(1<<PD0);	

		OCR1A=20000;
		OCR1B=20000;
		left_tx = 20000;
		right_tx = 20000;
	}
	else
	{	
			

		//OCR1A=10000;
		left_rpm=10000-pidcalc;
		right_rpm=10000+pidcalc;		
		if(left_rpm>20000) left_rpm=20000;
		if(right_rpm>20000) right_rpm=20000;
		left_tx = left_rpm;
		right_tx = right_rpm;
		if(right_rpm<0)
		{
			PORTD|=(1<<PD5);
			PORTD&=~(1<<PD4);	
			//right_rpm=000;
			right_rpm=-right_rpm;

		}
		else{
			PORTD|=(1<<PD4);
			PORTD&=~(1<<PD5);	
		}
		
		if(left_rpm<0)
		{
			PORTD|=(1<<PD0);
			PORTD&=~(1<<PD6);	
			//right_rpm=15000;
			left_rpm=-left_rpm;
		}
		else{
			PORTD|=(1<<PD6);
			PORTD&=~(1<<PD0);	
		}
						
		if(left_rpm>20000) left_rpm=20000;
		if(right_rpm>20000) right_rpm=20000;
		
		
		
		OCR1A=left_rpm;
		OCR1B=right_rpm;
/*		if(((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]))==0 && err>0)
	{
	PORTD|=1<<PD0;
	PORTD&=~(1<<PD6);	
	PORTD|=1<<PD4;
	PORTD&=~(1<<PD5);	
	OCR1A=15000;
	OCR1B=15000;
						
	}
		if(((a[0])+a[3]+(a[1])+(a[2])+(a[4])+(a[5])+(a[6]))==0 && err<0)
	{
	PORTD|=1<<PD6;
	PORTD&=~(1<<PD0);	
	PORTD|=1<<PD5;
	PORTD&=~(1<<PD4);	
	OCR1A=15000;
	OCR1B=15000;
	}
*/				
	
	
	}
		USART_TransmitNumber(err*100,0);
		USART_Transmitchar(' ',0);
		USART_TransmitNumber(left_tx,0);
		USART_Transmitchar(' ',0);
		USART_TransmitNumber(right_tx,0);
		USART_Transmitchar(0x0D,0);

}


