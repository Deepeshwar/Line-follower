/*
 * testing_code.c
 *
 * Created: 9/29/2016 5:05:57 PM
 *  Author: Baba
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/delay.h>
#include <math.h>
#include <avr/interrupt.h> 
#include "USART_128.h"

int Count_flag=1;
int count=0; 
int count2=0;
void pwm_init(void)
{
	DDRE=0xFF;
	TCCR3A|=1<<COM3A1|1<<COM3B1|1<<WGM31;
	TCCR3B|=1<<WGM32|1<<WGM33|1<<CS31;
	
}

void timer_init(void)
{
	TIMSK|=1<<OCIE1A;
	TCCR1B|=1<<CS12;		//256 prescalar
	OCR1A=31000;
}


int main(void)
{
	USART_Init(12,0);
	pwm_init();
	timer_init();
	DDRC|=0x00;
	DDRD|=0xFF;
	DDRE|=0xFF;
	ICR3=17000;
	sei();
	while(1)
    {	//USART_TransmitNumber(count,0);
		//USART_Transmitchar(0x0D,0);
		if((bit_is_set(PINC,0))&&(bit_is_set(PINC,1))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_set(PINC,5))&&(bit_is_set(PINC,6)))
		{
		if (bit_is_clear(PINC,7))
		
		 {
			if((count%2)==1)
				{ 
				    OCR3A=4000;
					OCR3B=4000;
				}
			
			if ((count%2)==0)
			   {
				OCR3A=7000;
				OCR3B=500;
		   	   }
			
			
		 }	 	 
		else if((bit_is_set(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_clear(PINC,0))&&(bit_is_clear(PINC,1))&&(bit_is_clear(PINC,5))&&(bit_is_clear(PINC,6)))
			{
				
				OCR3A=4000;		//for left motor
				OCR3B=4000;		//for right motor
			}
		 
		else if((bit_is_set(PINC,0))&&(bit_is_set(PINC,1))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_set(PINC,5))&&(bit_is_clear(PINC,6)))

		{
			if(bit_is_clear(PINC,7))
				{
					if(count>6 && count<9)
					{
						while ((bit_is_set(PINC,0))||(bit_is_set(PINC,1))||(bit_is_set(PINC,5))||(bit_is_set(PINC,6)))
						{
							OCR3A=0;
							OCR3B=10000;
						}
					}
					else
					{
						OCR3A=4000;		//straight
						OCR3B=4000;
						if(Count_flag==1)
						{
							Count_flag=0;
							count++;
							timer_init();
						}
					}

				}
				else
				{
					OCR3A=0;
					OCR3B=10000;
				}
		}
	
		else if((bit_is_set(PINC,0))&&(bit_is_set(PINC,1))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_clear(PINC,5))&&(bit_is_clear(PINC,6)))
		{
			
			if(bit_is_clear(PINC,7))
			{
				if(count>6 && count<9)
				{
					while ((bit_is_set(PINC,0))||(bit_is_set(PINC,1))||(bit_is_set(PINC,5))||(bit_is_set(PINC,6)))
					{
						OCR3A=0;
						OCR3B=10000;
					}
				}
				else
				{
					OCR3A=4000;		//straight
					OCR3B=4000;
					if(Count_flag==1)
					{
						Count_flag=0;
						count++;
						timer_init();
					}
					
				}
				
			}
			else
			{
				OCR3A=0;
				OCR3B=10000;
			}
	
		}
		else if (bit_is_clear(PINC,0)&&(bit_is_clear(PINC,1))&&(bit_is_clear(PINC,2))&&(bit_is_clear(PINC,3))&&(bit_is_clear(PINC,4))&&(bit_is_clear(PINC,5))&&(bit_is_clear(PINC,6))&&(bit_is_clear(PINC,7)))
		{
			while ((bit_is_set(PINC,0))||(bit_is_set(PINC,1))||(bit_is_set(PINC,5))||(bit_is_set(PINC,6)))
			{
				OCR3A=13000;
				OCR3B=0;
			}
			
		}
		else if((bit_is_clear(PINC,0))&&(bit_is_set(PINC,1))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_set(PINC,5))&&(bit_is_set(PINC,6)))
		{
			
					while ((bit_is_set(PINC,0))||(bit_is_set(PINC,1))||(bit_is_set(PINC,5))||(bit_is_set(PINC,6)))
					{
						OCR3A=7000;
						OCR3B=0;
					}	
			
			
		}
		else if((bit_is_clear(PINC,1))&&(bit_is_clear(PINC,0))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_set(PINC,5))&&(bit_is_set(PINC,6)))	//changing
		{
			while ((bit_is_set(PINC,0))||(bit_is_set(PINC,1))||(bit_is_set(PINC,5))||(bit_is_set(PINC,6)))
			{
				OCR3A=5000;					//right turn
				OCR3B=0;
			}
		}

			
	else if((bit_is_clear(PINC,0))&&(bit_is_clear(PINC,1))&&(bit_is_clear(PINC,2))&&(bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_set(PINC,5))&&(bit_is_set(PINC,6)))
	{
				OCR3A=7000;
				OCR3B=3000;
	}
	
	
			else if((bit_is_set(PINC,0))&&(bit_is_set(PINC,1))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3)))
			{
				OCR3A=300;     
				OCR3B=7000;
			}
			else if((bit_is_set(PINC,0))&&(bit_is_set(PINC,1))&&(bit_is_set(PINC,2)))
			{
				OCR3A=500;    
				OCR3B=7000;
			}		
				else if((bit_is_set(PINC,1))&&(bit_is_set(PINC,2))&&(bit_is_set(PINC,3)))
				{		
					OCR3A=3000;		//for left motor
					OCR3B=7000;		//for right motor	
				}
				else if((bit_is_set(PINC,3))&&(bit_is_set(PINC,4))&&(bit_is_set(PINC,5)))
				{
					OCR3A=7000;		//for left motor
					OCR3B=3000;		//for right motor	
				}
				else if((bit_is_set(PINC,4))&&(bit_is_set(PINC,5))&&(bit_is_set(PINC,6)))
				{
					OCR3A=7000;
					OCR3B=500;
				}
		
		else if((bit_is_set(PINC,5))&&(bit_is_set(PINC,6)))
		{
			OCR3A=7000;
			OCR3B=500;
		}
		else if((bit_is_set(PINC,0))&&(bit_is_set(PINC,1)))
		
		{
			OCR3A=500;     //white me set ho jayega and black me clear hoga
			OCR3B=7000;
		}
	else if((bit_is_set(PINC,6)))
	{
		OCR3A=12000;
		OCR3B=500; 
		
	}
	else if((bit_is_set(PINC,0)))
	
	{
		OCR3A=500;     //white me set ho jayega and black me clear hoga
		OCR3B=7000;
	}
	
	else
	PORTD=0b00000110;
	}
}

ISR(TIMER1_COMPA_vect)
{
	Count_flag=1;	
	
	TCCR1A=0;
	TCCR1B=0;
	TIMSK=0;
}
