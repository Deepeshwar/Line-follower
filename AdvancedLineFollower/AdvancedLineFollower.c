#define  F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include "USART_128.h"
#define lineSensorPort PINE
#define angleConst 10
#define kd 1
#define distancePerTick 0.05669
#define motorSensorDist 9
volatile float sensor[7],currentSensorReading=0,distance=0;
volatile float prevSensorReading=0,newErrorFromEqn=0,prevErrorFromEqn=0,m=0,c=0;
volatile int outOfLine=0;
void pwm_Init(){
	DDRB = 0xFF;
	TCCR1A |= (1<<COM1A1) | (1<<WGM11) ;
	TCCR1B |= (1<<CS11) | (1<< WGM13) | (1<<WGM12);	//non inverting mode,prescaler 8
	ICR1 = 19999;
}
void pwm_Init_Motor(){
	
	TCCR0 |= (1<<CS01) |(1<<COM01)|(1<< WGM01) | (1<<WGM00);	//non inverting mode,prescaler 8
	TCCR2 |= (1<<CS21) |(1<<COM21)|(1<< WGM21) | (1<<WGM20);	//non inverting mode,prescaler 8

}
void motor_speed_percent(int percent1,int percent2){
	OCR0=(int)(percent1*2.55);
	OCR2=(int)(percent2*2.55);
}
void moveServo_to_angle(int m)
{
	m+=90;
	OCR1A = m * (11.38) + 550.0;
}

void interrupt_Initialise(){
	//Initialize Interrupts on PD1
	EICRA|=1<<ISC01;
	EIMSK|=1<<INT0;
}
void read_Sensor(){
	//Line Sensor On Port E
	int i;
	for(i=0;i<7;i++){
		if(bit_is_clear(lineSensorPort,i)){
			sensor[i]=1;
		}
		else{
			sensor[i]=0;
		}
		USART_TransmitNumber(sensor[i],1);
	}
	USART_Transmitchar(0x0d,1);
}
void calculate_Errror(){
	int onsensors=sensor[0]+sensor[1]+sensor[2]+sensor[3]+sensor[4]+sensor[5]+sensor[6];
	currentSensorReading=0;
	for(int i=0;i<7;i++){
		currentSensorReading+=(float)((i-3)*sensor[i]);
	}
	currentSensorReading/=(7-onsensors);
	if((7-onsensors>=4) &&currentSensorReading<0)currentSensorReading--;
	else if((7-onsensors)>=4 && currentSensorReading>0)currentSensorReading++;
}

int main(void)
{	float angle=0;
	USART_Init(51,1);
	DDRE = 0x00;
	DDRD &= ~(1<<PD1); //A+ of encoder on interrupt pin PD1
	DDRD &= ~(1<<PD0);   //B+ of encoder on PD0
	interrupt_Initialise();	
	sei();
	pwm_Init();
	pwm_Init_Motor();
	float prevDistance=0;

	USART_TransmitString("starting",1);
	while(1)
    {		if(distance>(motorSensorDist/10)+prevDistance){
		    read_Sensor();
		    prevSensorReading=currentSensorReading;
			calculate_Errror();	
			
			m=(currentSensorReading-prevSensorReading)/(motorSensorDist);
		   
		    c=prevSensorReading-m*distance;
			prevDistance=distance;			
		}
		prevErrorFromEqn=newErrorFromEqn;
  		newErrorFromEqn=m*distance+c; 		 
 		if(!(sensor[0]==1&&sensor[1]==1&&sensor[2]==1&&sensor[3]==1&&sensor[4]==1&&sensor[5]==1&&sensor[6]==1))
		 {angle=newErrorFromEqn*angleConst + (newErrorFromEqn-prevErrorFromEqn)*kd;
		 }  		
		 if(angle>90)angle=90;
  		if(angle<-90)angle=-90;
  		USART_TransmitNumber(angle,1);
		USART_Transmitchar(0x0d,1);
		moveServo_to_angle(angle);
		
		if(angle*angle<=225)motor_speed_percent(80,80);
		else if(angle*angle<=400) motor_speed_percent(70,70);
		else motor_speed_percent(60,60);
		
	}
}

ISR(INT0_vect){			
	if(bit_is_set(PIND,1)) {
		distance += distancePerTick;	
	} else {
		distance -= distancePerTick;
	}	
}