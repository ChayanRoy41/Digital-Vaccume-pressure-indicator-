/*
 * Digital Medical Gas Alarm System.c
 *
 * Created: 26-08-2023 19:42:51
 * Author : Dott Systems
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/eeprom.h>

//Macros---------------------------
#define OVERLOAD    0x45
#define OVERVOLTAGE 0x55
#define NORMAL      0x65
#define HIGH_THRESHOLD 50
#define VOLTAGE_SCALE  1005
#define VOLTAGE_AMP_GAIN  2
#define CURRENT_AMP_GAIN  12.70
#define BLEEDER_RES  1.0
//Pressure macro--------------------------
#define HIGH_TH 300
#define LOW_TH  10
//----------------------------------------
//PV-------------------------------
//channel count variable------------------------------
char ch_count=1;
char status=NORMAL;
uint8_t volt_count=0;
float actual_volt=0;
char volt_ch_count=1;
char amp_ch_count=0;
float load=0;
float max_load_current=0;
volatile float load_current=0;
int rising=0,falling=0;
int duty=0,k=0;
uint8_t segment_no=1;
int digit1=0,digit2=0,digit3=0,digit4=0;
float prev_rms_current=0;
float alpha=0.5;
float rms_current=0;
int fault_count=0;
uint8_t fault=0;
//PVFP-----------------------------
void display_digit(uint8_t data);
void io_initialization(void);
void shift_data(void);
void latch_data(void);
void display_value_float(float value);
void display_value_int(int value);
void display_amp();
void adc_init();
int adc_read(int ch);
float calculate_amp(void);
void calculate_voltage(void);
void timer1_initialization(void);
//Display functions------------
void display_number(int digit);
void get_number(float number);
//----------------------------------
//ADC functions------------------------------
void adc_init()
{
     ADMUX=(1<<REFS0);   
	//ADMUX = (0<<REFS0) | (0<<REFS1);
	
	// ADC Enable and prescaler of 128
	// 8000000/128 = 62500
	ADCSRA = (1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
}
int adc_read(int ch)
{
	// select the corresponding channel 0~7
	// ANDing with '7' will always keep the value
	// of 'ch' between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing
	
	// start single conversion
	// write '1' to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes '0' again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}
//Number display---------------------------------------------------
void display_number(int number){
	
	
	switch(number)
	{
		case 0:
		    PORTD |=(1<<0);
			PORTD |=(1<<1);
			PORTD |=(1<<2);
			PORTD |=(1<<3);
			PORTD |=(1<<4);
			PORTD |=(1<<5);
			PORTD &=~(1<<6);
		   break;
		case 1:
		      PORTD &=~(1<<0);
		      PORTD |=(1<<1);
		      PORTD |=(1<<2);
		      PORTD &=~(1<<3);
		      PORTD &=~(1<<4);
		      PORTD &=~(1<<5);
		      PORTD &=~(1<<6);
		   break;
		case 2:
		       PORTD |=(1<<0);
		       PORTD |=(1<<1);
		       PORTD &=~(1<<2);
		       PORTD |=(1<<3);
		       PORTD |=(1<<4);
		       PORTD &=~(1<<5);
		       PORTD |=(1<<6);
		   break;
		case 3:
		      PORTD |=(1<<0);
		      PORTD |=(1<<1);
		      PORTD |=(1<<2);
		      PORTD |=(1<<3);
		      PORTD &=~(1<<4);
		      PORTD &=~(1<<5);
		      PORTD |=(1<<6);
		   break;
		case 4:
		       PORTD &=~(1<<0);
		       PORTD |=(1<<1);
		       PORTD |=(1<<2);
		       PORTD &=~(1<<3);
		       PORTD &=~(1<<4);
		       PORTD |=(1<<5);
		       PORTD |=(1<<6);
		   break;
		case 5:
		         PORTD |=(1<<0);
		         PORTD &=~(1<<1);
		         PORTD |=(1<<2);
		         PORTD |=(1<<3);
		         PORTD &=~(1<<4);
		         PORTD |=(1<<5);
		         PORTD |=(1<<6);
		   break;
		case 6:
		      PORTD |=(1<<0);
		      PORTD &=~(1<<1);
		      PORTD |=(1<<2);
		      PORTD |=(1<<3);
		      PORTD |=(1<<4);
		      PORTD |=(1<<5);
		      PORTD |=(1<<6);
		   break;
		case 7:
		        PORTD |=(1<<0);
		        PORTD |=(1<<1);
		        PORTD |=(1<<2);
		        PORTD &=~(1<<3);
		        PORTD &=~(1<<4);
		        PORTD &=~(1<<5);
		        PORTD &=~(1<<6);
		   break;
		case 8:
		        PORTD |=(1<<0);
		        PORTD |=(1<<1);
		        PORTD |=(1<<2);
		        PORTD |=(1<<3);
		        PORTD |=(1<<4);
		        PORTD |=(1<<5);
		        PORTD |=(1<<6);
		   break;
		case 9:
		          PORTD |=(1<<0);
		          PORTD |=(1<<1);
		          PORTD |=(1<<2);
		          PORTD |=(1<<3);
		          PORTD &=~(1<<4);
		          PORTD |=(1<<5);
		          PORTD |=(1<<6);
		   break;
		                              
	}

	
	
}
//Get number--------------------------------------------------------
void get_number(float number){
	
	int y=0;
	float temp_num=0;
	temp_num=number*10;
	y=(int)temp_num;
	
	digit4=y%10;
	y=y/10;
	digit3=y%10;
	y=y/10;
	digit2=y%10;
	y=y/10;
	digit1=y;
	switch(segment_no)
	{
		case 1:
		PORTC |=(1<<3);
		PORTC &=~(1<<4);
		PORTC &=~(1<<5);
		
		display_number(digit1);
		break;
		case 2:
		PORTC |=(1<<4);
		PORTC &=~(1<<3);
		PORTC &=~(1<<5);
	
		display_number(digit2);
		break;
		case 3:
		PORTC |=(1<<5);
		PORTC &=~(1<<3);
		PORTC &=~(1<<4);
		
		display_number(digit3);
		break;
		case 4:
	
		PORTC &=~(1<<4);
		PORTC &=~(1<<5);
		PORTC &=~(1<<3);
		display_number(digit4);
		break;
	}
   
	
	
	
	
}
//IO initialization---------------------
void io_initialization(void){
	
	
	DDRD |=(1<<0);//A
	DDRD |=(1<<1);//B
	DDRD |=(1<<2);//C
	DDRD |=(1<<3);//D
	DDRD |=(1<<4);//E
	DDRD |=(1<<5);//F
	DDRD |=(1<<6);//G
	
	//LED-------------------
	DDRC |=(1<<1);//S1
	DDRC |=(1<<2);//S2
	//---------------------------
	DDRC |=(1<<3);//S1
	DDRC |=(1<<4);//S2
	DDRC |=(1<<5);//S3
	DDRD |=(1<<7);//S4
	//---------------------------
	//Buzzer------------
	DDRC |=(1<<1);
	//------------------
	//LED---------------
	DDRC |=(1<<2);
	//------------------
	
    PORTC |=(1<<3);//S1
    PORTC |=(1<<4);//S2
    PORTC |=(1<<5);//S3
    PORTC |=(1<<1);//Green
    PORTC &=~(1<<2);//Red
	PORTD &=~(1<<7);//Buzzer
	
	
}

//calculate  current------------------------------
float calculate_amp()
{ 
	
	uint16_t adc_data_n=0;
	float np=0.00,V_sup=5.0;
	adc_data_n=(uint16_t)adc_read(0) & 0x3ff;
		  np=V_sup*adc_data_n;
		  np=np/1024;
		  np=np-(0.92*5);
		  np=np/0.03826;
		  np=np*7.50062;
		  np=(-1)*np;

		  if(np<=0){
			  
			  np=0;
			  
		  }
/*prev_rms_current=rms_current;
rms_current=0;
uint16_t i=0;
char buff[10]={0};
char buff1[10]={0};	
uint16_t adc_buffer[100]={0};
float rms_buffer[128]={0};
uint16_t  adc_data=0;
float offset=0.00;
float temp_current=0,last_temp_current,load_current,last_load_current=0;
char v_arr[100]={0};
float load_current_array[3]={0};
char load_pc_vrr[100]={0};
float load_percentage=0;
max_load_current=100;

for(i=0;i<100;i++){

adc_buffer[i]=(uint16_t)adc_read(0) & 0x3ff;

adc_data=adc_data+adc_buffer[i];

_delay_us(10);

}

offset=(float)adc_data/100*5.0/1024;
 

for(i=0;i<128;i++){

	adc_data=(uint16_t)adc_read(0) & 0x3ff;		
	rms_buffer[i]=(float)(adc_data*5.0/1024-offset);
	rms_current=rms_current+(rms_buffer[i]*rms_buffer[i]);
	_delay_us(10);
	
}

rms_current=rms_current/128;
rms_current=sqrt(rms_current);
rms_current=rms_current/BLEEDER_RES;
rms_current=rms_current/CURRENT_AMP_GAIN*2000;
if(rms_current>0){
	rms_current=rms_current-0.05;
}

rms_current =alpha * 	rms_current + ((1-alpha) * prev_rms_current);


	
  return rms_current;*/

if(np>HIGH_TH){
  

PORTC |=(1<<2);//Green
PORTC &=~(1<<1);//Red
fault=1;

}
else{
	if(np<HIGH_TH){
         PORTD &=~(1<<7);
		 PORTC |=(1<<1);//Green
		 PORTC &=~(1<<2);//Red
		 fault=0;
	}
}


 return np;
}
//fault detect-------------------------------------------
void amp_detect(void){

	
load_current=calculate_amp();

 //_delay_ms(100);
	/*ch_count++;
	if(ch_count>3){
		ch_count=1;
	}  */
}
//Timer 1 initialization--------------
void timer1_initialization(void){

TCNT1=65412;
TCCR1B |=(1<<CS12);
TIMSK |=(1<<TOIE1);

}

//-----------------------------
int main(void)
{
   
 int timer=0;
 char phase=0;
	
io_initialization();
adc_init();
timer1_initialization();
//lcd_init(LCD_DISP_ON); 
//lcd_clrscr();
//lcd_gotoxy(6,0);
//lcd_puts("1PH LPU");
//_delay_ms(2000);
//lcd_gotoxy(0,0);
//lcd_puts("                     ");

digit1=0;
digit2=0;
digit3=0;
digit4=0;

display_number(0);
_delay_ms(2000);
 sei();
    while (1) 
    {
/*PORTC ^=(1<<1);//S1
PORTC ^=(1<<2);

display_number(phase);
phase=phase+1;
if(phase>9){
	phase=0;
}
_delay_ms(1000);
*/
	 amp_detect();
	_delay_ms(100);
				
	
    }
	
}

ISR(INT0_vect){
	

}

ISR (TIMER1_OVF_vect){

TCNT1=65412;

if(segment_no>3){
	segment_no=1;
if(fault==1){	
	fault_count++;
	if(fault_count>10){
		fault_count=0;
	 PORTD ^=(1<<7);
	}
}
else{
	fault_count=0;
	PORTD &=~(1<<7);
	
}
}
 get_number(load_current);
segment_no++;
}