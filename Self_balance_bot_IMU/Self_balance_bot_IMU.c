/*
 * Self_balance_bot_IMU.c
 *
 * Created: 31-Mar-15 7:08:48 PM
 *  Author: VishnuTS
 */
#define F_CPU 14745600UL 
#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.1415926535897
#define dt 0.00108
#define P_GAIN 40
#define I_GAIN 0
#define D_GAIN 2
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm 
#include "timercounter0.h"
#include "timercounter2.h"

#include "mpu6050.h"

#define UART_BAUD_RATE 57600
#include "uart.h"

int main(void) {
DDRB=0x08;
/* PB3 pin of PORTB is declared output (PWM1 pin of DC Motor Driver is connected) */

DDRD=0x80;
/* PD7 pin of PORTD is declared output (PWM2 pin of DC Motor Driver is connected) */

DDRA=0x0f;
/*PA0,PA1,PA2 and PA3 pins of PortC are declared output ( i/p1,i/p2,i/p3 and i/p4 pins of DC Motor Driver are connected)*/

set_timercounter0_mode(1);
/*Timer counter 0 is set to Phase Correct pwm mode*/

set_timercounter0_prescaler(4);
/*Timer counter 0 frequency is set to 3.90625KHz*/

set_timercounter0_output_mode(2);
/*Timer counter 0 output mode is set to non-inverting mode*/

set_timercounter2_mode(1);
/*Timer counter 2 is set to Phase Correct pwm mode*/

set_timercounter2_prescaler(4);
/*Timer counter 2 frequency is set to 3.90625KHz*/

set_timercounter2_output_mode(2);
/*Timer counter 2 output mode is set to non-inverting mode*/

	#if MPU6050_GETATTITUDE == 0
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    double axg = 0;
    double ayg = 0;
    double azg = 0;
    double gxds = 0;
    double gyds = 0;
    double gzds = 0;
	double accXangle = 0;
	double gyroXangle = 0;
	double Xangle = 0 ;
	double error = 0;
	double I_error = 0;
	double D_error = 0;
	double previous_error = 0 ;
	double outputspeed = 0;
	/*double initangle = 0;*/
	#endif
	
	
   
    //init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	//init interrupt
	sei();

	//init mpu6050
	mpu6050_init();
	_delay_ms(50);
	
	#if MPU6050_GETATTITUDE == 0
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
	accXangle = (atan2(ayg,azg)+PI)*RAD_TO_DEG;
	gyroXangle = accXangle;
	#endif
	
	for(;;) {
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		#endif
		
        accXangle = (atan2(ayg,azg)+PI)*RAD_TO_DEG;
		gyroXangle = accXangle + gxds*dt;
		Xangle = 0.98*gyroXangle + 0.02*accXangle;
		
		error = 180 - Xangle;
		I_error += (error)*dt;
		D_error = (error - previous_error)/*/dt*/;
		
		outputspeed = (P_GAIN * error) + (I_GAIN * I_error) + (D_GAIN * D_error);
		previous_error = error;
	/*Bang Bang Controller 
	if((Xangle<=(180.01))&&(Xangle>=179.99))
	{
		
		PORTA = 0x00;
	}
	else if (Xangle>(180.01))
	{
		set_timercounter0_compare_value(255);
		

		set_timercounter2_compare_value(255);
		
		PORTA = 0x0a;
	}
	else if(Xangle<(179.99))
	{
		set_timercounter0_compare_value(255);
		

		set_timercounter2_compare_value(255);
		
		PORTA = 0x05;
	}
		Bang Bang Controller*/
		if((Xangle<=(180.1))&&(Xangle>=179.9))
		{
			
			PORTA = 0x00;
		}
		else if (Xangle>(180.1))
		{ 
				set_timercounter0_compare_value(abs(outputspeed));
				

				set_timercounter2_compare_value(abs(outputspeed));
				
		PORTA = 0x0a;
		}
		else if(Xangle<(179.9))
		{  
				set_timercounter0_compare_value(abs(outputspeed));
				

				set_timercounter2_compare_value(abs(outputspeed));
				
			PORTA = 0x05;
		}		
		#if MPU6050_GETATTITUDE == 0
		char itmp[10];
		/*dtostrf(ax, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(ay, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(az, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(gx, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(gy, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		 dtostrf(gz, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	    dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(accXangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyroXangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(initangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(Xangle, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(error, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(I_error, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(D_error, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');*/
		dtostrf(outputspeed, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("\r\n");
		#endif 

	}

}
