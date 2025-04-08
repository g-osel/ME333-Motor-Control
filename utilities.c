#include "utilities.h"
#include <stdio.h>
#include "NU32.h"  


/* changes the mode to the new mode */
void set_mode(mode new_m) {
	MODE = new_m;
}

/* returns our enum telling us what the mode is */
mode get_mode(void){
	return MODE;
}


/* initializing ISR peripherals for current controller */
void currentISR_init(void){
  T2CONbits.TCKPS = 1;            // Timer2 prescaler N=2 (1:2)
  PR2 = 7999;                    // period = (PR2+1) * N * 12.5 ns = 1/5000 us, 5kHz
  TMR2 = 0;                       // initial TMR2 count is 0
  T2CONbits.ON = 1;               // turn on Timer3
  IPC2bits.T2IP = 5;              // step 4: interrupt priority 5
  IPC2bits.T2IS = 0;              // step 4: interrupt priority 1
  IFS0bits.T2IF = 0;              // step 5: clear the int flag
  IEC0bits.T2IE = 1;              // step 6: enable INT0 by setting IEC0<3>
}


/* setting up PWM output */
void PWM_init(void){
  OC1CONbits.OC32 = 0;
  OC1CONbits.OCTSEL = 1;
  T3CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
  PR3 = 3999;          // period = (PR3+1) * N * 12.5 ns = 1/20000 us, 20 kHz
  TMR3 = 0;            // initial TMR3 count is 0
  OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 1000;           // duty cycle = OC1RS/(PR3+1) = 25%
  OC1R = 1000;            // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;       // turn on Timer3
  OC1CONbits.ON = 1;      // turn on OC1
  TRISDbits.TRISD1 = 0;		// make RD1 digital output for motor 
}

void motor_output_init(void){
	TRISDbits.TRISD1 = 0;		// make RD1 digital output for motor
	ODCDbits.ODCD1 = 0;				//Set as a buffered output (high or low)
}


/* initializing ISR peripherals for position controller */
void positionISR_init(void){
  T4CONbits.TCKPS = 0b100;            // Timer4 prescaler N=16 (1:16)
  PR4 = 24999;                    // period = (PR3+1) * N * 12.5 ns = 1/200 us, 200Hz
  TMR4 = 0;                       // initial TMR2 count is 0
  T4CONbits.ON = 1;               // turn on Timer3
  IPC4bits.T4IP = 6;              // step 4: interrupt priority 6
  IPC4bits.T4IS = 0;              // step 4: interrupt priority 1
  IFS0bits.T4IF = 0;              // step 5: clear the int flag
  IEC0bits.T4IE = 1;              // step 6: enable INT0 by setting IEC0<3>
}