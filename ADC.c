#include "ADC.h"
#include <stdio.h>
#include "NU32.h" 



/* Initialize ADC pin 0 */
void adc_init(void){
  AD1CON1bits.SSRC = 0b111;       //        conversion starts when sampling ends
  AD1CON1bits.ASAM = 0;           //        manual sampling
  AD1PCFGbits.PCFG0 = 0;                 // AN0 is an adc pin
  AD1CON3bits.ADCS = 2;                   // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
                                          //                           2*3*12.5ns = 75ns
    
  AD1CON3bits.SAMC = 20;           //        sample for 2 Tad
  AD1CON1bits.ADON = 1;                   // turn on A/D converter
}


/* Read ADC from Chapter 10 sample code */
unsigned int adc_sample_convert(int pin) { // sample & convert the value on the given 
                                           // adc pin the pin should be configured as an 
                                           // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = pin;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) { 
      ;                                   // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    return ADC1BUF0;                      // read the buffer with the result
}

unsigned int adc_avg(int pin){
	unsigned int sum = 0;
	int i;
	for(i = 0; i < 7; i++){
		sum += adc_sample_convert(pin);
	}
	return (sum / 7);
}


/* Convert ADC count to mA */
unsigned int current_read(int pin){
    unsigned int current = 0;
    current = (adc_avg(pin) * SLOPE) + INTERCEPT;
    return current;
}