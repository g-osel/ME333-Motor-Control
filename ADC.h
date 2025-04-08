#define SAMPLE_TIME 10  

#define SLOPE 3.8986
#define INTERCEPT -1987.5

void adc_init(void);

unsigned int adc_sample_convert(int pin) ;

unsigned int adc_avg(int pin);

unsigned int current_read(int pin);