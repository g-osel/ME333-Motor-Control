// Include guard.
#ifndef ENCODER__H__
#define ENCODER__H__

#define ENCODER_LINES 334
#define RESET_VAL 32768
#define RESOLUTION 4
#define COUNT_PER_REV (ENCODER_LINES * RESOLUTION)

static int encoder_command(int read);

// Initialize the encoder module.
void encoder_init(void);

// Read the encoder angle in ticks.
int encoder_counts(void);

// Read the encoder angle in units of degrees.
float encoder_angle(void);

// Reset the encoder position.
int encoder_reset(void) ;



#endif