

typedef enum {
    IDLE,
    PWM,
    ITEST,
    HOLD,
    TRACK
} mode;

static mode MODE;

void set_mode(mode new_m);
mode get_mode(void);
void currentISR_init(void);
void PWM_init(void);
void motor_output_init(void);
void positionISR_init(void);