#include "NU32.h" // config bits, constants, funcs for startup and UART
#include "stdio.h"
// include other header files here
#include "utilities.h"
#include "encoder.h"
#include "ADC.h"

// #define SAMPLE_TIME 10

#define BUF_SIZE 200
#define DIRECTION_PIN LATDbits.LATD1 //Pin controlling the motor direction on the PIC
#define FWD 1
#define REV 0
#define PLOT_PTS 100 //Current reference array
#define TRAJ 10000

static volatile float PWMinput = 0;

// Current Controller Stuff
static volatile float KpCurr = 0, KiCurr = 0;
static volatile int EintCurr = 0, eCurr=0;       //PI integral of the error
static volatile int REFarray[PLOT_PTS]; //reference values to plot
static volatile int ADCarray[PLOT_PTS]; //measured values to plot
static volatile int StoringData = 0;    // if this flag = 1, currently storing
                                        // plot data

// Position Controller Stuff
static volatile float KpPos = 0, KiPos = 0, KdPos = 0;
static volatile int EintPos = 0, EdotPos = 0, eprev = 0; //PI integral of the error, derivative of error, previous error
static volatile float DesAng = 0;                        // User specified angle of interest
static volatile float uPos;
static volatile float uPosNew;
static volatile float TrajREFarray[TRAJ]; //reference values to plot for trajectory
static volatile float TrajSAMParray[TRAJ];        //reference values to plot for trajectory
static volatile int Traj_N = 0; //used to know the size of sampes being received
static volatile int tcounter = 0;               //used to iterate thru the array
static volatile int ePos = 0;
/* Position Controller */

void __ISR(_TIMER_4_VECTOR, IPL6SOFT) PosController(void)
{ // _TIMER_4_VECTOR = 8

  // static float uPos;
  // static float uPosNew;
  switch (get_mode())
  {
  case HOLD:
  {
    float r, s;
    r = DesAng;
    s = encoder_angle();
    ePos = r - s;
    EintPos += ePos;
    EdotPos = ePos - eprev;

    if (EintPos > 1000)
    { // ADDED: integrator anti-windup
      EintPos = 1000;
    }
    else if (EintPos < -1000)
    { //ADDED:integrator anti-windup
      EintPos = -1000;
    }

    uPos = (KpPos * ePos) + (KiPos * EintPos) + (KdPos * EdotPos); //the position controller
    //uPosNew = uPos;
    eprev = ePos;


    break;
  }
  case TRACK:
  {

    static float rt, st;
    //read the encoder, ref is given by user
    rt = TrajREFarray[tcounter];                   //new to the track mode
    st = encoder_angle();
    
    ePos = rt - st;
    EintPos += ePos;
    EdotPos = ePos - eprev;

    if (EintPos > 1000)
    { // ADDED: integrator anti-windup
      EintPos = 1000;
    }
    else if (EintPos < -1000)
    { //ADDED:integrator anti-windup
      EintPos = -1000;
    }

    uPos = (KpPos * ePos) + (KiPos * EintPos) + (KdPos * EdotPos); //the position controller
    //uPosNew = uPos;
    eprev = ePos;    


    TrajSAMParray[tcounter] = st;    
    //increment the counter
    tcounter++;
    //get out to avoid indexing past the array capacity
    if(tcounter == Traj_N){
      DesAng = TrajREFarray[Traj_N-1];      //set holding angle
      StoringData = 0;
      set_mode(HOLD);
    }

    break;
  }
  default:
  {
    NU32_LED2 = 0; // turn on LED2 to indicate an error
    break;
  }
  }
  // NU32_LED1 = !NU32_LED1;
  // OC1RS = 1000; // insert line(s) to set OC1RS
  TMR4 = 0;
  IFS0bits.T4IF = 0; // insert line to clear interrupt flag
}

/* Current Controller */
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrController(void)
{                         // _TIMER_2_VECTOR = 8
  static int counter = 0; // initialize counter once
  //static int eCurr;
  static float uCurr;
  static float uCurrNew;

  switch (get_mode())
  {
  case IDLE: // dummy command for demonstration purposes
  {
    /* Put the H-Bridge in Brake Mode */
    OC1RS = 0;

    break;
  }
  case PWM: // addition command for demonstration purposes
  {
    if (PWMinput < 0)
    {
      OC1RS = (PWMinput / -100) * (PR3 + 1); // Calculating new duty cycle based on client
      DIRECTION_PIN = REV;                   // Going in reverse
    }
    else if (PWMinput >= 0)
    {
      OC1RS = (PWMinput / 100) * (PR3 + 1); // Calculating new duty cycle based on client
      DIRECTION_PIN = FWD;                  // Going forward
    }
    break;
  }
  case ITEST:
  {
    static volatile int amplitude = 200;
    /*Making the reference waveform*/
    if (counter < PLOT_PTS)
    {
      // Toggling between +200 and−200 mA every 25 counts
      if (counter % 25 == 0)
      {
        amplitude *= -1; //flip sign every 25
      }
      ADCarray[counter] = current_read(0);
      REFarray[counter] = amplitude;

      // Set mode to IDLE at count = 99
      if (counter == PLOT_PTS - 1)
      {
        set_mode(IDLE);
        counter = 0;
        StoringData = 0; // tell main data is ready to be sent to MATLAB
      }
    }

    static int r, s;
    r = REFarray[counter];
    s = ADCarray[counter];

    eCurr = r - s; //calculate error
    EintCurr += eCurr;

    if (EintCurr > 1000)
    { // ADDED: integrator anti-windup
      EintCurr = 1000;
    }
    else if (EintCurr < -1000)
    { //ADDED:integrator anti-windup
      EintCurr = -1000;
    }

    uCurr = (KpCurr * eCurr) + (KiCurr * EintCurr); //the current controller
    uCurrNew = uCurr;

    if (uCurrNew > 100.0)
    {
      uCurrNew = 100.0;
    }
    else if (uCurrNew < -100.0)
    {
      uCurrNew = -100.0;
    }

    /*Set direction bit*/
    if (uCurrNew < 0.0)
    {

      uCurrNew = abs(uCurrNew); //bring to positive
      // OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
      DIRECTION_PIN = REV;
    }
    else 
    {
      // OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
      DIRECTION_PIN = FWD;
    }
    /*Update the duty cycle PWM*/

    //char test[BUF_SIZE];
    OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
    //sprintf(test, "%d\r\n", OC1RS);
    //NU32_WriteUART3(test);

    counter++;
    break;
  }
  case HOLD:
  {
    static int rp, sp;
    rp = uPos;
    sp = current_read(0);

    eCurr = rp - sp; //calculate error
    EintCurr += eCurr;

    if (EintCurr > 1000)
    { // ADDED: integrator anti-windup
      EintCurr = 1000;
    }
    else if (EintCurr < -1000)
    { //ADDED:integrator anti-windup
      EintCurr = -1000;
    }

    uCurr = (KpCurr * eCurr) + (KiCurr * EintCurr); //the current controller
    uCurrNew = uCurr;

    if (uCurrNew > 100.0)
    {
      uCurrNew = 100.0;
    }
    else if (uCurrNew < -100.0)
    {
      uCurrNew = -100.0;
    }

    /*Set direction bit*/
    if (uCurrNew < 0.0)
    {

      uCurrNew = abs(uCurrNew); //bring to positive
      //OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
      DIRECTION_PIN = REV;
    }
    else
    {
      //OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
      DIRECTION_PIN = FWD;
    }
    NU32_LED1 = 0;
    /*Update the duty cycle PWM*/
    OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));

    break;
  }
  case TRACK:
  {
    // HOLD does the work here
    static int rp, sp;
    rp = uPos;
    sp = current_read(0);

    eCurr = rp - sp; //calculate error
    EintCurr += eCurr;

    if (EintCurr > 1000)
    { // ADDED: integrator anti-windup
      EintCurr = 1000;
    }
    else if (EintCurr < -1000)
    { //ADDED:integrator anti-windup
      EintCurr = -1000;
    }

    uCurr = (KpCurr * eCurr) + (KiCurr * EintCurr); //the current controller
    uCurrNew = uCurr;

    if (uCurrNew > 100.0)
    {
      uCurrNew = 100.0;
    }
    else if (uCurrNew < -100.0)
    {
      uCurrNew = -100.0;
    }

    /*Set direction bit*/
    if (uCurrNew < 0.0)
    {

      uCurrNew = abs(uCurrNew); //bring to positive
      //OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
      DIRECTION_PIN = REV;
    }
    else
    {
      //OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));
      DIRECTION_PIN = FWD;
    }
    NU32_LED1 = 0;
    /*Update the duty cycle PWM*/
    OC1RS = (unsigned int)((uCurrNew / 100.0) * (float)(3999));

    break;
  }
  default:
  {
    NU32_LED2 = 0; // turn on LED2 to indicate an error
    break;
  }
  }

  // LATDINV = 0b10;
  // OC1RS = 1000; // insert line(s) to set OC1RS
  TMR2 = 0;
  IFS0bits.T2IF = 0; // insert line to clear interrupt flag
}

int main()
{

  int pin_val = 0;
  adc_init();

  set_mode(IDLE);
  char buffer[BUF_SIZE];
  char message[BUF_SIZE]; // message to and from MATLAB
  NU32_Startup();         // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;          // turn off the LEDs
  NU32_LED2 = 1;
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  // ISR_init();
  encoder_init(); // SPI initialization for reading from the decoder chip
  currentISR_init();
  PWM_init();
  motor_output_init();
  positionISR_init();

  __builtin_enable_interrupts();

  while (1)
  {
    int i;
    // TRISDbits.TRISD2 = 0;		// make RD2 digital output to test 200Hz Controller
    // ODCDbits.ODCD2 = 0;				//Set as a buffered output (high or low)
    NU32_ReadUART3(buffer, BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                    // clear the error LED
    switch (buffer[0])
    {
    case 'y': // dummy command for demonstration purposes
    {
      int n = 0;
      NU32_ReadUART3(buffer, BUF_SIZE);
      sscanf(buffer, "%d", &n);
      sprintf(buffer, "%d\r\n", n + 1); // return the number + 1
      NU32_WriteUART3(buffer);
      break;
    }
    case 'x': // addition command for demonstration purposes
    {
      int n = 0;
      int o = 0;
      NU32_ReadUART3(buffer, BUF_SIZE);
      sscanf(buffer, "%d %d", &n, &o);
      sprintf(buffer, "%d\r\n", n + o); // return the numbers added together
      NU32_WriteUART3(buffer);
      break;
    }
    case 'q':
    {
      // handle q for quit. Later you may want to return to IDLE mode here.
      set_mode(IDLE);
      break;
    }
    case 'c':
    {
      sprintf(buffer, "%d\r\n", encoder_counts());
      NU32_WriteUART3(buffer); // send encoder count to client
      break;
    }
    case 'd':
    {
      sprintf(buffer, "%f\r\n", encoder_angle());
      NU32_WriteUART3(buffer); // send encoder count in degrees to client
      break;
    }
    case 'e':
    {
      sprintf(buffer, "%d\r\n", encoder_reset());
      NU32_WriteUART3(buffer); // send encoder count reset value to client
      break;
    }
    case 'r':
    {
      sprintf(buffer, "%d\r\n", get_mode());
      NU32_WriteUART3(buffer);
      break;
    }
    case 'a':
    {
      sprintf(buffer, "%d\r\n", adc_avg(pin_val));
      NU32_WriteUART3(buffer);
      break;
    }
    case 'b':
    {
      sprintf(buffer, "%d\r\n", current_read(pin_val));
      NU32_WriteUART3(buffer);
      break;
    }
    case 'f': // Set PWM
    {

      float pwm_val = 0;
      NU32_ReadUART3(buffer, BUF_SIZE);
      sscanf(buffer, "%f", &pwm_val);
      sprintf(buffer, "%f\r\n", pwm_val); // return the client value
      NU32_WriteUART3(buffer);
      PWMinput = pwm_val;
      set_mode(PWM);

      break;
    }
    case 'p': // Unpower motor
    {
      set_mode(IDLE);
      break;
    }
    case 'g': // Set current gains
    {
      float Kp = 0;
      float Ki = 0;
      NU32_ReadUART3(buffer, BUF_SIZE);
      sscanf(buffer, "%f %f", &Kp, &Ki);
      // sprintf(buffer,"Kp: %f Ki: %f\r\n", Kp, Ki); // return the numbers added together
      // NU32_WriteUART3(buffer);
      __builtin_disable_interrupts();
      KpCurr = Kp;
      KiCurr = Ki;
      __builtin_enable_interrupts();
      break;
    }
    case 'h': // Get current gains
    {
      sprintf(buffer, "%f %f\r\n", KpCurr, KiCurr); // return the numbers added together
      NU32_WriteUART3(buffer);
      break;
    }
    case 'k': // Test current gains
    {
      set_mode(ITEST);
      StoringData = 1;
      while (StoringData)
      {
        ; //Collecting data
      }

      NU32_WriteUART3("100\n"); // Tells Matlab how many samples are coming
      for (i = 0; i < PLOT_PTS; i++)
      { // send plot data to MATLAB
        sprintf(message, "%d %d\r\n", REFarray[i], ADCarray[i]);
        NU32_WriteUART3(message);
      }
      break;
    }
    case 'i': // Set position gains
    {
      float Kp = 0;
      float Ki = 0;
      float Kd = 0;
      NU32_ReadUART3(buffer, BUF_SIZE);
      sscanf(buffer, "%f %f %f", &Kp, &Ki, &Kd);
      // sprintf(buffer,"Kp: %f Ki: %f\r\n", Kp, Ki); // return the numbers added together
      // NU32_WriteUART3(buffer);
      __builtin_disable_interrupts();
      KpPos = Kp;
      KiPos = Ki;
      KdPos = Kd;
      __builtin_enable_interrupts();
      break;
    }
    case 'j': // Get position gains
    {
      sprintf(buffer, "%f %f %f\r\n", KpPos, KiPos, KdPos); // return the numbers added together
      NU32_WriteUART3(buffer);
      break;
    }
    case 'l': // Get user specified degree to go to
    {
      set_mode(IDLE);
      int des_ang = 0;

      
      __builtin_disable_interrupts();
      EintPos = 0;
      EdotPos = 0;
      ePos = 0;
      eCurr = 0;
      EintCurr = 0;
      NU32_ReadUART3(buffer, BUF_SIZE);
      sscanf(buffer, "%d", &des_ang);
      // sprintf(buffer,"Kp: %f Ki: %f\r\n", Kp, Ki); // return the numbers added together
      // NU32_WriteUART3(buffer);
      DesAng = des_ang;
      set_mode(HOLD);
      __builtin_enable_interrupts();
      break;
    }
    case 'm': //Store step trajectory
    {
      set_mode(IDLE);
      int traj_N=0;
      float traj_samp=0;
      NU32_ReadUART3(message, BUF_SIZE); // wait for a message from MATLAB
      sscanf(message, "%d", &traj_N);    //store the number of samples we are expecting locally
      __builtin_disable_interrupts();
      
      Traj_N = traj_N;                   //Update globally
      /*store each element */
      int t;
      for (t = 0; t < traj_N; t++)
      {
        /*store each element into the array*/
        NU32_ReadUART3(message, BUF_SIZE); // wait for a message from MATLAB
        sscanf(message, "%f", &traj_samp); //store the current ref sample locally
        TrajREFarray[t] = traj_samp;       //store it in the global array
      }
      
      // sprintf(message, "%d\r\n", traj_N);
      // NU32_WriteUART3(message);
      tcounter = 0; //reset the counter
      __builtin_enable_interrupts();
      break;
    }
    case 'n': //Store cubic trajectory
    {
      set_mode(IDLE);
      int traj_N=0; 
      float traj_samp=0;
      __builtin_disable_interrupts();
      NU32_ReadUART3(message, BUF_SIZE); // wait for a message from MATLAB
      sscanf(message, "%d", &traj_N);    //store the number of samples we are expecting locally
      Traj_N = traj_N;                   //Update globally
      /*store each element */
      int t;
      for (t = 0; t < traj_N; t++)
      {
        /*store each element into the array*/
        NU32_ReadUART3(message, BUF_SIZE); // wait for a message from MATLAB
        sscanf(message, "%f", &traj_samp); //store the current ref sample locally
        TrajREFarray[t] = traj_samp;       //store it in the global array
      }
      
      // sprintf(message, "%d\r\n", traj_N);
      // NU32_WriteUART3(message);
      tcounter = 0; //reset the counter
      __builtin_enable_interrupts();
      break;
    }
    case 'o': //Execute trajectory
    {
      encoder_reset();
      set_mode(TRACK);
      while (get_mode() == TRACK)
      {
        ; //wait to get all data
      }
      int i;
      /*Send the shits to matlab (current)*/
      sprintf(message, "%d\r\n", Traj_N);
      NU32_WriteUART3(message); 
      for (i = 0; i < Traj_N; i++)
      { // send plot data to MATLAB
        sprintf(message, "%f %f\r\n", TrajREFarray[i], TrajSAMParray[i]);
        NU32_WriteUART3(message);
      }
      break;
    }
    default:
    {
      NU32_LED2 = 0; // turn on LED2 to indicate an error
      break;
    }
    }
  }
  return 0;
}
