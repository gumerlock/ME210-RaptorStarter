/******************************************************************************
Module
  Raptor.cpp 
Version
  1.0.2 20190109
Description
  This library contains code to interface with the ME210 Sparki.
  It includes a functions to convert the line sensors into binary chars.

Arduino framework version:  framework-arduinoavr 1.10623.1
  
History
When      Who  Description
--------  ---  -------------------------------------
01/09/16  MTP  adapted RoachLib.cpp for use with Sparkiraptor
10/25/16  AL   combined with code from Sparki.cpp to remove dependency
               on Sparki library
01/13/17  AL   Removed SetServoDeg, other servo functionalities (unused)
01/09/19  KLG  Changed all datatypes to C99 inttypes.h-compatible datatypes;
               verified compatibility with PlatformIO
01/11/19  GG   Fixed uint8_t to int8_t to solve not being able to reverse motor
               directions
******************************************************************************/
/*----------------------------- Include Files -------------------------------*/
#include "Raptor.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "Arduino.h"
#include <stdlib.h>
#include <SPI.h>

/*----------------------------- Module Variables ---------------------------*/

static int8_t step_dir[3];                  // -1 = ccw, 1 = cw  
static uint8_t motor_speed[3];              // stores last set motor speed (0-100%)
uint8_t ir_active = 1;

// Values for the servo
// volatile int8_t servo_deg_offset = 0;

static volatile uint8_t move_speed = 100;
static volatile uint8_t speed_index[3];
// For each motor, how many 200uS waits between each step. 
// Cycles through an array of 10 of these counts to average 
// for better speed control
static volatile uint8_t speed_array[3][SPEED_ARRAY_LENGTH];    

static volatile int8_t step_index[3];             // index into _steps array  
static uint8_t _steps_right[9];                   // bytes defining stepper coil activations
static uint8_t _steps_left[9];                    // bytes defining stepper coil activations
static volatile uint32_t remainingSteps[3];       // number of steps before stopping motor
static volatile uint32_t isRunning[3];            // tells if motor is running

static volatile int16_t speedCounter[3];          // variable used in maintaing speed
static volatile int16_t speedCount[3];            // what speedCount is set at when speed cycle resets

static volatile uint8_t shift_outputs[3];         // tells if motor is running

// Initialize the RGB timer variables
static volatile uint8_t RGB_vals[3];
static volatile uint16_t RGB_timer;
static volatile uint8_t irSwitch;

static uint8_t SharedByte;
static uint16_t SharedWord;

RaptorClass raptor;

/*-------------------- Module Code: Public Functions -----------------------*/

RaptorClass::RaptorClass()
{
 begin();
}

/******************************************************************************
Function:     LeftMtrSpeed
Contents:     This function is used to set the speed and direction of the left motor.
Parameters:   char newSpeed - A value between -100 and 100 which is the new speed of
              the left motor.  0 stops the motor.  A negative value is reverse.
Returns:      OK_OPERATION == new speed was successfully sent
              ERR_BADINPUT == an invalid speed was given
Notes:
******************************************************************************/
RaptorReturn_t RaptorClass::LeftMtrSpeed(int8_t newSpeed) {
  if (abs(newSpeed) > 100) return ERR_BADINPUT;

  int16_t direction = newSpeed > 0 ? DIR_CCW : DIR_CW;
  motorRotate(MOTOR_LEFT, direction, abs(newSpeed));

  return OK_OPERATION;
}

/******************************************************************************
Function:     RightMtrSpeed
Contents:     This function is used to set the speed and direction of the right motor.
Parameters:   char newSpeed - A value between -100 and 100 which is the new speed of
              the right motor.  0 stops the motor.  A negative value is reverse.
Returns:      OK_OPERATION == new speed was successfully sent
              ERR_BADINPUT == an invalid speed was given
Notes:
******************************************************************************/
RaptorReturn_t RaptorClass::RightMtrSpeed(int8_t newSpeed) {
  if (abs(newSpeed) > 100) return ERR_BADINPUT;

  int16_t direction = newSpeed > 0 ? DIR_CW : DIR_CCW;
  motorRotate(MOTOR_RIGHT, direction, abs(newSpeed));

  return OK_OPERATION;
}

/******************************************************************************
Function:     SetServoDeg -- REMOVED 1/13/17
Contents:     This function sets the degree of the Sparki onboard servo.
Parameters:   char deg - the desired degree of the servo.
Notes:        Implementation taken from original Sparki library.  We don't use
              the Servo library or PWM control to interface well with the Sparki
              scheduler and existing funtionality
******************************************************************************/
// RaptorReturn_t RaptorClass::SetServoDeg(uint8_t deg)
// { 
//   if (abs(deg) > 80) return ERR_BADINPUT;

//   // Compute the duty cycle for the servo
//   int16_t duty = int16_t((((float(-deg+servo_deg_offset)*2000/180)+1500)/20000)*1024);
  
//   uint32_t dutyCycle = 20000;
//   dutyCycle *= duty;
//   dutyCycle >>= 10;
   
//   uint8_t oldSREG = SREG;
//   noInterrupts();
//   OCR1A = dutyCycle;
  
//   SREG = oldSREG;
//   interrupts();
//   return OK_OPERATION;
// }

/******************************************************************************
Function:     LightLevel
Contents:     This function is used to read the A/D converter value for the
              light sensor.
Parameters:   None
Returns:      A 10 bit unsigned integer corresponding to the amount of light
              incident on the Raptor's center photocell.  Higher values
              indicate higher light levels.
Notes:        Implementation taken from original the Sparki library.
******************************************************************************/

uint16_t RaptorClass::LightLevel(){
  setMux(LIGHT_CENTER);
  return (uint16_t)analogRead(MUX_ANALOG);
}

/******************************************************************************
  Functions:  EdgeRight, LineRight, LineCenter, LineLeft, EdgeLeft 
  Contents:   These functions correspond with each of the five line sensors on
              the bottom of the Sparki.
  Parameters: None
  Returns:    An unsigned int corresponding to the amount of reflected IR
              detected by the line sensors.  Lower values correspond to lower
              amounts of detected IR ('darker' areas), and higher values
              correspond to higher amounts ('lighter areas')
  Notes:      Implementations taken from original the Sparki library.
******************************************************************************/

uint16_t RaptorClass::EdgeRight(){
  setMux(IR_EDGE_RIGHT);
    return readSensorIR(MUX_ANALOG);
}

uint16_t RaptorClass::LineRight(){
  setMux(IR_LINE_RIGHT);
    return readSensorIR(MUX_ANALOG);
}

uint16_t RaptorClass::LineCenter(){
  setMux(IR_LINE_CENTER);
    return readSensorIR(MUX_ANALOG);
}

uint16_t RaptorClass::LineLeft(){
  setMux(IR_LINE_LEFT);
    return readSensorIR(MUX_ANALOG);
}

uint16_t RaptorClass::EdgeLeft(){
  setMux(IR_EDGE_LEFT);
    return readSensorIR(MUX_ANALOG);
}

/******************************************************************************
Function:     RGB
Contents:     This function sets display colors of the onboard RGB LED
Parameters:   uint8_t R - desired percentage brightness of red LED
              uint8_t G - desired percentage brightness of green LED
              uint8_t B - desired percentage brightness of blue LED
Notes:        Implementation taken from original the Sparki library.
******************************************************************************/

void RaptorClass::RGB(uint8_t R, uint8_t G, uint8_t B)
{
    if(R > 100){
        R = 100;
    }
    if(G > 100){
        G = 100;
    }
    if(B > 100){
        B = 100;
    }
  RGB_vals[0] = uint8_t(R/2.0);
  RGB_vals[1] = uint8_t(G/2.0);
  RGB_vals[2] = uint8_t(B/2.0);
}

/******************************************************************************
Function:     Beep
Contents:     This function activates the buzzer on Sparki
Parameters:   int freq - desired frequency in Hz
              int time - desired duration in milliseconds
Notes:        Implementation taken from original the Sparki library.
******************************************************************************/

void RaptorClass::Beep(uint16_t freq, uint32_t time){
    tone(BUZZER, freq, time);
}

/******************************************************************************
  Function:    ReadTriggers
  Contents:    This function checks the five light sensors and reports whether
                the signal has fallen enough to trigger 1 or more
                of the bumper sensors.
  Parameters:  int threshold - threshold to determine if a line is hit
  Returns:     An 8 bit value where the upper 4 bits correspond to the bumper
                sensors.  If a bumper is hit, the corresponding bit will be 0,
                otherwise it will be 1.  The lower 4 bits always return 0.
  Notes:
******************************************************************************/
uint8_t RaptorClass::ReadTriggers(uint16_t threshold) {
  uint8_t trigger = 0x00;
  trigger = trigger|((LineLeft()<threshold))|
                    ((LineCenter()<threshold)<<2)|
                    ((LineRight()<threshold)<<4);

  return trigger;
}

/******************************************************************************
  Function:    SET_SHARED_BYTE_TO
  Contents:    This function sets the value of the module-level variable
                SharedByte to the new value specified when called.  The data
                is intended to be used immediately afterward using the function
                GET_SHARED_BYTE.
  Parameters:  An 8 bit value.
  Returns:     Nothing
  Notes:
******************************************************************************/
void RaptorClass::SET_SHARED_BYTE_TO(uint8_t newByte)
{
  SharedByte = newByte;
}

/******************************************************************************
  Function:    GET_SHARED_BYTE
  Contents:    This function returns the value of the module-level variable
                SharedByte, and is intended to be called immediately after storing
                a value in SharedByte using the SET_SHARED_BYTE_TO function.
  Parameters:  None
  Returns:     An 8 bit value.
  Notes:
******************************************************************************/
uint8_t RaptorClass::GET_SHARED_BYTE(void)
{
  return SharedByte;
}

/******************************************************************************
  Function:    SET_SHARED_WORD_TO
  Contents:    This function sets the value of the module-level variable
                SharedWord to the new value specified when called.  The data
                is intended to be used immediately afterward using the function
                GET_SHARED_WORD.
  Parameters:  A 16 bit value.
  Returns:     Nothing
  Notes:
******************************************************************************/
void RaptorClass::SET_SHARED_WORD_TO(uint16_t newWord)
{
  SharedWord = newWord;
}

/******************************************************************************
  Function:    GET_SHARED_WORD
  Contents:    This function returns the value of the module-level variable
                SharedWord, and is intended to be called immediately after storing
                a value in SharedWord using the SET_SHARED_WORD_TO function.
  Parameters:  None
  Returns:     A 16 bit value.
  Notes:
******************************************************************************/
uint16_t RaptorClass::GET_SHARED_WORD(void)
{
  return SharedWord;
}

/*-------------------- Module Code: Private Functions ----------------------*/


//  Setup for RaptorClass functionality
void RaptorClass::begin( ) { 
  Serial.begin(9600);

  //  Set up the Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  //  Setup Buzzer
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // //  Setup Servo
  // pinMode(SERVO, OUTPUT);
  // //  Keep offset from going too off
  // if (servo_deg_offset > MAX_SERVO_OFFSET){
  //   servo_deg_offset = 0;
  // }
  // if (servo_deg_offset < -MAX_SERVO_OFFSET){
  //   servo_deg_offset = 0;
  // }
  
  // Setup Analog Multiplexer
  pinMode(MUX_ANALOG, INPUT);
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);

  //  Setup IR send for reflectance sensors
  pinMode(IR_SEND, OUTPUT);
  
  //  Setup the SPI bus for the shift register
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2); 

  //  Set the shift-register clock select pin to output 
  DDRD |= (1<<5);
  
  //  Clear out the shift registers
  PORTD &= 0xDF;                                      //  Pull PD5 low
  SPI.transfer(shift_outputs[1]);
  SPI.transfer(shift_outputs[0]);
  PORTD |= 0x20;                                      //  Pull PD5 high to latch in spi transfers

  //  Set up the IR Switch
  irSwitch = 0;

  //  Defining steps for the stepper motors
  _steps_left[0] = 0x10;
  _steps_left[1] = 0x30;
  _steps_left[2] = 0x20;
  _steps_left[3] = 0x60;
  _steps_left[4] = 0x40;
  _steps_left[5] = 0xC0;
  _steps_left[6] = 0x80;
  _steps_left[7] = 0x90;
  _steps_left[8]  = 0x00;

  _steps_right[0] = 0x01;
  _steps_right[1] = 0x03;
  _steps_right[2] = 0x02;
  _steps_right[3] = 0x06;
  _steps_right[4] = 0x04;
  _steps_right[5] = 0x0C;
  _steps_right[6] = 0x08;
  _steps_right[7] = 0x09;
  _steps_right[8] = 0x00;

  //  Setup initial Stepper settings
  motor_speed[MOTOR_LEFT] = motor_speed[MOTOR_RIGHT] = motor_speed[MOTOR_GRIPPER] = move_speed;
  
  //  Set up the scheduler routine to run every 200uS, based off Timer4 interrupt
  cli();                                               //  disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;

  OCR4A = 48;                                          //  compare match register 64MHz/2048 = 31250hz
  TCCR4B = 0x06;
  TIMSK4 |= (1 << OCIE4A);                             //  enable Timer4 compare interrupt A
  sei();                                               //  enable all interrupts
  interrupts();
}

//  Sets up the appropriate interrupts for servo pwm
// void RaptorClass::startServoTimer(){
//   char oldSREG = SREG;        
//   noInterrupts();                                       //  Disable interrupts for 16 bit register access
//   TCCR1A = 0;                                           //  clear control register A 
//   TCCR1B = _BV(WGM13);                                  //  set mode 8: phase and frequency correct pwm, stop the timer
//   ICR1 = 20000;                                         //  ICR1 is TOP in p & f correct pwm mode
//   TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//   TCCR1B |= 0x02;                                       //  reset clock select register, and starts the clock
//   DDRB |= _BV(PORTB1);                                  //  sets data direction register for pwm output pin
//   TCCR1A |= _BV(COM1A1);                                //  activates the output pin
//   interrupts();                                         //  re-enable interrupts
//   SREG = oldSREG;
// }

//  Sets MUX values for reading from input IR and light sensors
void RaptorClass::setMux(uint8_t A, uint8_t B, uint8_t C){
  digitalWrite(MUX_A, A);
  digitalWrite(MUX_B, B);
  digitalWrite(MUX_C, C);
  delay(1);
}

//  Turns on the reflectance IR LEDs, and then reads from the appropriate line sensor
uint16_t RaptorClass::readSensorIR(uint8_t pin){
  int read = 0;
  onIR();
  read = analogRead(pin);
  offIR();
  return (uint16_t)read;
}

//  Wrapper function for motorRotate that sets the steps paramater to ULONG_MAX,
//  effectively setting for continuous location.
void RaptorClass::motorRotate(int16_t motor, int16_t direction, int16_t speed)
{
   motorRotate(motor, direction, speed, ULONG_MAX);
}

void RaptorClass::motorRotate(int16_t motor, int16_t direction, int16_t speed, int32_t steps)
{  
   motor_speed[motor] = speed; // speed in 1-100 precent
   
   // Populate the speed array with multiples of 200us waits between steps
   // Note: having 10 different waits allows finer grained control
   if(speed == 0){
      uint8_t oldSREG = SREG; cli();
      remainingSteps[motor] = 0; 
      isRunning[motor] = false;
      SREG = oldSREG; sei(); 
   }
   else{
      int16_t base_waits = 500/speed;
      int16_t remainder_waits = int((500.0/float(speed) - float(base_waits))*SPEED_ARRAY_LENGTH); 

      for(uint8_t i=0; i< (SPEED_ARRAY_LENGTH-remainder_waits); i++){
         speed_array[motor][i] = base_waits+1;
       }
      for(uint8_t i=(SPEED_ARRAY_LENGTH-remainder_waits); i<SPEED_ARRAY_LENGTH; i++){
         speed_array[motor][i] = base_waits;
       }
      
      uint8_t oldSREG = SREG; cli();
      speed_index[motor] = 0;
      speedCount[motor] = speed_array[motor][0];
      speedCounter[motor] = speedCount[motor];
      remainingSteps[motor] = steps;
      step_dir[motor] = direction;  
      isRunning[motor] = true;
      SREG = oldSREG; sei();
   }
   delay(1);
}

//  Turns on the IR Detection LEDs
void RaptorClass::onIR()
{
    irSwitch = 1;
    delay(1);             //  Give time for a scheduler cycle to run
}

//  Turns off the IR Detection LEDs
void RaptorClass::offIR()
{
    irSwitch = 0;
    delay(1);             //  Give time for a scheduler cycle to run
}

/***********************************************************************************
The Scheduler

Part of the original Sparki code, runs continuous handling of LEDs, IR Sensors, and
Stepper Motors.

Every 200uS (5,000 times a second), we update the 2 shift registers used to increase
the amount of outputs the processor has
***********************************************************************************/

static volatile uint8_t shift_old_0 = 0x00;
static volatile uint8_t shift_old_1 = 0x00;

//  Interrupt service routine that wraps a user defined function supplied by attachInterrupt
ISR(TIMER4_COMPA_vect)
{
  //  Clear the timer interrupt counter
  TCNT4=0;
  
  shift_old_0 = shift_outputs[0];
  shift_old_1 = shift_outputs[1];

	// Clear the shift register values so we can re-write them
  shift_outputs[0] = 0x00;
  shift_outputs[1] = 0x00;
  
  //  Update the RGB leds
  if(RGB_timer < RGB_vals[0]){ // update Red led
	shift_outputs[RGB_SHIFT] |= RGB_R;
  }
  if(RGB_timer < RGB_vals[1]){ // update Green led
	shift_outputs[RGB_SHIFT] |= RGB_G;
  }
  if(RGB_timer < RGB_vals[2]){ // update Blue led
	shift_outputs[RGB_SHIFT] |= RGB_B;
  }
  RGB_timer++;
  if(RGB_timer == 50){
  	RGB_timer = 0;
  }

  //  IR Detection Switch for Line Sensors
  if(irSwitch == 0){
  	shift_outputs[1] &= 0xF7;
  }
  else{
  	shift_outputs[1] |= 0x08;
  }
    
  //// Motor Control ////
  //   Determine what state the stepper coils are in:
  //   Check if finished stepping   
  //   speedCount determines the stepping frequency
  //   interrupt speed (5khz) divided by speedCounter equals stepping freq
  //   1khz is the maximum reliable frequency at 5v, so we use 5 as the top speed
  //   5,000hz/5 = 1000hz = micro-stepping frequency
	for(byte motor=0; motor<3; motor++){
		if( remainingSteps[motor] > 1 ){
			if(speedCounter[motor] == 0) { 
				step_index[motor] += step_dir[motor];
				remainingSteps[motor]--;
				speedCounter[motor] = speed_array[motor][speed_index[motor]];
				speed_index[motor]++;
				if(speed_index[motor] >= SPEED_ARRAY_LENGTH){
			      speed_index[motor] = 0;
			    }
			}
			else{
			   speedCounter[motor] = speedCounter[motor]-1;
			}
			
		}
		else {  //  If this was the last step
			remainingSteps[motor] = 0;  
			isRunning[motor] = false;
			step_index[motor] = 8;
			speedCounter[motor] = -1;
		}     
		
		//  Keep indicies from rolling over or under
		if( step_index[motor] >= 8){
			step_index[motor] = 0;
		}
		else if( step_index[motor] < 0){
			step_index[motor] = 7;
		}
		if(isRunning[motor] == false){
			step_index[motor] = 8;
		}
	}

  shift_outputs[0] |= _steps_right[step_index[MOTOR_RIGHT]];
  shift_outputs[0] |= _steps_left[step_index[MOTOR_GRIPPER]];
  shift_outputs[1] |= _steps_left[step_index[MOTOR_LEFT]];

  if( (shift_old_0 != shift_outputs[0]) || (shift_old_1 != shift_outputs[1]) ){

      PORTB |= 0x01; 

      //  Output values to shift registers
      PORTD &= ~(1<<5);                       // Pull PD5 (shift-register latch) low
      SPI.transfer(shift_outputs[1]);
      SPI.transfer(shift_outputs[0]);
      PORTD |= (1<<5);                        //  Pull PD5 (shift-register latch) high         
  }  
}