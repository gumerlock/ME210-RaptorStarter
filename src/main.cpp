#include <Raptor.h>
#include <SPI.h>

#include <Chrono.h>


/*---------------Module Defines-----------------------------*/

#define LIGHT_THRESHOLD         0   // *Choose your own thresholds*
                                    // (this will be smaller at night)
#define LINE_THRESHOLD          0   // *Choose your own thresholds*

#define LED_TIME_INTERVAL       1000
#define MOTOR_TIME_INTERVAL     2000

#define HALF_SPEED              50

#define TIMER_0            0
/*---------------Module Function Prototypes-----------------*/
void checkGlobalEvents(void);
void handleMoveForward(void);
void handleMoveBackward(void);
uint8_t TestForKey(void);
void RespToKey(void);
uint8_t TestForLightOn(void);
void RespToLightOn(void);
uint8_t TestForLightOff(void);
void RespToLightOff(void);
uint8_t TestForFence(void);
void RespToFence(void);
uint8_t TestLedTimerExpired(void);
void RespLedTimerExpired(void);

/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_BACKWARD
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
static Chrono chrTimer0;  // Chrono defaults to MILLIS
uint8_t isLEDOn;

/*---------------Raptor Main Functions----------------*/

void setup() {
  // put your setup code here, to run once:
  
  /* Open the serial port for communication using the Serial
     C++ class. On the Leonardo, you must explicitly wait for
   the class to report ready before commanding a println.
  */
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello, world!");
  
  state = STATE_MOVE_FORWARD;
  isLEDOn = false;
}

void loop() {
  //  put your main code here, to run repeatedly:
  checkGlobalEvents();
  switch (state) {
    case STATE_MOVE_FORWARD:
      handleMoveForward();
      break;
    case STATE_MOVE_BACKWARD:
      handleMoveBackward();
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

/*----------------Module Functions--------------------------*/


void handleMoveForward(void) {
  raptor.LeftMtrSpeed(HALF_SPEED);
  raptor.RightMtrSpeed(HALF_SPEED);
  delay(MOTOR_TIME_INTERVAL);
  state = STATE_MOVE_BACKWARD;
}

void handleMoveBackward(void) {
  raptor.LeftMtrSpeed(-1*HALF_SPEED);
  raptor.RightMtrSpeed(-1*HALF_SPEED);
  delay(MOTOR_TIME_INTERVAL);
  state = STATE_MOVE_FORWARD;
}

uint8_t TestLedTimerExpired(void) {
  // Using .hasPassed with interval and restart flag means we don't
  // need to reset it in the response routine, which makes the _actual_
  // intervals much closer to LED_TIME_INTERVAL
  return (uint8_t) chrTimer0.hasPassed(LED_TIME_INTERVAL, true);
}

void RespLedTimerExpired(void) {
  if (isLEDOn) {
    isLEDOn = false;
    raptor.RGB(RGB_OFF);
  } else {
    isLEDOn = true;
    raptor.RGB(RGB_WHITE);
  }
}

uint8_t TestForKey(void) {
  uint8_t KeyEventOccurred;
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}

void RespToKey(void) {
  uint8_t theKey;
  theKey = Serial.read();
  Serial.print(theKey);
  Serial.print(", ASCII=");
  Serial.println(theKey, HEX);
}

void checkGlobalEvents(void) {
  if (TestLedTimerExpired()) RespLedTimerExpired();
  if (TestForKey()) RespToKey();
}

uint8_t TestForLightOn(void) {
  // To be written in Part 2
  return 0;
}

void RespToLightOn(void) {
  // To be written in Part 2
}

uint8_t TestForLightOff(void) {
  // To be written in Part 2
  return 0;
}

void RespToLightOff(void) {
  // To be written in Part 2
}

uint8_t TestForFence(void) {
  // To be written in Part 2
  return 0;
}

void RespToFence(void) {
  // To be written in Part 2
}