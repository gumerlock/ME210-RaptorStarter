#ifndef Raptor_h
#define Raptor_h

#include <inttypes.h>

/*----------------------------- Library Constants ---------------------------*/
/* Note: Some lines have been commented out to preserve the original Sparki
 * pin mappings for potential future use
 */


#define SHIFTREG_LATCH      TXLED0   // PD5
#define STATUS_LED          13        
#define BUZZER              11       // PB7
#define BUZZER_FREQ         2800
// #define ULTRASONIC_ECHO     5        // PC6
// #define ULTRASONIC_TRIG     10       // PB6
#define IR_RECEIVE          7        // PE6
#define IR_SEND             6        // PD7

// #define SERVO               9        // 
// #define SERVO_LEFT          -80
// #define SERVO_CENTER        0
// #define SERVO_RIGHT         80
// #define MAX_SERVO_OFFSET    30

// defining the MUX pins
#define MUX_ANALOG		A2 // PF5
#define MUX_A  	        A3 // PF4
#define MUX_B      		A4 // PF1
#define MUX_C        	A5 // PF0

// defining the IR line sensor pins (on the multiplexer)
#define IR_EDGE_RIGHT      LOW, HIGH, LOW	// Mux A2     
#define IR_LINE_RIGHT      HIGH, LOW, LOW	// Mux A1     
#define IR_LINE_CENTER     LOW, LOW, LOW	// Mux A0      
#define IR_LINE_LEFT       HIGH, HIGH, LOW	// Mux A3 
#define IR_EDGE_LEFT       HIGH, LOW, HIGH // Mux A5       

// defining the light sensors
// #define LIGHT_RIGHT         HIGH, HIGH, HIGH // Mux A7
#define LIGHT_CENTER        LOW, HIGH, HIGH	 // Mux A6
// #define LIGHT_LEFT          LOW, LOW, HIGH	 // Mux A4

// define the shift registers pin output values for the RGB arrays
#define RGB_R 0x01 // pin value of the Red LED on the RGB on the shift register
#define RGB_G 0x02 // pin value of the Green LED on the RGB on the shift register
#define RGB_B 0x04 // pin value of the Blue LED on the RGB on the shift register
#define RGB_SHIFT 1 // which shift register the RGB is on (starts at 0)

#define RGB_RED     100, 0,   0
#define RGB_ORANGE  90,  100, 0
#define RGB_YELLOW  60,  100, 0
#define RGB_GREEN   0,   100, 0
#define RGB_BLUE    0,   0,   100
#define RGB_PINK    90,  0,   100
#define RGB_INDIGO  20,  0,   100
#define RGB_VIOLET  60,  0,   100
#define RGB_WHITE   60,  100, 90
#define RGB_OFF     0,   0,   0

// properties about the robot in cm
// #define STEPS_PER_REV 4096          // steps for wheels to revolve 360 degrees
// #define WHEEL_DIAMETER_CM 5.00
// #define TRACK_WIDTH_CM 8.51         //tire seperation in cm  
// const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * PI;
// const float CM_PER_STEP            = WHEEL_CIRCUMFERENCE_CM / STEPS_PER_REV;
// const float CM_PER_DEGREE          = WHEEL_CIRCUMFERENCE_CM / 360.0;     // wheel movement per degree rotation of robot 
// const float STEPS_PER_CM             = STEPS_PER_REV/(WHEEL_DIAMETER_CM*PI);
// const float STEPS_PER_ROTATION     = (TRACK_WIDTH_CM / WHEEL_DIAMETER_CM) * STEPS_PER_REV ;  // robot rotation
// const float STEPS_PER_DEGREE       = STEPS_PER_ROTATION / 360.0;         // robot rotation
// const float STEPS_PER_DEGREE         = (TRACK_WIDTH_CM / WHEEL_DIAMETER_CM) * STEPS_PER_REV / 360.0;
// const float STEPS_PER_ARM_CM         = 650;
// #define DISTANCE_TIME_COSNTANT 222.222222
// #define DEGREES_TIME_COSNTANT  21.388888

#define SPEED_ARRAY_LENGTH 10  // uses an array to determine speed. 
                               // increase this number (<255) to increase precision of speed control

// defines for left and right motors
#define MOTOR_LEFT    0
#define MOTOR_RIGHT   1
#define MOTOR_GRIPPER 2

// defines for direction
#define DIR_CCW -1
#define DIR_CW   1

typedef enum {
   OK_OPERATION, ERR_BADINPUT
} RaptorReturn_t;

class RaptorClass {
  public:
    RaptorClass();

    RaptorReturn_t LeftMtrSpeed(int8_t newSpeed);
    RaptorReturn_t RightMtrSpeed(int8_t newSpeed);

    // // Servo Functions
    // RaptorReturn_t SetServoDeg(char deg);

    // Light level sensors
    uint16_t LightLevel();

    // Infrared reflectance sensors
    uint16_t EdgeRight();
    uint16_t LineRight();
    uint16_t LineCenter();
    uint16_t LineLeft();  
    uint16_t EdgeLeft();
    uint8_t ReadTriggers(uint16_t threshold);

    void RGB(uint8_t R, uint8_t G, uint8_t B);
    void Beep(uint16_t, uint32_t);


    void SET_SHARED_BYTE_TO(uint8_t);
    uint8_t GET_SHARED_BYTE(void);
    void SET_SHARED_WORD_TO(uint16_t);
    uint16_t GET_SHARED_WORD(void);

  private:    
    void begin();
    // void startServoTimer();
    void setMux(uint8_t, uint8_t, uint8_t);
    uint16_t readSensorIR(uint8_t);

    void motorRotate( int16_t motor, int16_t direction,  int16_t speed);
    void motorRotate( int16_t motor, int16_t direction,  int16_t speed, int32_t steps);
    void motorStop(int16_t motor);

    void onIR();
    void offIR();
};

extern RaptorClass raptor;

#endif