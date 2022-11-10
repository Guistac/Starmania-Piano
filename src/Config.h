
//——————————————————————————————————————————————————————————————————————————————————————
//—————————————————————————— MECANUM ROBOT CONFIGURATION FILE ——————————————————————————
//——————————————————————————————————————————————————————————————————————————————————————

/*——————— Upload Instruction:

  Before uploading set the following options in the arduino menu "Tools"
  Select Tools -> Board -> Teensyduino -> Teensy 4.1
  Select Tools -> CPU Speed -> 600 Mhz
  Select Tools -> Port -> <The Currently Connected Teensy Microcontroller>

  To Upload press the arrow button or use the menu Sketch -> Upload

  To Debug the robot:
  -make sure it is connected over usb
  -make sure it is selected in Tools -> Port
  -remove the "//" before the DEBUG options at the bottom of this file and upload the code to enable the debug messages
  -open the Serial Monitor in Tools -> Serial Monitor to display debug messages

*/

//——————————————— SERVO PIN ASSIGNEMENT ———————————————

#define BACK_LEFT_SERVO_DIRECTION_PIN 3 //brow cable on pin 27 of kinco io connector, direct connection to teensy
#define BACK_LEFT_SERVO_PULSE_PIN 4     //green cable on pin 31 of kinco io connector, direct connection to teensy
#define BACK_LEFT_SERVO_ENABLED_PIN 28  //red-white cable on pin 11 of kinco io connector, direct connection to teensy pin with pullup resistor enabled
#define BACK_LEFT_SERVO_FAULT_PIN 32    //orange cable on pin 20 of kinco io connector, direct connection to teensy pin with pullup resistor enabled

#define BACK_RIGHT_SERVO_DIRECTION_PIN 5
#define BACK_RIGHT_SERVO_PULSE_PIN 6
#define BACK_RIGHT_SERVO_ENABLED_PIN 29
#define BACK_RIGHT_SERVO_FAULT_PIN 33

#define FRONT_LEFT_SERVO_DIRECTION_PIN 7
#define FRONT_LEFT_SERVO_PULSE_PIN 8
#define FRONT_LEFT_SERVO_ENABLED_PIN 30
#define FRONT_LEFT_SERVO_FAULT_PIN 34

#define FRONT_RIGHT_SERVO_DIRECTION_PIN 9
#define FRONT_RIGHT_SERVO_PULSE_PIN 10
#define FRONT_RIGHT_SERVO_ENABLED_PIN 31
#define FRONT_RIGHT_SERVO_FAULT_PIN 35

#define ENABLE_ALL_SERVO_POWER_STAGES_PIN 11  //drives 24v relay, signal is interrupted by EStop and manual brake release
#define RESET_ALL_SERVO_FAULTS_PIN 12         //drives 24v relay




//——————————————— RADIO REMOTE PIN ASSIGNEMENT ———————————————

#define NEGATIVE_X_AXIS_PIN 21 //A0
#define POSITIVE_X_AXIS_PIN 20 //A1
#define NEGATIVE_Y_AXIS_PIN 17 //A2
#define POSITIVE_Y_AXIS_PIN 16 //A3
#define NEGATIVE_Z_AXIS_PIN 14 //A6
#define POSITIVE_Z_AXIS_PIN 15 //A7

#define MODE_ABSOLUTE_PIN 37
#define MODE_RELATIVE_PIN 40

#define FAST_SPEED_MODE_PIN 38
#define SLOW_SPEED_MODE_PIN 41

#define EMERGENCY_STOP_PIN 23
//#define REARM_BUTTON_PIN 27

#define STATE_FEEDBACK_PIN 19 //enabled / not enabled
#define MODE_FEEDBACK_PIN 18 //mode absolute / relative

//——————————————— LED PIN ASSIGNEMENT ———————————————

#define INTERNAL_LED_PIN 13
#define GREEN_LED_PIN 25
#define RED_LED_PIN 26

//——————————————— CONTROL LOGIC ———————————————

#define WHEEL_DIAMETER_MM 152.4

#define FRONT_LEFT_WHEEL_POSITION_MM    Vector2{.x = -700.0, .y = 700.0}
#define FRONT_RIGHT_WHEEL_POSITION_MM   Vector2{.x = 700.0, .y = 700.0}
#define BACK_LEFT_WHEEL_POSITION_MM     Vector2{.x = -700.0, .y = -700.0}
#define BACK_RIGHT_WHEEL_POSITION_MM    Vector2{.x = 700.0, .y = -700.0}

#define FAST_TRANSLATION_VELOCITY_LIMIT_MMPS 500.0  //translation velocity limit in millimeters per second
#define FAST_TRANSLATION_ACCELERATION_MMPS2 150.0   //fixed translation acceleration in millimeters per second squared
#define FAST_ROTATION_VELOCITY_LIMIT_DEGPS 15.0     //rotation velocity limit in degrees per second 
#define FAST_ROTATION_ACCELERATION_DEGPS2 5.0      //fixed rotation acceleration in degrees per second squared

#define SLOW_TRANSLATION_VELOCITY_LIMIT_MMPS 250.0  //translation velocity limit in millimeters per second
#define SLOW_TRANSLATION_ACCELERATION_MMPS2 150.0   //fixed translation acceleration in millimeters per second squared
#define SLOW_ROTATION_VELOCITY_LIMIT_DEGPS 7.5     //rotation velocity limit in degrees per second
#define SLOW_ROTATION_ACCELERATION_DEGPS2 5.0     //fixed rotation acceleration in degrees per second squared

#define HEADING_CORRECTION_FACTOR 1.176 //mesuré au milieu du grand atelier

//wheel friction vectors for mecanum wheels mounted in an X formation
//motor inversions are compensated with vector directions
//ONLY CHANGE IF WHEEL OR MOTOR CONFIGURATION CHANGES
#define FRONT_LEFT_WHEEL_FRICTION_VECTOR_MMPREV     Vector2{.x = -sin(45.0) * WHEEL_CIRCUMFERENCE_MM,   .y = -sin(45.0) * WHEEL_CIRCUMFERENCE_MM} 
#define FRONT_RIGHT_WHEEL_FRICTION_VECTOR_MMPREV    Vector2{.x = -sin(45.0) * WHEEL_CIRCUMFERENCE_MM,   .y = sin(45.0) * WHEEL_CIRCUMFERENCE_MM}
#define BACK_LEFT_WHEEL_FRICTION_VECTOR_MMPREV      Vector2{.x = sin(45.0) * WHEEL_CIRCUMFERENCE_MM,    .y = -sin(45.0) * WHEEL_CIRCUMFERENCE_MM}
#define BACK_RIGHT_WHEEL_FRICTION_VECTOR_MMPREV     Vector2{.x = sin(45.0) * WHEEL_CIRCUMFERENCE_MM,    .y = sin(45.0) * WHEEL_CIRCUMFERENCE_MM}

#define WHEEL_COUNT 4                                 //DO NOT CHANGE
#define WHEEL_CIRCUMFERENCE_MM PI * WHEEL_DIAMETER_MM //DO NOT CHANGE
#define UPDATE_FREQUENCY 500.0;                       //DO NOT CHANGE

//——————————————— SERVO MOTORS ———————————————

#define PULSES_PER_WHEEL_REVOLUTION 6400  //is equal to pulses per motor revolution * gear reduction 
#define MAX_MOTOR_VELOCITY_RPS 3.333      //max velocity for the wheels
#define ACCELERATION_RPS2 10.0

#define MAX_PULSE_FREQUENCY 500000.0
#define MIN_PULSE_FREQUENCY 10.0


//——————————————— DEBUGGING ———————————————
//uncomment these lines to debug the the corresponding section in the serial monitor

//#define DEBUG_GENERAL
//#define DEBUG_REMOTE
//#define DEBUG_CONTROLLOGIC
//#define DEBUG_STATECHANGES
