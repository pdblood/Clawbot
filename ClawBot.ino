/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        7  //14    
#define PS2_CMD        6  //15
#define PS2_SEL        5 //16
#define PS2_CLK        4  //17
#define MOTOR_PWM      12

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false


PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you conect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
int error = 0; 
byte type = 0;
byte vibrate = 0;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Setup motors
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorClaw = AFMS.getMotor(2);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);
Adafruit_DCMotor *motorArm = AFMS.getMotor(4);


//Servo motorClaw;  // Additional motors are setup using the Servo Library 
                 // and the VEX Motor Controller 29 PWM to MotorOut
                 // device.

int motorRightSpeed = 0;
int motorLeftSpeed = 0;

int rightStickY = 0;
int leftStickY = 0;

void setup() {
  // Serial.begin(57600);
 delay(3000);
 //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

 
  /* if(error == 0){ */
  /*   Serial.print("Found Controller, configured successful "); */
  /*   Serial.print("pressures = "); */
  /* 	if (pressures) */
  /* 	  Serial.println("true "); */
  /* 	else */
  /* 	  Serial.println("false"); */
  /* 	Serial.print("rumble = "); */
  /* 	if (rumble) */
  /* 	  Serial.println("true)"); */
  /* 	else */
  /* 	  Serial.println("false"); */
  /*   Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;"); */
  /*   Serial.println("holding L1 or R1 will print out the analog stick values."); */
  /*   Serial.println("Note: Go to www.billporter.info for updates and to report bugs."); */
  /* }   */
  /* else if(error == 1) */
  /*   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips"); */
   
  /* else if(error == 2) */
  /*   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips"); */

  /* else if(error == 3) */
  /*   Serial.println("Controller refusing to enter Pressures mode, may not support it. "); */
   
   //Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType(); 
  /* switch(type) { */
  /*   case 0: */
  /*     Serial.print("Unknown Controller type found "); */
  /*     break; */
  /*   case 1: */
  /*     Serial.print("DualShock Controller found "); */
  /*     break; */
  /*   case 2: */
  /*     Serial.print("GuitarHero Controller found "); */
  /*     break; */
  /*   case 3: */
  /*     Serial.print("Wireless Sony DualShock Controller found "); */
  /*     break; */
  /*  } */

  
  //  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorRight->setSpeed(0);
  motorLeft->setSpeed(0);
  motorArm->setSpeed(255);
  motorClaw->setSpeed(200);
 
  //motorClaw.attach(MOTOR_PWM);
  //vexMotorWrite(motorClaw, 0);  

  motorRight->run(FORWARD);
  motorLeft->run(FORWARD);
  motorArm->run(FORWARD);
  motorClaw->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
  motorArm->run(RELEASE);
  motorClaw->run(RELEASE);
}

void loop() {
  uint8_t i;


  if(error == 1) //skip loop if no controller found
    return; 
   
  //   motorClaw->setSpeed(255);
  
   ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
    
   /* if(ps2x.Button(PSB_START))                   //will be TRUE as long as button is pressed */
   /*   Serial.println("Start is being held"); */
   /* if(ps2x.Button(PSB_SELECT)) */
   /*   Serial.println("Select is being held"); */
         
         
   /* if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed */
   /*   Serial.print("Up held this hard: "); */
   /*   Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC); */
   /* } */
   /* if(ps2x.Button(PSB_PAD_RIGHT)){ */
   /*   Serial.print("Right held this hard: "); */
   /*   Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC); */
   /* } */
   /* if(ps2x.Button(PSB_PAD_LEFT)){ */
   /*   Serial.print("LEFT held this hard: "); */
   /*   Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC); */
   /* } */
   /* if(ps2x.Button(PSB_PAD_DOWN)){ */
   /*   Serial.print("DOWN held this hard: "); */
   /*   Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC); */
   /* }    */
  
    
   /* vibrate = ps2x.Analog(PSAB_BLUE);        //this will set the large motor vibrate speed based on  */
   /* //how hard you press the blue (X) button     */
    
   /* if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on) */
   /*   { */
     
       
         
   /*     if(ps2x.Button(PSB_L3)) */
   /*       Serial.println("L3 pressed"); */
   /*     if(ps2x.Button(PSB_R3)) */
   /*       Serial.println("R3 pressed"); */
   /*     if(ps2x.Button(PSB_L2)) */
   /*       Serial.println("L2 pressed"); */
   /*     if(ps2x.Button(PSB_R2)) */
   /*       Serial.println("R2 pressed"); */
   /*     if(ps2x.Button(PSB_GREEN)) */
   /*       Serial.println("Triangle pressed"); */
         
   /*   }    */
         
    
   /* if(ps2x.ButtonPressed(PSB_RED))             //will be TRUE if button was JUST pressed */
   /*   Serial.println("Circle just pressed"); */
         
   /* if(ps2x.ButtonReleased(PSB_PINK))             //will be TRUE if button was JUST released */
   /*   Serial.println("Square just released");      */
    
   /* if(ps2x.NewButtonState(PSB_BLUE))            //will be TRUE if button was JUST pressed OR released */
   /*   Serial.println("X just changed");     */

   rightStickY = ps2x.Analog(PSS_RY);
   leftStickY = ps2x.Analog(PSS_LY);

   if (rightStickY <= 100) {
     motorRightSpeed = map(rightStickY,0,100,255,0);
     motorRight->setSpeed(motorRightSpeed);
     motorRight->run(FORWARD);
   }
   else if (rightStickY >= 155) {
     motorRightSpeed = map(rightStickY,155,255,0,255);
     motorRight->setSpeed(motorRightSpeed);
     motorRight->run(BACKWARD);
   }
   else {
     motorRight->setSpeed(0);
   }

   if (leftStickY <= 100) {
     motorLeftSpeed = map(leftStickY,0,100,255,0);
     motorLeft->setSpeed(motorLeftSpeed);
     motorLeft->run(FORWARD);
   }
   else if (leftStickY >= 155) {
     motorLeftSpeed = map(leftStickY,155,255,0,255);
     motorLeft->setSpeed(motorLeftSpeed);
     motorLeft->run(BACKWARD);
   }
   else {
     motorLeft->setSpeed(0);
   }

   /* Serial.print("Stick Values:"); */
   /* Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX */
   /* Serial.print(","); */
   /* Serial.print(ps2x.Analog(PSS_LX), DEC); */
   /* Serial.print(","); */
   /* Serial.print(ps2x.Analog(PSS_RY), DEC); */
   /* Serial.print(","); */
   /* Serial.println(ps2x.Analog(PSS_RX), DEC); */
    
    
   if(ps2x.Button(PSB_L1)) // print stick values if either is TRUE
     {

       motorArm->run(FORWARD);
     }
   else if (ps2x.Button(PSB_L2)){
     motorArm->run(BACKWARD);
   }
   else {
     motorArm->run(RELEASE);
   }	  
   
   if(ps2x.Button(PSB_R1)) // print stick values if either is TRUE
     {
       motorClaw->run(FORWARD);
       //       vexMotorWrite(motorClaw, 255);
     }
   else if (ps2x.Button(PSB_R2)){
     motorClaw->run(BACKWARD);
     //       vexMotorWrite(motorClaw, -255);
   }
   else {
     motorClaw->run(RELEASE);
     //       vexMotorWrite(motorClaw, 0);		
   }	  


  //
  //Serial.print("open");
  //motorClaw->run(FORWARD);
  //delay(500);
  //Serial.print("close");
  //motorClaw->run(BACKWARD);
  //delay(500);
  //Serial.print("tech");
  //motorClaw->run(RELEASE);
  //delay(1000);

 delay(50); 
}

void vexMotorWrite(Servo motorObj, int speed)
{ 
  motorObj.write(map(speed, -255, 255, 1000, 2000));
}
