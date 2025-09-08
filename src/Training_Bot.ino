/* Authors:
Carlos Giron
*/

/* NOTES:
* PINS 0 AND 1 ARE RESERVED FOR UART COMMS WITH R_PI!!!!!
* coding conventions? snake_case for funcs or pascal? need ricP
* freeRTOS for reading input stream in a seperate thread? Can't simulate here
*/

// ---------------------
// Preprocessor
// ---------------------

#include <stdint.h>


#define FR_EN 		  3
#define FR_FORWARD  4
#define FR_BACKWARD 5

#define FL_EN		    6
#define FL_FORWARD 	7
#define FL_BACKWARD 13

#define BL_EN		    9
#define BL_FORWARD  10
#define BL_BACKWARD 8

#define BR_EN		    11
#define BR_FORWARD	2
#define BR_BACKWARD	12

#define SPEED 255

// ---------------------
// Class Definition (move to a header file)
// ---------------------

class Wheel {
  private:
  	uint8_t pinEn;
    uint8_t pinF ;
  	uint8_t pinB ;
  
  public:
  Wheel(uint8_t pE, uint8_t pF, uint8_t pB) {
      pinEn = pE;
      pinF  = pF;
      pinB  = pB;
    
      pinMode(pinEn, OUTPUT);
  	  pinMode(pinF , OUTPUT);
      pinMode(pinB , OUTPUT);
	}
  
  void drive(uint8_t input) {
    bool    direction;
    uint8_t duty;

    if (input < 127) {
      duty = (127 - input) << 1;
      direction = false;
    }
    else if (input >= 127) {
      duty = (input - 127) << 1;
      direction = true;
    }
    digitalWrite(pinF,  direction);
    digitalWrite(pinB, !direction);
    analogWrite(pinEn, duty);
  } 
};

// ---------------------
// Setup & Loop
// ---------------------

Wheel frontLeft(FL_EN, FL_FORWARD, FL_BACKWARD);
Wheel frontRight(FR_EN, FR_FORWARD, FR_BACKWARD);
Wheel backLeft(BL_EN, BL_FORWARD, BL_BACKWARD);
Wheel backRight(BR_EN, BR_FORWARD, BR_BACKWARD);

Wheel wheelAr[4] = {frontLeft, frontRight, backLeft, backRight};

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if (Serial.available() > 0) {

    byte data[4]; // FL, FR, BL, BR
    Serial.readBytes(data, 4);

    for (int i=0; i<4; i++) {
      wheelAr[i].drive(data[i]);
    }
  }
}