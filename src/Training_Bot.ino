/* Authors:
Carlos Giron
*/

/* NOTES:
* For using tele-op, change func in main to "control_process()" 
* Need to clean up this code by writing a header file for functions and testing and tele-op stuff
*/

// ---------------------
// Preprocessor
// ---------------------

#include <stdint.h>

#define FR_EN 		3
#define FR_FORWARD  4
#define FR_BACKWARD 5

#define FL_EN		6
#define FL_FORWARD 	7
#define FL_BACKWARD 8

#define BL_EN		6
#define BL_FORWARD  7
#define BL_BACKWARD 8

#define BR_EN		3
#define BR_FORWARD	4
#define BR_BACKWARD	5

#define ZERO  126
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

  void drive(uint8_t duty, bool isForward) {
    digitalWrite(pinF,  isForward);
    digitalWrite(pinB, !isForward);
    analogWrite(pinEn, duty);
  }
  
  void tele_drive(uint8_t input) {
    bool    isForward;
    uint8_t duty;

    if (input < ZERO) {
      duty = (ZERO - input) << 1;
      isForward = false;
    }
    else if (input >= ZERO) {
      duty = (input - ZERO) << 1;
      isForward = true;
    }
    drive(duty, isForward);
  }
};


// ---------------------
// setting up wheels and funcs for tele-op control or testing 
// ---------------------

Wheel frontLeft(FL_EN, FL_FORWARD, FL_BACKWARD);
Wheel frontRight(FR_EN, FR_FORWARD, FR_BACKWARD);
Wheel backLeft(BL_EN, BL_FORWARD, BL_BACKWARD);
Wheel backRight(BR_EN, BR_FORWARD, BR_BACKWARD);

Wheel wheelAr[4] = {frontLeft, backLeft,frontRight, backRight};

void control_process(void) {
  if (Serial.available() > 0) {

    byte data[2]; // Left, Right 
    Serial.readBytes(data, 2);

    for (int i=0; i<2; i++) {
      wheelAr[i*2].tele_drive(data[i]);
      wheelAr[i*2+1].tele_drive(data[i]);
    }
  }
}

bool forward = true;
void test_process(void) {
  for (int i=0; i<2; i++) {
    wheelAr[i].test_drive(150, forward);
    wheelAr[i+1].test_drive(150, !forward);
  }
  delay(2500);
  forward = !forward;
}

// ---------------------
// Setup & Loop
// ---------------------

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  control_process();
}
