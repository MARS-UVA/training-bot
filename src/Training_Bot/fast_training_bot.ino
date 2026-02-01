/* Authors:
Carlos Giron
*/

/* NOTES:
* Change code to only use R and L sides for less function calls
* Use port manipulation for faster digital writes 
* Up baud rate to 250,000
* use checks to return early if a packet has the same data as previous
*/

// ---------------------
// Preprocessor
// ---------------------

#include <stdint.h>

// Actually R and L are reverse on hardware

#define L_EN 		    3
#define L_FORWARD   4
#define L_BACKWARD  5

#define R_EN		    6
#define R_FORWARD 	7
#define R_BACKWARD  8

#define ZERO  126
#define SPEED 255

#define L_FDIR(F)    (PORTD |= (1 << F))
#define L_BDIR(B)    (PORTD |= (1 << B))

#define R_FDIR(F)    (PORTD |= (1 << F))
#define R_BDIR(B)    (PORTB |= (1 << (B-8)))

uint8_t lastDuty = 0;
bool    lastDir  = false;

// ---------------------
// Class Definition (move to a header file)
// ---------------------

class Wheels {
  private:
  	uint8_t pinEn;
    uint8_t pinF ;
  	uint8_t pinB ;
    char    side ;
  
  public:
  Wheels(uint8_t pE, uint8_t pF, uint8_t pB, char s) {
      pinEn = pE;
      pinF  = pF;
      pinB  = pB;
      side  = s ;
    
      pinMode(pinEn, OUTPUT);
  	  pinMode(pinF , OUTPUT);
      pinMode(pinB , OUTPUT);

      digitalWrite(pinF, LOW);
      digitalWrite(pinB, LOW);
      analogWrite(pinEn, 0);
	}

  void drive(uint8_t duty, bool isForward) {
    if (duty == lastDuty && isForward == lastDir) return;
    digitalWrite(pinF,  isForward);
    digitalWrite(pinB, !isForward);
    analogWrite(pinEn, duty);
  }

  void drive2(uint8_t duty, bool isForward) {
    if (duty == lastDuty && isForward == lastDir) return;

    if (isForward) {
      if (side == 'l') {
        L_FDIR(pinF);
      } else {
        R_FDIR(pinF);
      }
    } else {
      if (side == 'l') {
        L_BDIR(pinB);
      } else {
        R_BDIR(pinB);
      }
    }
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

Wheels Left(L_EN, L_FORWARD, L_BACKWARD, 'l');
Wheels Right(R_EN, R_FORWARD, R_BACKWARD, 'r');


Wheels wheelAr[2] = {Left, Right};

void control_process(void) {
    byte data[3]; // 255, Left, Right 
    Serial.readBytes(data, 3);
    if (data[0] == 255) {
      for (int i=0; i<2; i++) {
        wheelAr[i].tele_drive(data[i+1]);
      }
    }
}

void control_process2(void) {
  static byte data[3];
  static uint8_t index = 0;

  while (Serial.available()) {
    data[index++] = Serial.read();

    if (index == 3) {
      PORTD &= 0b01001111;
      PORTB &= 0b11111110;
      if (data[0] == 255) {
        for (int i=0; i<2; i++) {
          wheelAr[i].tele_drive(data[i+1]);
        }
      }
      index = 0;
    }
  }
}


bool forward = true;
void test_process(void) {
  PORTD &= 0b01001111;
  PORTB &= 0b11111110;
  for (int i=0; i<2; i++) {
    wheelAr[i].drive2(150, forward);
  }
  delay(2500);
  forward = !forward;
}

void portManTest() {
  PORTB &= 0b11111110;
  L_FDIR(L_FORWARD);
  analogWrite(L_EN, 150);
}

// ---------------------
// Setup & Loop
// ---------------------

void setup()
{
  Serial.begin(250000);
}

void loop()
{
  control_process2();
}