#include <Servo.h>

#define LED_P 3
#define LED_R 5
#define LED_N 6
#define LED_D 9

#define BUTTON_UP 2
#define BUTTON_DOWN 7
#define BUTTON_PRESSED 0

#define SERVO_PIN 19

#define UP() (digitalRead(BUTTON_UP) == BUTTON_PRESSED)
#define DOWN() (digitalRead(BUTTON_DOWN) == BUTTON_PRESSED)

#define IDLE_PWM 20

enum State {
  _P = 0,
  _R,
  _N,
  _D,
  _LAST
};

unsigned char LEDS[_LAST];
unsigned char SERVO_POS[_LAST];

unsigned char state = _P;
unsigned char pressed = 0;
Servo servo;

void setup() {
  LEDS[_P] = LED_P;
  LEDS[_R] = LED_R;
  LEDS[_N] = LED_N;
  LEDS[_D] = LED_D;

  for(auto i = 0; i< _LAST; i++) {
    pinMode(LEDS[i], OUTPUT);
    SERVO_POS[i] = 180/_LAST*i;
  }

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  servo.attach(SERVO_PIN);
}

void updateLeds() {
  digitalWrite(LEDS[state], HIGH);
  
  for(auto i = 0; i< _LAST; i++)
    if(i != state)
      analogWrite(LEDS[i], IDLE_PWM);
}

void checkButtons() {
  if(pressed) {
    if(!UP() && !DOWN())
      pressed = 0;
    return;
  }

  if(UP() && (state+1) < _LAST) {
    ++state;
    pressed = 1;
  }
  if(DOWN() && (state-1) >= 0) {
    --state;
    pressed = 1;
  }
}

void updateServo() {
  static char last = -1;
  if(last != state) {
    servo.write(SERVO_POS[state]);
    last = state;
  }
}

unsigned char i = 0;

void loop() {
  checkButtons();
  updateLeds();
  updateServo();
  delay(100);
}

