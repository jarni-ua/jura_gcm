#include <Servo.h>
#include <EEPROM.h>

#define LED_P 3
#define LED_R 5
#define LED_N 6
#define LED_D 9

#define BUTTON_UP 2
#define BUTTON_DOWN 7
#define BUTTON_BTN 10
#define BUTTON_PRESSED 0

#define SERVO_PIN 19

#define UP() (digitalRead(BUTTON_UP) == BUTTON_PRESSED)
#define DOWN() (digitalRead(BUTTON_DOWN) == BUTTON_PRESSED)
#define BTN() (digitalRead(BUTTON_BTN) == BUTTON_PRESSED)

#define IDLE_PWM 20

enum State {
  _P = 0,
  _R,
  _N,
  _D,
  _LAST
};

struct EEP {
  unsigned char SERVO_POS[_LAST];
  unsigned char IDLE_LED;
  unsigned char CRC;

  void init() {
    for(auto i = 0; i< _LAST; i++) {
      SERVO_POS[i] = 58+i*5;
    }
    IDLE_LED = 5;
    CRC = 0;
    for(unsigned char *p = (unsigned char *)this; p < &CRC; ++p)
      CRC += *p;
  }

  EEP() {
    init();
  }

  void read() {
    EEPROM.get(0, *this);

    unsigned char tmp = 0;
    for(unsigned char *p = (unsigned char *)this; p < &CRC; ++p)
      tmp += *p;
    if(tmp != CRC)
      init();
  }

  void write() {
    CRC = 0;
    for(unsigned char *p = (unsigned char *)this; p < &CRC; ++p)
      CRC += *p;
    EEPROM.put(0, *this);
  }
} eeprom;

unsigned char LEDS[_LAST];
unsigned char *SERVO_POS = (unsigned char *)&eeprom.SERVO_POS;

unsigned char state = _P;
unsigned char pressed = 0;
unsigned char tune = 0;

Servo servo;

void setup() {
  // load values from EERPOM
  eeprom.read();

  LEDS[_P] = LED_P;
  LEDS[_R] = LED_R;
  LEDS[_N] = LED_N;
  LEDS[_D] = LED_D;

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_BTN, INPUT_PULLUP);

  for(auto i = 0; i< _LAST; i++) {
    pinMode(LEDS[i], OUTPUT);
  }

  servo.attach(SERVO_PIN);
  servo.write(SERVO_POS[state]);

  if(BTN()) {
    tune = 1;
    pressed = 1;
  }
}

void updateLeds() {
  static unsigned char cnt = 0;
  char st = HIGH;
  if(tune) {
    static unsigned int c = 0;
    if(++c > 1000) {
      // flash active led
      digitalWrite(LEDS[state], !digitalRead(LEDS[state]));
      c = 0;
    }
  } else {
    digitalWrite(LEDS[state], st);
  }
  st = LOW;
  if(cnt < eeprom.IDLE_LED)
    st = HIGH;
  if(tune && state != _LAST)
    st = LOW;
  for(auto i = 0; i< _LAST; i++)
    if(i != state)
      digitalWrite(LEDS[i], st);

  ++cnt;
}

void checkButtons() {
  static unsigned int cnt = 0;
  if(++cnt < 2000)
    return;
  cnt = 0;

  if(pressed) {
    if(tune && !BTN()) {
      pressed = 0;
      return;
    }
    if(!UP() && !DOWN() && !BTN())
      pressed = 0;
    return;
  }

  if(BTN()) {
    pressed = 1;
    if(tune) {
      if(state == _LAST) {
        while(BTN());
        eeprom.write();
        tune = 0;
        state = _P;
        return;
      }
      ++state;
      return;
    }
  }

  if(tune) {
    unsigned char *v;
    if(state != _LAST) {
      v = &SERVO_POS[state];
    } else {
      v = &eeprom.IDLE_LED;
    }
    if(UP())
      (*v)++;
    if(DOWN())
      (*v)--;
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
  if((tune || (last != state)) && state < _LAST) {
    servo.write(SERVO_POS[state]);
    last = state;
  }
}

unsigned char i = 0;

void loop() {
  checkButtons();
  updateLeds();
  updateServo();
}

