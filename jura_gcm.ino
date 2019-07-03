#include <Servo.h>
#include <EEPROM.h>

#define LED_P 3
#define LED_R 5
#define LED_N 6
#define LED_D 9
#define LED_M 8
#define OUT_UP         14
#define OUT_DOWN       15

#define BUTTON_UP       2
#define BUTTON_DOWN     7
#define BUTTON_BTN     10
#define BUTTON_BRAKE   12
#define BUTTON_PARK    11
#define BUTTON_MANUAL  16

#define SERVO_PIN 19

#define UP()     (digitalRead(BUTTON_UP)     == LOW)
#define DOWN()   (digitalRead(BUTTON_DOWN)   == LOW)
#define BTN()    (digitalRead(BUTTON_BTN)    == LOW)
#define PARK()   (digitalRead(BUTTON_PARK)   == LOW)
#define BRAKE()  (digitalRead(BUTTON_BRAKE)  == LOW)
#define MANUAL() (digitalRead(BUTTON_MANUAL) == HIGH)

#define IDLE_PWM 20

enum State {
    _P = 0,
    _R,
    _N,
    _D,
    _LAST_SERVO, // servo is for PRND only
    _M = _LAST_SERVO,
    _LAST
};

struct EEP {
    unsigned char SERVO_POS[_LAST_SERVO];
    unsigned char IDLE_LED;
    unsigned char CRC;

    void init() {
        for(auto i = 0; i< _LAST_SERVO; i++) {
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
    LEDS[_M] = LED_M;

    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_BTN, INPUT_PULLUP);
    pinMode(BUTTON_BRAKE, INPUT_PULLUP);
    pinMode(BUTTON_PARK, INPUT_PULLUP);
    pinMode(BUTTON_MANUAL, INPUT_PULLUP);
    pinMode(OUT_UP, OUTPUT);
    pinMode(OUT_DOWN, OUTPUT);
    digitalWrite(OUT_UP, LOW);
    digitalWrite(OUT_DOWN, LOW);

    for(auto i = 0; i< _LAST; i++) {
        pinMode(LEDS[i], OUTPUT);
    }

    servo.attach(SERVO_PIN);
    servo.write(SERVO_POS[state]);

    if(BTN()) {
        tune = 1;
        pressed = 1;
    }

    Serial.begin(115200);
}

void updateLeds() {
    static unsigned char cnt = 0;
    char st = LOW;
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
    st = HIGH;
    if(cnt < eeprom.IDLE_LED)
        st = LOW;
    if(tune && state != _LAST)
        st = HIGH;
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
        if(!UP() && !DOWN()) {
            pressed = 0;
        }
        return;
    }

    if(tune) {
        if(BTN()) {
            while(BTN());
            if(state == _LAST_SERVO) {
                eeprom.write();
                tune = 0;
                state = _P;
                return;
            }
            ++state;
            return;
        }

        unsigned char *v;
        if(state != _LAST_SERVO) {
            v = &SERVO_POS[state];
        } else {
            v = &eeprom.IDLE_LED;
        }
        if(UP()) {
            (*v)++;
            return;
        }
        if(DOWN()){
            (*v)--;
            return;
        }
        return;
    }

    if(BTN() && BRAKE() && PARK()) {            // if park pressed - go to parking from any mode
        state = _P;
        return;
    }
    
    switch(state) {
        case _P:
            if (!BTN() || !BRAKE())
                break;
            if(UP()) {          // P -> D
                state = _D;
                pressed = 1;
            } else if(DOWN()) { // P -> R
                state = _R;
                pressed = 1;
            }
            break;

        case _R:
            if (!BTN() || !BRAKE())
                break;
            if(UP()) {          // R -> N
                state = _N;
                pressed = 1;
            } else if(DOWN()) { // R -> P
                state = _P;
                pressed = 1;
            }
            break;

        case _N:
            if (!BTN() || !BRAKE())
                break;
            if(UP()) {          // N -> D
                state = _D;
                pressed = 1;
            } else if(DOWN()) { // N -> R
                state = _R;
                pressed = 1;
            }
            break;

        case _D:
            if(UP()) {          // D -> D
                state = _D;
                pressed = 1;
            } else if(DOWN()) { // D -> N
                if (!BTN() || !BRAKE())
                    break;
                state = _N;
                pressed = 1;
            }
            else if (MANUAL()) { // D -> M
                state = _M;
                pressed = 1; // ?
                digitalWrite(OUT_UP, HIGH);
                digitalWrite(OUT_DOWN, HIGH);
            }
            break;
        case _M:
        {
            if (!MANUAL()) { // M -> D
                state = _D;
                pressed = 1; // ?
                digitalWrite(OUT_UP, LOW);
                digitalWrite(OUT_DOWN, LOW);
            }
            else
            {
                int u = HIGH, d = HIGH;
                if(UP()) {
                    u = LOW;
                    pressed = 1;
                } else if (DOWN()) {
                    d = LOW;
                    pressed = 1;
                }
                digitalWrite(OUT_UP, u);
                digitalWrite(OUT_DOWN, d);
            }
        }
    }
}

void updateServo() {
    static char last = -1;
    if((tune || (last != state)) && state < _LAST_SERVO) {
        servo.write(SERVO_POS[state]);
        last = state;
    }
}

void loop() {
    checkButtons();
    updateLeds();
    updateServo();
}

