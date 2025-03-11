
#include "ShieldMotor.h"

static uint8_t latch_state;

void ShieldMotor::run(uint8_t direction){
    uint8_t a, b;
    switch (m_motor) {
        case 1:
            a = M1A; b = M1B; break;
        case 2:
            a = M2A; b = M2B; break;
        case 3:
            a = M3A; b = M3B; break;
        case 4:
            a = M4A; b = M4B; break;
        default:
            return;
    }

    switch (direction) {
        case FORWARD:
            latch_state |= _BV(a);
            latch_state &= ~_BV(b); 
            latch_tx();
            break;
        case BACKWARD:
            latch_state &= ~_BV(a);
            latch_state |= _BV(b); 
            latch_tx();
            break;
        case BRAKE:
            latch_state &= ~_BV(a);     // A and B both low
            latch_state &= ~_BV(b); 
            latch_tx();
            break;
    }
}

void ShieldMotor::latch_tx(void) {
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(latch_state);  // Hele 8 bits tegelijk uitsturen
    digitalWrite(DIR_LATCH, HIGH);
}

ShieldMotor::ShieldMotor(int motor) {
    m_motor = motor;

    pinMode(DIR_EN, OUTPUT);
    pinMode(DIR_SER, OUTPUT);
    pinMode(DIR_LATCH, OUTPUT);
    pinMode(DIR_CLK, OUTPUT);

    latch_state = 0;
    latch_tx();

    digitalWrite(DIR_EN, LOW);

    initPWM();
}

void ShieldMotor::shiftOut(uint8_t data) {
    for (int j = 0; j < 8; j++) {
        digitalWrite(DIR_SER, (data & 0x80) ? HIGH : LOW);
        digitalWrite(DIR_CLK, HIGH);  // Clock pulse omhoog
        digitalWrite(DIR_CLK, LOW);   // Clock pulse omlaag
        data <<= 1;  // Shift data naar links
    }
}

void ShieldMotor::initPWM() {
    uint8_t freq = MOTOR34_8KHZ;
    
    // GOOD
    switch (m_motor) {
        case 1:
            latch_state &= ~_BV(M1A) & ~_BV(M1B); // set both motor pins to 0
            latch_tx();
            TCCR1A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc1a
            TCCR1B = (freq & 0x7) | _BV(WGM12);
            OCR1A = 0;

            pinMode(11, OUTPUT);
            break;
        case 2:
            latch_state &= ~_BV(M2A) & ~_BV(M2B); // set both motor pins to 0
            latch_tx();
            TCCR3A |= _BV(COM1C1) | _BV(WGM10); // fast PWM, turn on oc3c
            TCCR3B = (freq & 0x7) | _BV(WGM12);
            OCR3C = 0;

            pinMode(3, OUTPUT);
            break;
        case 3:
            latch_state &= ~_BV(M3A) & ~_BV(M3B); // set both motor pins to 0
            latch_tx();

            TCCR4A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc4a
            TCCR4B = (freq & 0x7) | _BV(WGM12);
            OCR4A = 0;

            pinMode(6, OUTPUT);
            break;
        case 4:
            latch_state &= ~_BV(M4A) & ~_BV(M4B); // set both motor pins to 0
            latch_tx();
            TCCR3A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc3a
            TCCR3B = (freq & 0x7) | _BV(WGM12);
            //TCCR4B = 1 | _BV(WGM12);
            OCR3A = 0;

            pinMode(5, OUTPUT);
            break;
    }
}

int ShieldMotor::getSpeed() {
  return m_speed;
}

void ShieldMotor::setSpeed(uint8_t speed) {
  m_speed = speed;
  switch (m_motor) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  case 3:
    setPWM3(speed); break;
  case 4:
    setPWM4(speed); break;
  }
}

void ShieldMotor::setPWM1(uint8_t speed) {
    // GOOD
    OCR1A = speed ;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM2(uint8_t speed) {
    // GOOD
    OCR3C = speed;  // PWM via Timer3C op Arduino Mega
}

void ShieldMotor::setPWM3(uint8_t speed) {
    // NO SPEED
    OCR4A = speed;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM4(uint8_t speed) {
    //BAD
    OCR3A = speed;  // PWM via Timer3A op Arduino Mega
}

