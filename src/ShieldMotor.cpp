#include "ShieldMotor.h"

uint8_t MotorsF[] = { 0,0x04,0x10,0x01,0x80 };
uint8_t MotorsR[] = { 0,0x08,0x02,0x40,0x20 };
static uint8_t latch_state;

void ShieldMotor::run(uint8_t direction){
    switch(direction){
        case FORWARD: motorForward(); break;
        case BACKWARD: motorBackward(); break;
        case BRAKE: motorBrake(); break;
        default: return;
    }
}

void ShieldMotor::latch_tx(void) {
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(latch_state);  // Hele 8 bits tegelijk uitsturen
    digitalWrite(DIR_LATCH, HIGH);
}

ShieldMotor::ShieldMotor(int motor) {
    pinMode(DIR_EN, OUTPUT);
    pinMode(DIR_SER, OUTPUT);
    pinMode(DIR_LATCH, OUTPUT);
    pinMode(DIR_CLK, OUTPUT);
    pinMode(PWM0A, OUTPUT);
    pinMode(PWM0B, OUTPUT);
    pinMode(PWM2A, OUTPUT);
    pinMode(PWM2B, OUTPUT);
    digitalWrite(PWM2A, HIGH);
    digitalWrite(PWM2B, HIGH);
    digitalWrite(PWM0A, HIGH);
    digitalWrite(PWM0B, HIGH);
    digitalWrite(DIR_EN, LOW);
    pinMode(M3B, OUTPUT);

    m_motor = motor;
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
    uint8_t freq = _BV(CS01);
    
    // GOOD
    TCCR1A |= _BV(COM1A1) | _BV(WGM10); 
    TCCR1B = (freq & 0x7) | _BV(WGM12);
    OCR1A = 0;

    // GOOD
    TCCR3A |= _BV(COM1C1) | _BV(WGM10); 
    TCCR3B = (freq & 0x7) | _BV(WGM12);
    OCR3C = 0;    

    // SPEED doesnt work
    TCCR4A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc4a
    TCCR4B = (freq & 0x7) | _BV(WGM12);
    //TCCR4B = 1 | _BV(WGM12);
    OCR4A = 0;

    // BAD
    TCCR3A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc3a
    TCCR3B = (freq & 0x7) | _BV(WGM12);
    //TCCR4B = 1 | _BV(WGM12);
    OCR3A = 0;    

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

void ShieldMotor::motorForward() {
    latch_state |= MotorsF[m_motor];  // Zet Forward bit aan
    latch_state &= ~MotorsR[m_motor]; // Zet Reverse bit uit
    latch_tx();                       // Verstuur complete latch_state
}

void ShieldMotor::motorBackward() {
    latch_state |= MotorsR[m_motor];  // Zet Reverse bit aan
    latch_state &= ~MotorsF[m_motor]; // Zet Forward bit uit
    latch_tx();                       // Verstuur complete latch_state
}

void ShieldMotor::motorBrake() {
    latch_state &= ~MotorsF[m_motor];  // Zet Forward bit uit
    latch_state &= ~MotorsR[m_motor];  // Zet Reverse bit uit
    latch_tx();                        // Verstuur complete latch_state
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
