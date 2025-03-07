#include "ShieldMotor.h"

uint8_t MotorsF[] = { 0,0x04,0x10,0x01,0x80 };
uint8_t MotorsR[] = { 0,0x08,0x02,0x40,0x20 };

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
    // Timer 1 (Motor 1 - Pin 11)
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS11);  // Fast PWM, prescaler 8

    // Timer 3 (Motor 2 & Motor 4 - Pin 5, Pin 2)
    TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM30);
    TCCR3B = _BV(WGM32) | _BV(CS31);  // Fast PWM, prescaler 8

    // Timer 0 (Motor 3 - Pin 6)
    TCCR0A = _BV(COM0A1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B = _BV(CS01);  // Fast PWM, prescaler 8
    
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
    OCR1A = speed ;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM2(uint8_t speed) {
    OCR3C = speed;  // PWM via Timer3C op Arduino Mega
}

void ShieldMotor::setPWM3(uint8_t speed) {
    OCR0A = speed;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM4(uint8_t speed) {
    OCR3A = speed;  // PWM via Timer3A op Arduino Mega
}
