#include "ShieldMotor.h"

uint8_t MotorsF[] = { 0,0x04,0x10,0x01,0x20 };
uint8_t MotorsR[] = { 0,0x08,0x02,0x40,0x80 };


void ShieldMotor::run(uint8_t direction, uint8_t speed){
    switch(direction){
        case FORWARD: motorForward(speed); break;
        case BACKWARD: motorBackward(speed); break;
        case BRAKE: motorBrake(); break;
        default: return;
    }
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
}

void ShieldMotor::shiftOut(uint8_t data) {
    for (int j = 0; j < 8; j++) {
        digitalWrite(DIR_SER, (data & 0x80) ? HIGH : LOW);
        digitalWrite(DIR_CLK, HIGH);  // Clock pulse omhoog
        digitalWrite(DIR_CLK, LOW);   // Clock pulse omlaag
        data <<= 1;  // Shift data naar links
    }
}

void ShieldMotor::motorForward(uint8_t speed) {
    switch (m_motor) {
        case 1: setPWM1(speed); break;  // Motor 1 (Achter rechts)
        case 2: setPWM2(speed); break;  // Motor 2 (Achter links)
        case 3: setPWM3(speed); break;  // Motor 3 (Voor links)
        case 4: setPWM4(speed); break;  // Motor 4 (Voor rechts)
        default: return;  // Ongeldige motor
    }
    digitalWrite(DIR_CLK, HIGH);
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(speed);
    shiftOut(MotorsF[m_motor]);
    digitalWrite(DIR_LATCH, HIGH);
}

void ShieldMotor::motorBackward(uint8_t speed){
    switch (m_motor) {
        case 1: setPWM1(speed); break;  // Motor 1 (Achter rechts)
        case 2: setPWM2(speed); break;  // Motor 2 (Achter links)
        case 3: setPWM3(speed); break;  // Motor 3 (Voor links)
        case 4: setPWM4(speed); break;  // Motor 4 (Voor rechts)
        default: return;  // Ongeldige motor
    }
    digitalWrite(DIR_CLK, HIGH);
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(speed);
    shiftOut(MotorsR[m_motor]);
    digitalWrite(DIR_LATCH, HIGH);
}

void ShieldMotor::motorBrake(){
    switch (m_motor) {
        case 1: setPWM1(255); break;  // Motor 1 (Achter rechts)
        case 2: setPWM2(255); break;  // Motor 2 (Achter links)
        case 3: setPWM3(255); break;  // Motor 3 (Voor links)
        case 4: setPWM4(255); break;  // Motor 4 (Voor rechts)
        default: return;  // Ongeldige motor

                 digitalWrite(DIR_CLK, HIGH);
                 digitalWrite(DIR_LATCH, LOW);
                 shiftOut(255);
                 shiftOut(MotorsR[m_motor]);
                 shiftOut(MotorsF[m_motor]);
                 digitalWrite(DIR_LATCH, HIGH);
    }
}
void ShieldMotor::setPWM1(uint8_t speed) {
    OCR1A = speed;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM2(uint8_t speed) {
    OCR3C = speed;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM3(uint8_t speed) {
    OCR0A = speed;  // PWM via Timer1A op Arduino Mega
}

void ShieldMotor::setPWM4(uint8_t speed) {
    OCR3A = speed;  // PWM via Timer1A op Arduino Mega
}
