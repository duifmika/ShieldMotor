#include "ShieldMotor.h"

uint8_t ShieldMotor::latch_state = 0;

void ShieldMotor::run(Direction dir) {
    if (m_motor == INVALID_MOTOR)
        return;

    uint8_t a, b;
    constexpr uint8_t MOTOR_BIT_A[4] = {MOTOR1A, MOTOR2A, MOTOR3A, MOTOR4A};
    constexpr uint8_t MOTOR_BIT_B[4] = {MOTOR1B, MOTOR2B, MOTOR3B, MOTOR4B};
    
    // convert base from 1 to 0
    a = MOTOR_BIT_A[m_motor-1];
    b = MOTOR_BIT_B[m_motor-1];

    switch (dir) {
        case FORWARD:
            latch_state |= _BV(a);
            latch_state &= ~_BV(b); 
            break;
        case BACKWARD:
            latch_state &= ~_BV(a);
            latch_state |= _BV(b); 
            break;
        case RELEASE:
            latch_state &= ~_BV(a); 
            latch_state &= ~_BV(b); 
            setSpeed(0);
            break;
    }
    latch_set();
}

void ShieldMotor::latch_set() {
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(latch_state); 
    digitalWrite(DIR_LATCH, HIGH);
}

ShieldMotor::ShieldMotor(int motor) {
    m_motor = motor;

    if (m_motor < 1 || m_motor > 4)
        m_motor = INVALID_MOTOR;

    pinMode(DIR_ENABLE, OUTPUT);
    pinMode(DIR_DATA, OUTPUT);
    pinMode(DIR_LATCH, OUTPUT);
    pinMode(DIR_CLK, OUTPUT);

    latch_state = 0;
    latch_set();

    digitalWrite(DIR_ENABLE, LOW);

    initPWM();
}

void ShieldMotor::shiftOut(uint8_t data) {
    for (int j = 0; j < 8; j++) {
        digitalWrite(DIR_DATA, (data & 0x80) ? HIGH : LOW);
        digitalWrite(DIR_CLK, HIGH); 
        delayMicroseconds(1);
        digitalWrite(DIR_CLK, LOW);  
        data <<= 1; 
    }
}

void ShieldMotor::initPWM() {
    if (m_motor == INVALID_MOTOR)
        return;

    switch (m_motor) {
        case 1:
            latch_state &= ~_BV(MOTOR1A) & ~_BV(MOTOR1B); 
            latch_set();

            TCCR1A |= _BV(COM1A1) | _BV(WGM10); 
            TCCR1B = (MOTOR_FREQ & 0x7) | _BV(WGM12);
            OCR1A = 0;

            pinMode(PWM2A, OUTPUT);
            break;
        case 2:
            latch_state &= ~_BV(MOTOR2A) & ~_BV(MOTOR2B); 
            latch_set();

            TCCR3A |= _BV(COM1C1) | _BV(WGM10); 
            TCCR3B = (MOTOR_FREQ & 0x7) | _BV(WGM12);
            OCR3C = 0;

            pinMode(PWM2B, OUTPUT);
            break;
        case 3:
            latch_state &= ~_BV(MOTOR3A) & ~_BV(MOTOR3B); 
            latch_set();

            TCCR4A |= _BV(COM1A1) | _BV(WGM10); 
            TCCR4B = (MOTOR_FREQ & 0x7) | _BV(WGM12);
            OCR4A = 0;

            pinMode(PWM0A, OUTPUT);
            break;
        case 4:
            latch_state &= ~_BV(MOTOR4A) & ~_BV(MOTOR4B); 
            latch_set();

            TCCR3A |= _BV(COM1A1) | _BV(WGM10); 
            TCCR3B = (MOTOR_FREQ & 0x7) | _BV(WGM12);
            OCR3A = 0;

            pinMode(PWM0B, OUTPUT);
            break;
    }
}

int ShieldMotor::getSpeed() {
    return m_speed;
}

void ShieldMotor::setSpeed(uint8_t speed) {
    if (m_motor == INVALID_MOTOR)
        return;

    m_speed = speed;
    static void (*pwmFuncs[4])(uint8_t) = {setPWM1, setPWM2, setPWM3, setPWM4};
    pwmFuncs[m_motor-1](speed);
}

void ShieldMotor::setPWM1(uint8_t speed) {
    OCR1A = speed;
}

void ShieldMotor::setPWM2(uint8_t speed) {
    OCR3C = speed;
}

void ShieldMotor::setPWM3(uint8_t speed) {
    OCR4A = speed; 
}

void ShieldMotor::setPWM4(uint8_t speed) {
    OCR3A = speed;  
}

