#pragma once
#include "Arduino.h"

#define MOTOR34_8KHZ _BV(CS01)              // divide by 8

#define PWM2B 3 //For slots M1&M2 on the L293D Motor Shield
#define DIR_CLK 4 //SHCP of the Shift Register 74HC595
#define PWM0B 5 //For slots M3&M4 on the L293D Motor Shield
#define PWM0A 6 //For slots M3&M4 on the L293D Motor Shield
#define DIR_EN 7 //OE of the Shift Register 74HC595
#define DIR_SER 8 //DS of the Shift Register 74HC595
#define PWM1A 9 //For slots SERVO1&SERVO3 on the L293D Motor Shield
#define PWM1B 10 //For slots SERVO1&SERVO3 on the L293D Motor Shield
#define PWM2A 11 //For slots M1&M2 on the L293D Motor Shield
#define DIR_LATCH 12 //STCP of the Shift Register 74HC595

#define M3A 5 //Q0 a (0x01)
#define M2A 1 //Q1 b  (0x02)
#define M1A 2 //Q2 c  (0x04)
#define M1B 3 //Q3 d  (0x08)
#define M2B 4 //Q4 e  (0x10)
#define M4A 0 //Q5 f  (0x20)
#define M3B 7 //Q6 g  (0x40)
#define M4B 6 //Q7 h  (0x80)

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

class ShieldMotor {
public:
    ShieldMotor(int motor);
    void run(uint8_t direction);
    void setSpeed(uint8_t speed);
    int getSpeed();
private:
    int m_motor = 0;
    int m_speed = 0;
private:
    void initPWM(void);
    void latch_tx(void);
    
private:
    void shiftOut(uint8_t data);
    void setPWM1(uint8_t speed);
    void setPWM2(uint8_t speed);
    void setPWM3(uint8_t speed);
    void setPWM4(uint8_t speed);
};
