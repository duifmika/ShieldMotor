#pragma once
#include "Arduino.h"

#define MOTOR_FREQ _BV(CS01) // 1/8 

#define PWM0A 6 
#define PWM0B 5 
#define PWM2A 11 
#define PWM2B 3 
                 
#define DIR_CLK    4  //SHCP 
#define DIR_LATCH  12 //STCP
#define DIR_ENABLE 7  //OE 
#define DIR_DATA   8  //DS 

#define MOTOR1A 2 
#define MOTOR1B 3 
#define MOTOR2A 1 
#define MOTOR2B 4 
#define MOTOR3A 5 
#define MOTOR3B 7 
#define MOTOR4A 0 
#define MOTOR4B 6 

#define INVALID_MOTOR 0

enum Direction : int8_t {
    FORWARD  = 1,
    BACKWARD = 2,
    RELEASE  = 3,
};

class ShieldMotor {
public:
    ShieldMotor(int motor);

    void    run(Direction dir);
    void    setSpeed(uint8_t speed);
    uint8_t getSpeed();
private:
    int8_t m_motor = 0;
    uint8_t m_speed = 0;
    
    static uint8_t latch_state;
private:
    void initPWM();
    void latch_set();
private:
    static void shiftOut(uint8_t data);
    static void setPWM1(uint8_t speed);
    static void setPWM2(uint8_t speed);
    static void setPWM3(uint8_t speed);
    static void setPWM4(uint8_t speed);
};

