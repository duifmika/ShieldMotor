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

#define M3A 15 //Q0 a (0x01)
#define M2A 1 //Q1 b  (0x02)
#define M1A 2 //Q2 c  (0x04)
#define M1B 3 //Q3 d  (0x08)
#define M2B 4 //Q4 e  (0x10)
#define M4A 5 //Q5 f  (0x20)
#define M3B 6 //Q6 g  (0x40)
#define M4B 7 //Q7 h  (0x80)

int8_t MotorsF[] = {0,0x04,0x10,0x01,0x40};
int8_t MotorsR[] = {0,0x08,0x02,0x40,0x80};

/*
  motor 1 3 en 6 output --> 1 high enable, 2 high of 7 high achter rechts
  1 = PWM2A
  2 = M1A
  7 = M1B
  motor 2 11 en 14 output --> 9 high enable, 10 high of 15 high achterlinks
  9 = PWM2B
  10 = M2A
  15 = m2B
  motor 3 3 en 6 --> output --> 1 high enable, 2 high of 7 high  voorlinks
  1 = PWM0B
  2 = M3A == 14
  7 = M35
  motor 4 11 en 14--> output --> 9 high enable, 10 high of 15 high voorrechts
  9 = PWM0A
  10 = M4A
  15 = m4B
  */
  
  void setup ()
  {
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


  }


void shiftOut(int8_t Data) {
    for (int j = 0; j < 8; j++) {
        digitalWrite(DIR_SER, (Data & 0x80) ? HIGH : LOW);  
        digitalWrite(DIR_CLK, HIGH);  // Clock pulse omhoog
        digitalWrite(DIR_CLK, LOW);   // Clock pulse omlaag
        Data <<= 1;  // Shift data naar links
    }
}


void setPWM1(int8_t snelheid) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR1A = snelheid;  // PWM via Timer1A op Arduino Mega
#endif
}

void setPWM2(int8_t snelheid) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3C = snelheid;  // PWM via Timer1A op Arduino Mega
#endif
}

void setPWM3(int8_t snelheid) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR0A = snelheid;  // PWM via Timer1A op Arduino Mega
#endif
}

void setPWM4(int8_t snelheid) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3A = snelheid;  // PWM via Timer1A op Arduino Mega
#endif
}

void motor_Vooruit(int8_t N, int8_t snelheid){

  switch (N) {
        case 1: setPWM1(snelheid); break;  // Motor 1 (Achter rechts)
        case 2: setPWM2(snelheid); break;  // Motor 2 (Achter links)
        case 3: setPWM3(snelheid); break;  // Motor 3 (Voor links)
        case 4: setPWM4(snelheid); break;  // Motor 4 (Voor rechts)
        default: return;  // Ongeldige motor
  }
    digitalWrite(DIR_CLK, HIGH);
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(snelheid);
    shiftOut(MotorsF[N]);
    digitalWrite(DIR_LATCH, HIGH);
}

void motor_Achteruit(int8_t N, int8_t snelheid){

  switch (N) {
        case 1: setPWM1(snelheid); break;  // Motor 1 (Achter rechts)
        case 2: setPWM2(snelheid); break;  // Motor 2 (Achter links)
        case 3: setPWM3(snelheid); break;  // Motor 3 (Voor links)
        case 4: setPWM4(snelheid); break;  // Motor 4 (Voor rechts)
        default: return;  // Ongeldige motor
  }
    digitalWrite(DIR_CLK, HIGH);
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(snelheid);
    shiftOut(MotorsR[N]);
    digitalWrite(DIR_LATCH, HIGH);
}

void Rem(int8_t N){

switch (N) {
        case 1: setPWM1(255); break;  // Motor 1 (Achter rechts)
        case 2: setPWM2(255); break;  // Motor 2 (Achter links)
        case 3: setPWM3(255); break;  // Motor 3 (Voor links)
        case 4: setPWM4(255); break;  // Motor 4 (Voor rechts)
        default: return;  // Ongeldige motor

  digitalWrite(DIR_CLK, HIGH);
    digitalWrite(DIR_LATCH, LOW);
    shiftOut(255);
    shiftOut(MotorsR[N]);
    shiftOut(MotorsF[N]);
    digitalWrite(DIR_LATCH, HIGH);
}
}
void loop(){
  motor_Vooruit(1,255);
  motor_Vooruit(2,255);
  motor_Vooruit(3,255);
  motor_Vooruit(4,255);
  delay(4000);

  motor_Vooruit(1,122);
  motor_Vooruit(2,122);
  motor_Vooruit(3,122);
  motor_Vooruit(4,122);
  delay(4000);

  motor_Vooruit(1,0);
  motor_Vooruit(2,0);
  motor_Vooruit(3,0);
  motor_Vooruit(4,0);
  delay(4000);

  motor_Achteruit(1,255);
  motor_Achteruit(2,255);
  motor_Achteruit(3,255);
  motor_Achteruit(4,255);
  delay(4000);

  motor_Achteruit(1,122);
  motor_Achteruit(2,122);
  motor_Achteruit(3,122);
  motor_Achteruit(4,122);
  delay(4000);

  motor_Achteruit(1,0);
  motor_Achteruit(2,0);
  motor_Achteruit(3,0);
  motor_Achteruit(4,0);
  delay(4000);


for(int i = 100; i > 0; i - 10){
  motor_Achteruit(1,255);
  motor_Achteruit(2,255);
  motor_Achteruit(3,255);
  motor_Achteruit(4,255);
  delay(1000);

  Rem(1);
  Rem(2);
  Rem(3);
  Rem(4);
  delay(i);

  motor_Vooruit(1,255);
  motor_Vooruit(2,255);
  motor_Vooruit(3,255);
  motor_Vooruit(4,255);
  delay(1000);

  Rem(1);
  Rem(2);
  Rem(3);
  Rem(4);
  delay(i);
  }
}