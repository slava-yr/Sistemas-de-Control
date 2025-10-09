#include <Arduino.h>
#define IN1 32 // Motor Driver
#define IN2 33 // Motor Driver
#define nSLEEP 25 // Motor Driver enable
#define CH_A 34 // Encoder
#define CH_B 35 // Encoder

double periodo=1000000;
double tiempo;

void IRAM_ATTR ISR_FUN() // Calcula el periodo
{
  periodo = micros() - tiempo;
  tiempo = micros(); //update
}

void setup() {
  pinMode(nSLEEP,OUTPUT);
  pinMode(CH_A, INPUT_PULLUP);
  pinMode(CH_B, INPUT_PULLUP);
  digitalWrite(nSLEEP,HIGH); // Enable driver

  // Setup PWM pins
  ledcSetup(1,20000,10);
  ledcSetup(2,20000,10);

  // Attach to motor pins
  ledcAttachPin(IN1,1);
  ledcAttachPin(IN2,2);
  
  attachInterrupt(CH_A, ISR_FUN, RISING); // Triggered by encoder

  tiempo = micros();
  Serial.begin(115200);
  Serial.println("Done!");
}

void loop() {
  // put your main code here, to run repeatedly:
  double vm=5;
  double vp=2;

  uint32_t Duty;

  Duty = (uint32_t)((1-vp/vm)*1023);

  ledcWrite(1,1023); // Duty Cycle 100%
  ledcWrite(2,Duty);

  Serial.println(periodo); // Imprime el período
  delay(50);
}