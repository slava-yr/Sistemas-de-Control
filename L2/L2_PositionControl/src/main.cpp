#include <Arduino.h>

#define IN1 32
#define IN2 33
#define SLEEP 25
#define CH_A 34
#define CH_B 35

double periodo=10000000;
double tiempo;
double velang;
double velangfa=0;
double velangfb=0;
double velangfc;
double velangf2;
double Resolucion=1280;
double vel_1=0;
double vel_2=0;
double vmotorg=0;
int numbuffer=0;
double contador=0;
double posicion;

void IRAM_ATTR ISR_FUN()  {  
  double velangf;

  periodo = micros()-tiempo;
  tiempo = micros();

  if (digitalRead(CH_B)==1){
    velang = 360.0/(periodo*0.000001*Resolucion);
    contador=contador+1;   // Contador de interrupciones
  }
  else {
    velang = -360.0/(periodo*0.000001*Resolucion);
    contador=contador-1;
  }
  //Aplicando el filtro mediana
  if (((velang<vel_1)&&(velang>=vel_2))||((velang>=vel_1)&&(velang<vel_2))){
    velangf = velang;
  }
  if (((vel_1<velang)&&(vel_1>=vel_2))||((vel_1>=velang)&&(vel_1<vel_2))){
    velangf = vel_1;
  }
  if (((vel_2<velang)&&(vel_2>=vel_1))||((vel_2>=velang)&&(vel_2<vel_1))){
    velangf = vel_2;
  }

  posicion = 360*contador/Resolucion; // Ángulo

  vel_2 = vel_1;
  vel_1 = velang;

  if (numbuffer==0){
    velangfa=velangf;
    numbuffer=1;
  }
  else {
    velangfb=velangf;
    numbuffer=0;
  }
  velangfc=velangf;
}

double velocidad(void){
  if (numbuffer==0){
    return velangfb;
  }
  else
  {
    return velangfa;
  }
}

void env_volt(double vmotor){
  double vm = 5; //voltaje máximo
  uint32_t D;

  // Saturación a +3V o -3V 
  if (vmotor>3.0){
    vmotor=3.0;
  };
  if (vmotor<-3.0){
    vmotor=-3.0;
  };

  // Sentido 1 
  if (vmotor>=0){
    D = (uint32_t)((1.0-vmotor/vm)*1024);
    ledcWrite(1,1024); //Duty cycle 
    ledcWrite(2,D); //Duty cycle 100%
  }
  // Sentido contrario
  else {
    D = (uint32_t)((1.0+vmotor/vm)*1024);
    ledcWrite(1,D); //Duty cycle 
    ledcWrite(2,1024); //Duty cycle 100%
  }

}

void filtrar_vel(void *pvParameters){
  double yp;
  double Tf = 0.005;
  double wc = 50;
  double y_1 =0;
  double y =0;
  // double t=0;
  while (1){
    yp=(velangfc-y)*wc;
    y=y_1+yp*Tf;
    velangf2=y;
    y_1=y;
    // t=t+5;    
    vTaskDelay(5);
  }
}

void mostrar_vel(void *pvParameters){
  double t = 0;
  while (1){
    Serial.print(t);
    Serial.print("  ");
    Serial.print(vmotorg);
    Serial.print("  ");
    Serial.print(velangfc);
    Serial.print("  ");
    Serial.println(velangf2);
    t=t+20;
    vTaskDelay(20);
  }
}

void control_pos(void *pvParameters){
  double t =0;
  double Kp = 0.0378;
  double Td = 0.1;
  double e_ant = 0;
  double D;
  double e;
  double tetad = 90; 
  double T=0.05;
  double u;

  while (1){
    e = (tetad-posicion);
    D = (e-e_ant)/T;
    u = Kp*(e+Td*D);
    env_volt(u);
    e_ant = e;

    Serial.print(t);
    Serial.print("  ");
    Serial.print(u);
    Serial.print("  ");
    Serial.print(velangfc);
    Serial.print("  ");
    Serial.println(posicion);
    t=t+50;
    vTaskDelay(50);
  }
}

void setup() {
  pinMode(SLEEP,OUTPUT);
  digitalWrite(SLEEP,HIGH);

  ledcSetup(1,20000,10);
  ledcSetup(2,20000,10);

  ledcAttachPin(IN1,1);
  ledcAttachPin(IN2,2);

  pinMode(CH_A,INPUT_PULLUP);
  pinMode(CH_B,INPUT_PULLUP);

  attachInterrupt(CH_A,ISR_FUN,RISING);
  tiempo =micros();

  Serial.begin(115200);

  xTaskCreatePinnedToCore(filtrar_vel," ", 4000, NULL, 3 , NULL, 1); // Low Pass Filter
  xTaskCreatePinnedToCore(mostrar_vel," ", 4000, NULL, 2 , NULL, 1); // ishowspeed
  xTaskCreatePinnedToCore(control_pos," ", 4000, NULL, 2 , NULL, 1); // PID position control
}

void loop() {}

