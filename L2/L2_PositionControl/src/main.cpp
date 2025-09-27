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

  periodo = micros()-tiempo; // Update period
  tiempo = micros(); // Update interrupt time

  if (digitalRead(CH_B)==1){
    velang = 360.0/(periodo*0.000001*Resolucion); // Computer angular velocity
    contador=contador+1;   // Interrupt counter
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

  // Compute angular position
  posicion = 360*contador/Resolucion; // Ángulo

  // Update past velocity values
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

void env_volt(double vmotor){
  double vm = 5; //voltaje máximo
  uint32_t D; // Duty Cycle

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

void filtrar_vel(void *pvParameters) // Low pass filter
{
  double yp;
  double Tf = 0.005; // 5 ms
  double wc = 50;
  double y_1 =0;
  double y=0;

  while (1){
    yp=(velangfc-y)*wc;
    y=y_1+yp*Tf;
    velangf2=y; // Filtered angular velocity
    y_1=y;    
    vTaskDelay(5);
  }
}

void control_pos(void *pvParameters){
  double t =0;
  double T=0.05; // Periodo de muestreo
  // Parámetros del controlador PID
  double Kp = 0.0669;
  double Ti = 1;
  double Td = 0.25;

  // Variables de la regla de control
  double e_ant = 0;
  double I_ant = 0;
  double e = 0;
  double I = 0;
  double D;

  // Posición deseada
  double tetad = 100; 
  
  double u;

  while (1){
    e = (tetad-posicion);

    D = (e-e_ant)/T;
    I = I_ant + e*T;

    u = Kp*(e+ (1/Ti)*I + Td*D);

    env_volt(u);
    e_ant = e;
    I_ant = I;

    Serial.print(t);
    Serial.print("  ");
    Serial.print(u);
    Serial.print("  ");
    Serial.print(velangf2); // Show filtered speed
    Serial.print("  ");
    Serial.println(posicion); // Show position
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
  // xTaskCreatePinnedToCore(mostrar_vel," ", 4000, NULL, 2 , NULL, 1); // ishowspeed
  xTaskCreatePinnedToCore(control_pos," ", 4000, NULL, 2 , NULL, 1); // PID position control
}

void loop() {}

