#include <Arduino.h>

#define IN1 32
#define IN2 33
#define SLEEP 25
#define CH_A 34
#define CH_B 35

double periodo=10000000;

double tiempo;
double velang;
double velangfc;
double velangf2;
double Resolucion=1280;
double vel_1=0;
double vel_2=0;
double contador=0;
double posicion; 
double tetad = 80; // Posición deseada

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
    velangfc = velang;
  }
  else if (((vel_1<velang)&&(vel_1>=vel_2))||((vel_1>=velang)&&(vel_1<vel_2))){
    velangfc = vel_1;
  }
  else if (((vel_2<velang)&&(vel_2>=vel_1))||((vel_2>=velang)&&(vel_2<vel_1))){
    velangfc = vel_2;
  }

  // Compute angular position
  posicion = 360*contador/Resolucion; // Ángulo

  // Update past velocity values
  vel_2 = vel_1;
  vel_1 = velang;
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

void servosistema(void *pvParameters) // Servosistema
{
  double t;
  double K1 = 0.0245;
  double K2_1 = 0.1352;
  double K2_2 = 0.0299;
  double vk_1 = 0; // v[k-1]
  double vk = 0;
  double ek;
  double T = 0.1; // 100 ms
  double uk;

  while(1)
  {
    ek = (tetad - posicion); // error en la posición
    vk = vk_1 + ek;
    uk = K1*vk - K2_1*posicion - K2_2*velangf2;
    env_volt(uk);
    vk_1 = vk;

    Serial.print(t);
    Serial.print("  ");
    Serial.print(uk);
    Serial.print("  ");
    Serial.print(velangf2);
    Serial.print("  ");
    Serial.println(posicion);
    t += T*1000;
    vTaskDelay(T*1000); // Período de muestreo de 100 ms
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

  // Tasks
  xTaskCreatePinnedToCore(filtrar_vel," ", 4000, NULL, 3, NULL, 1); // Low Pass Filter
  xTaskCreatePinnedToCore(servosistema," ", 4000, NULL, 2 , NULL, 1); // Low Pass Filter


}

void loop() {}