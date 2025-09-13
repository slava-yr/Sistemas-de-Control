#include <Arduino.h>
#define IN1 32 // Motor Driver
#define IN2 33 // Motor Driver
#define nSLEEP 25 // Motor Driver enable
#define CH_A 34 // Encoder
#define CH_B 35 // Encoder
#define wc 50 // Frecuencia de corte del filtro (rad/s)
#define Tf 0.005 // Tiempo de muestreo (s) [5 ms]

const uint16_t Resolucion = 1280; 

double periodo=1000000;
double tiempo;
double vel_ang; // velocidad angular (raw)
double vel_1 = 0, vel_2 = 0;
volatile double vel_ang_median; // velocidad angular (filtro mediana)
double vel_ang_filt; // velocidad angular (filtro pasabajos)
double vpg = -2.5;

// Filtro pasabajos
double y_k_1 = 0; // y[k-1] 

void IRAM_ATTR ISR_FUN() // Calcula el periodo
{
  periodo = micros() - tiempo;
  tiempo = micros(); //update
  if (digitalRead(CH_B) == 1)
  {
    vel_ang = 360.0/(periodo*0.000001*Resolucion);
  }
  else
  {
    vel_ang = -360.0/(periodo*0.000001*Resolucion);
  }

  // Filtro mediana
  if (((vel_ang < vel_1) && (vel_ang > vel_2)) || ((vel_ang > vel_1) && (vel_ang < vel_2)))
  {
    vel_ang_median = vel_ang;
  }

  else if (((vel_1 < vel_ang) && (vel_1 > vel_2)) || ((vel_1 > vel_ang) && (vel_1 < vel_2)))
  {
    vel_ang_median = vel_1;
  }
  else if (((vel_2 < vel_ang) && (vel_2 > vel_1)) || ((vel_2 > vel_ang) && (vel_2 < vel_1)))
  {
    vel_ang_median = vel_2;
  }
  
  vel_2 = vel_1; // vel[k-2]
  vel_1 = vel_ang; // vel[k-1]
}

void low_pass_filter(void *pvParameters) // Filtro pasabajos
{
  while(1)
  {
    double x_k = vel_ang_median;
    double y_k = (1.0/(wc*Tf + 1))*(y_k_1 + wc*Tf*x_k);
    y_k_1 = y_k; // Update y[k-1] 
    
    vel_ang_filt = y_k; // Update global variable
    vTaskDelay(5); // 5 ms
  }
}

void env_volt(double vp)
{
  double vm = 5; // Voltaje máximo
  uint32_t Duty;

  if (vp >= 0)
  {
    Duty = (uint32_t)((1-vp/vm)*1023);

    ledcWrite(1,1023); // Duty Cycle 100%
    ledcWrite(2,Duty);
  }
  else
  {
    Duty = (uint32_t)((1+vp/vm)*1023);
    ledcWrite(1,Duty); // Duty Cycle 100%
    ledcWrite(2,1023);
  }
  
}

void mostrar_velocidad(void *pvParameters)
{
  double t = 0;
  while(1)
  {
    // vTaskDelay(1500);
    Serial.print(t); // Vector de tiempo
    Serial.print("  	");
    Serial.print(vpg); // Vector de voltaje 
    Serial.print("  	");
    Serial.print(vel_ang_median); // Vector de voltaje 
    Serial.print("  	");
    Serial.println(vel_ang_filt); // Vector velocidad (tras filtro pasabajo)
    t+=0.025;
    vTaskDelay(25); // Cada 25 ms
  }
}

void variar_voltaje(void *pvParameters)
{    
  while(1)
  {
    // vpg += 0.5;
    // if (vpg > 2.5) vpg = -2.5;
    vpg = 2.4; // Voltaje a enviar al motor
    env_volt(vpg);

    vTaskDelay(2000);
  }
}

void setup() {
  pinMode(nSLEEP,OUTPUT);
  pinMode(CH_A, INPUT_PULLUP);
  pinMode(CH_B, INPUT_PULLUP);
  digitalWrite(nSLEEP,HIGH); // Enable driver

  // Setup PWM pins
  ledcSetup(1,20000,10);
  ledcSetup(2,20000,10);

  // Attach to encoder pins
  ledcAttachPin(IN1,1);
  ledcAttachPin(IN2,2);
  
  attachInterrupt(CH_A, ISR_FUN, RISING); // Triggered by encoder

  tiempo = micros();
  Serial.begin(115200);

  xTaskCreatePinnedToCore(mostrar_velocidad, "", 4000, NULL, 1, NULL, APP_CPU_NUM); // Create task
  xTaskCreatePinnedToCore(variar_voltaje, "", 4000, NULL, 2, NULL, APP_CPU_NUM); // Create task
  xTaskCreatePinnedToCore(low_pass_filter, "", 4000, NULL, 3, NULL, APP_CPU_NUM); // Create filter task
  Serial.println("Done!");
}

void loop() {}