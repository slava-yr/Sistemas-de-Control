#include <Arduino.h>

#define STBY 23
// Motor 1
#define AIN1 19 // CAMBIAR 
#define AIN2 21 // CAMBIAR 
#define PWMA 22 

// // Motor 2
// #define BIN1 18 // CAMBIAR  
// #define BIN2 17 // CAMBIAR
// #define PWMB 16

// Encoder
#define M1_CH_A 15 // Encoder Motor 1
#define M1_CH_B 16 // Encoder Motor 1
// #define M2_CH_A  // Encoder Motor 2

// Filtro pasabajos
#define wc 50 // Frecuencia de corte del filtro (rad/s)
#define Tf 0.005 // Tiempo de muestreo (s) [5 ms]

#define Resolucion 1801 // Número de pulsos por revolución del encoder
#define VM 11.1 // Voltaje máximo a los motores

double periodo = 1000000;
double tiempo;

// Cálculo de velocidad
double vel_ang; // velocidad angular (raw)
double vel_1 = 0, vel_2 = 0;
double vel_ang_median; // velocidad angular (filtro mediana)
double vel_ang_filt; // velocidad angular (filtro pasabajos)

// Filtro pasabajos
double y_k_1 = 0; // y[k-1]

// Voltaje a enviar al motor
double Vmotor = 5;

void IRAM_ATTR ISR_FUN_M1() // Calcula el periodo
{
  periodo = micros() - tiempo;
  tiempo = micros(); //update
  if (digitalRead(M1_CH_B) == 1)
  {
    vel_ang = -360.0/(periodo*0.000001*Resolucion);
  }
  else
  {
    vel_ang = 360.0/(periodo*0.000001*Resolucion);
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

void env_volt(double vp)
{
  uint32_t Duty;
 
  if (vp > VM) vp = VM;
  if (vp < -VM) vp = -VM;

  if (vp >= 0)
  {
    Duty = (uint32_t)((1-vp/VM)*1023);
    ledcWrite(1,1023-Duty);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    Duty = (uint32_t)((1+vp/VM)*1023);
    ledcWrite(1,1023-Duty);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } 
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

void variar_voltaje(void *pvParameters)
{    
  while(1)
  {
    for (int volt = 2; volt <= 12; volt+=2)
    {
      Vmotor = volt;
      env_volt(volt);
      vTaskDelay(1000);
    }
  }
}

void mostrar_velocidad(void *pvParameters)
{
  double t = 0;
  while(1)
  {
    Serial.print(t); // Vector de tiempo
    Serial.print("  	");
    Serial.print(Vmotor); // Vector de voltaje 
    Serial.print("  	");
    Serial.println(vel_ang_filt); // Vector velocidad (tras filtro pasabajo)
    t+=0.025;
    vTaskDelay(25); // Cada 25 ms
  }
}

void setup() {
  // Declare motor pins as output
  pinMode(M1_CH_A, INPUT_PULLUP); // Encoder Motor 1
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  // pinMode(BIN1, OUTPUT);
  // pinMode(BIN2, OUTPUT);
  // pinMode(PWMB, OUTPUT);

  // Setup PWM pins
  ledcSetup(1,20000,10);

  // Attach to motor pins
  ledcAttachPin(PWMA,1);

  // Encoder
  attachInterrupt(M1_CH_A, ISR_FUN_M1, RISING); // Triggered by encoder

  // Tasks
  xTaskCreatePinnedToCore(low_pass_filter, "", 4000, NULL, 3, NULL, APP_CPU_NUM); // Create task
  xTaskCreatePinnedToCore(variar_voltaje, "", 4000, NULL, 2, NULL, APP_CPU_NUM); // Create task
  xTaskCreatePinnedToCore(mostrar_velocidad, "", 4000, NULL, 1, NULL, APP_CPU_NUM); // Create task


  digitalWrite(STBY, HIGH); // Enable driver
  Serial.begin(115200);
  Serial.println("Setup done!");
}

void loop() {
}