/*****************************
  Código de prueba de los dos motores para el proyecto de Sistemas de Control 2025-2

  Revisado: 31/10/2025
  
*********************************/

#include <Arduino.h>
/* Motor Driver Pins */
// Motor 1
#define PWMA 23 
#define AIN1 18  
#define AIN2 19 

// Motor 2
#define BIN1 5 
#define BIN2 17 
#define PWMB 16 

/* Encoder Pins */
// Encoder motor 1 
#define EncAM1 14
#define EncBM1 27
// Encoder motor 2
#define EncAM2 26
#define EncBM2 25

// Filtro pasabajos
#define wc 50 // Frecuencia de corte del filtro (rad/s)
#define Tf 0.005 // Tiempo de muestreo (s) [5 ms]

#define Resolucion 1801 // Número de pulsos por revolución del encoder
// TODO: Definir resolución por encoder

#define VM 11.1 // Voltaje máximo a los motores

typedef struct {
  double periodo;
  double tiempo;
  // Posición
  int32_t contador;
  double posicion;
  // Cálculo de velocidad
  double vel_ang, vel_1, vel_2, vel_ang_median, vel_ang_filt; 
  // Filtro pasabajos
  double y_k_1;
  int numbuffer;
  double vmotor;
  uint8_t id; // Identificador del motor
}MotorVars;

// Variables globales de motores
MotorVars motor1 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
MotorVars motor2 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2};

void IRAM_ATTR ISR_FUN_M1() {
  // Interruption Service Routine for Motor 1
  isr_func(&motor1, EncAM1, Resolucion);
}

void IRAM_ATTR ISR_FUN_M2() {
  // Interruption Service Routine for Motor 2
  isr_func(&motor2, EncAM2, Resolucion);
}

void isr_func(MotorVars *m, int enc, int resolution)
{
  /* Interrupt Service Routine to update motor variables
    m: pointer to MotorVars struct
    enc: encoder channel pin
    resolution: encoder resolution (pulses per revolution)
  */
  uint32_t now = micros();
  m->periodo = now - m->tiempo; // Update period
  m->tiempo = now; // Update interrupt time

  if (digitalRead(enc) == 1){
    m->vel_ang = 360.0 / (m->periodo * 0.000001 * resolution); // Compute angular velocity
    m->contador = m->contador + 1;   // Interrupt counter
  }
  else {
    m->vel_ang = -360.0 / (m->periodo * 1e-6 * resolution);
    m->contador = m->contador - 1;
  }

  //Aplicando el filtro mediana
  if (((m->vel_ang<m->vel_1)&&(m->vel_ang>=m->vel_2))||((m->vel_ang>=m->vel_1)&&(m->vel_ang<m->vel_2))){
    m->vel_ang_median = m->vel_ang;
  }
  else if (((m->vel_1<m->vel_ang)&&(m->vel_1>=m->vel_2))||((m->vel_1>=m->vel_ang)&&(m->vel_1<m->vel_2))){
    m->vel_ang_median = m->vel_1;
  }
  else if (((m->vel_2<m->vel_ang)&&(m->vel_2>=m->vel_1))||((m->vel_2>=m->vel_ang)&&(m->vel_2<m->vel_1))){
    m->vel_ang_median = m->vel_2;
  }

  // Compute angular position
  m->posicion = 360.0*m->contador/resolution; // Ángulo

  // Update past velocity values
  m->vel_2 = m->vel_1;
  m->vel_1 = m->vel_ang;
}

void env_volt(double vp, uint8_t IN1, uint8_t IN2)
{
  /* Send voltage to motor driver
    vp: voltage to send to motor
    IN1: AIN1 / BIN1 pin
    IN2: AIN2 / BIN2 pin
  */
  uint32_t Duty;
 
  if (vp > VM) vp = VM;
  if (vp < -VM) vp = -VM;

  if (vp >= 0)
  {
    Duty = (uint32_t)((1-vp/VM)*1023);
    ledcWrite(1,1023-Duty);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    Duty = (uint32_t)((1+vp/VM)*1023);
    ledcWrite(1,1023-Duty);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } 
}

void low_pass_filter(void *pvParameters)
{
  MotorVars *m = (MotorVars *)pvParameters;  // point to the right motor

  while (1)
  {
    double x_k = m->vel_ang_median;
    double y_k = (1.0 / (wc * Tf + 1.0)) * (m->y_k_1 + wc * Tf * x_k);
    m->y_k_1 = y_k;
    m->vel_ang_filt = y_k;

    vTaskDelay(5);  // 5 ms delay
  }
}



void variar_voltaje(void *pvParameters)
{    
  // Escalera de tensiones al motor
  while(1)
  {
    for (int volt = 2; volt <= 12; volt+=2)
    {
      env_volt(volt, AIN1, AIN2); // Motor 1
      env_volt(volt, BIN1, BIN2); // Motor 2
      vTaskDelay(1000);
    }
  }
}

void enviar_voltaje(void *pvParameters)
{    
  // Enviar voltaje a un motor e imprimir voltaje, velocidad y posición
  MotorVars *m = (MotorVars *)pvParameters;  // point to the right motor
  
  m->vmotor = 6.0; // Voltaje a enviar
  if (m->id == 1)
  {
    env_volt(m->vmotor, AIN1, AIN2);
  }
  else // m->id == 2
  {
    env_volt(m->vmotor, BIN1, BIN2);
  }
   
  while(1)
  {
    Serial.print(m->vmotor); // Vector de voltaje 
    Serial.print("  	");
    Serial.print(m->vel_ang_filt); // Vector velocidad (tras filtro pasabajo)
    Serial.print("  	");
    Serial.println(m->posicion); // Vector posición
    Serial.println();
    vTaskDelay(25); // Cada 25 ms
  }
}

void setup() {
  // Encoders
  pinMode(EncAM1, INPUT_PULLUP); // Encoder Motor 1
  pinMode(EncBM1, INPUT_PULLUP); // Encoder Motor 1
  pinMode(EncAM2, INPUT_PULLUP); // Encoder Motor 2
  pinMode(EncBM2, INPUT_PULLUP); // Encoder Motor 2
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Setup PWM pins
  ledcSetup(1,20000,10); 
  ledcSetup(2,20000,10);

  // Attach to motor pins
  ledcAttachPin(PWMA,1);
  ledcAttachPin(PWMB,2);

  // Encoder
  attachInterrupt(EncAM1, ISR_FUN_M1, RISING); // Triggered by encoder
  attachInterrupt(EncBM1, ISR_FUN_M2, RISING); // Triggered by encoder

  // Tasks
  xTaskCreatePinnedToCore(low_pass_filter, "LPF_M1", 4000, &motor1, 1, NULL, APP_CPU_NUM); // LPF for motor 1
  xTaskCreatePinnedToCore(low_pass_filter, "LPF_M2", 4000, &motor2, 1, NULL, APP_CPU_NUM); // LPF for motor 2
  // xTaskCreatePinnedToCore(variar_voltaje, "", 4000, NULL, 2, NULL, APP_CPU_NUM); // Variar voltaje ambos motores
  xTaskCreatePinnedToCore(enviar_voltaje, "Enviar_Voltaje_M1", 4000, &motor1, 2, NULL, APP_CPU_NUM); // Enviar voltaje motor 1
  xTaskCreatePinnedToCore(enviar_voltaje, "Enviar_Voltaje_M2", 4000, &motor2, 2, NULL, APP_CPU_NUM); // Enviar voltaje motor 2

  Serial.begin(115200);
  Serial.println("Setup done!");
}

void loop() {}