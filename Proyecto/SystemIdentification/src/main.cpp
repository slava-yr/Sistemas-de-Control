/*****************************
  Código de identificación de los dos motores para el proyecto de Sistemas de Control 2025-2


  Revisado: 11/11/2025
*********************************/

// MOTOR 1: L
// MOTOR 2: R

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* Wi-Fi and MQTT*/
#define WIFI_SSID "iPhone de Jose"
#define WIFI_PASSWORD "Jj12345678"
#define TOPIC "the_xspacer/val0408" // To publish into

WiFiClient espClient;
PubSubClient client(espClient);

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
  const uint16_t resolucion;
  uint8_t id; // Identificador del motor
}MotorVars;

// Variables globales de motores
MotorVars motor1 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1080, 1};
MotorVars motor2 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1083, 2};

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

void IRAM_ATTR ISR_FUN_M1() {
  // Interruption Service Routine for Motor 1
  isr_func(&motor1, EncAM1, motor1.resolucion);
}

void IRAM_ATTR ISR_FUN_M2() {
  // Interruption Service Routine for Motor 2
  isr_func(&motor2, EncAM2, motor2.resolucion);
}

void env_volt(double vp, uint8_t motor_id)
{
  /* Send voltage to motor driver
    vp: voltage to send to motor
    IN1: AIN1 / BIN1 pin
    IN2: AIN2 / BIN2 pin
  */
  uint32_t Duty;
  uint8_t IN1, IN2;

  if (motor_id == 1) // L
  {
    IN1 = AIN1;
    IN2 = AIN2;
  }
  else // R
  {
    IN1 = BIN2;
    IN2 = BIN1;
  }
 
  // Saturation
  if (vp > VM) vp = VM;
  else if (vp < -VM) vp = -VM;

  // Direction
  if (vp >= 0)
  {
    Duty = 1023 - (uint32_t)((1-vp/VM)*1023);
    ledcWrite(motor_id, Duty);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    Duty = 1023 - (uint32_t)((1+vp/VM)*1023);
    ledcWrite(motor_id, Duty);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } 
}

// void low_pass_filter(void *pvParameters)
// {
//   MotorVars *m = (MotorVars *)pvParameters;  // point to the right motor

//   while (1)
//   {
//     double x_k = m->vel_ang_median;
//     double y_k = (1.0 / (wc * Tf + 1.0)) * (m->y_k_1 + wc * Tf * x_k);
//     m->y_k_1 = y_k;
//     m->vel_ang_filt = y_k;

//     vTaskDelay(1);  // 1 ms delay
//   }
// }

void enviar_voltaje(void *pvParameters)
{    
  // Enviar voltaje a un motor e imprimir voltaje, velocidad y posición
  MotorVars *m = (MotorVars *)pvParameters;  // point to the right motor
  
  
  m->vmotor = 3.0; // Voltaje a enviar
  if (m->id == 1) // L
  {
    env_volt(m->vmotor, 1);
  }
  else // m->id == 2 (R)
  {
    env_volt(m->vmotor, 2);
  }
  
  while(1)
  {
    vTaskDelay(25); // Cada 25 ms
    if (millis() > 6000) // After 5 seconds, stop the motor
    {
      m->vmotor = 0.0;
      if (m->id == 1) // L
      {
        env_volt(m->vmotor, 1);
      }
      else // m->id == 2 (R)
      {
        env_volt(m->vmotor, 2);
      }
    }
  }
}

void printSpeed(void *pvParameters)
{
  const uint16_t delay_ms = 1; // 1 ms
  uint16_t t = 0;
  char msg_temp[64];
  while (1)
  {
    if (t < 400)
    {
      Serial.print(t);
      Serial.print("  ");
      Serial.print("Motor_L");
      Serial.print("  ");
      Serial.print(motor1.vmotor);
      Serial.print("  ");
      Serial.print(motor1.vel_ang);
      Serial.print("  ");
      Serial.print("Motor_R");
      Serial.print("  ");
      Serial.print(motor2.vmotor);
      Serial.print("  ");
      Serial.println(motor2.vel_ang);
      t += delay_ms;
      // // --- MQTT publish ---
      // client.loop(); // keep connection alive
      // snprintf(msg_temp, sizeof(msg_temp),
      //          "{\"m1\":%.3f,\"m2\":%.3f}",
      //          motor1.vel_ang_filt,
      //          motor2.vel_ang_filt);
      // client.publish(TOPIC, msg_temp); 
    }  
    vTaskDelay(delay_ms); 
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

  Serial.begin(115200);
  
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // while (WiFi.status() != WL_CONNECTED) {
  // 	delay(1000);
  // 	Serial.println("Conectando a WiFi...");
  // }
  // Serial.println("Conectado a WiFi ");

  // // MQTT
  // client.setServer("www.xspace.pe", 1883); 
	// while (!client.connected())
  // { 
  //   Serial.println("Connecting to MQTT...");
  //   if (client.connect("ESP32Client")) 
  //     Serial.println("connected");
  //   else
  //   {
  //     Serial.print("failed with state ");
  //   	Serial.print(client.state());
  //     delay(1000);
  //   }
  // }

  Serial.println("Starting setup...");
  delay(5000); // Delay 1 second before starting
  Serial.println("Setup started.");
  // Encoder
  attachInterrupt(EncAM1, ISR_FUN_M1, RISING); // Triggered by encoder (motor 1)
  attachInterrupt(EncAM2, ISR_FUN_M2, RISING); // Triggered by encoder (motor 2)

  // Tasks
  // xTaskCreatePinnedToCore(low_pass_filter, "LPF_M1", 4000, &motor1, 1, NULL, APP_CPU_NUM); // LPF for motor 1
  // xTaskCreatePinnedToCore(low_pass_filter, "LPF_M2", 4000, &motor2, 1, NULL, APP_CPU_NUM); // LPF for motor 2
  xTaskCreatePinnedToCore(enviar_voltaje, "Enviar_Voltaje_M1", 4000, &motor1, 2, NULL, APP_CPU_NUM); // Enviar voltaje motor 1
  xTaskCreatePinnedToCore(enviar_voltaje, "Enviar_Voltaje_M2", 4000, &motor2, 2, NULL, APP_CPU_NUM); // Enviar voltaje motor 2
  xTaskCreatePinnedToCore(printSpeed, "printSpeed", 4000, NULL, 1, NULL, APP_CPU_NUM); // Print speeds
  
  Serial.println("Setup done!");
}

void loop() {}