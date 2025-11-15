/*****************************
  Código de identificación de los dos motores para el proyecto de Sistemas de Control 2025-2


  Revisado: 11/11/2025
*********************************/

// MOTOR 1: L
// MOTOR 2: R

#include <Arduino.h>
// #include <WiFi.h>
// #include <PubSubClient.h>

// /* Wi-Fi and MQTT*/
// #define WIFI_SSID "iPhone de Jose"
// #define WIFI_PASSWORD "Jj12345678"
// #define TOPIC "the_xspacer/val0408" // To publish into

// WiFiClient espClient;
// PubSubClient client(espClient);

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

#define VM 11.1 // Voltaje máximo a los motores
#define DIAMETER 68.8 // mm

typedef struct {
  double periodo;
  double tiempo;
  // Posición
  int32_t contador;
  double posicion;
  // Cálculo de velocidad
  double vel_ang, vel_1, vel_2, vel_ang_median; 
  // Filtro pasabajos
  double y_k_1;
  double vmotor; // voltage
  const uint16_t resolucion;
  double Kp; // For PI
  double vel_d; // Velocidad deseada
  double ticks_to_mm; // Conversion factor ticks to mm: PI*D / resolucion
  uint8_t id; // Identificador del motor
}MotorVars;

typedef struct {
    bool active; // Active movement
    bool initialized; // initialize flag for every movement
    double target_distance; // distance to move
    double target_velocity_deg_s; // velocity to move at
    int16_t start_l; // starting position (ticks)
    int16_t start_r; // starting position (ticks)
} MovementState;

MovementState move_state = {0};



// Variables globales de motores
MotorVars motor1 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1080, 0.014, 0, 0 ,1};
MotorVars motor2 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1083, 0.0142, 0, 0, 2};

void isr_func(MotorVars *m, int encB, int resolution)
{
  /* Interrupt Service Routine to update motor variables
    m: pointer to MotorVars struct
    enc: encoder channel pin
    resolution: encoder resolution (pulses per revolution)
  */
  uint32_t now = micros();
  m->periodo = now - m->tiempo; // Update period
  m->tiempo = now; // Update interrupt time

  uint8_t stateB = digitalRead(encB);  // channel B 
  if (stateB == 0){
    m->contador = m->contador + 1;   // Interrupt counter
    m->vel_ang = 360.0 / (m->periodo * 1e-6 * resolution); // Compute angular velocity
  }
  else {
    m->contador = m->contador - 1;
    m->vel_ang = -360.0 / (m->periodo * 1e-6 * resolution);
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
  isr_func(&motor1, EncBM1, motor1.resolucion);
}

void IRAM_ATTR ISR_FUN_M2() {
  // Interruption Service Routine for Motor 2
  isr_func(&motor2, EncBM2, motor2.resolucion);
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

  // Saturation
  if (vp > VM) vp = VM;
  else if (vp < -VM) vp = -VM;

  // Update vmotor
  if (motor_id == 1) // L
  {
    motor1.vmotor = vp;
    IN1 = AIN1;
    IN2 = AIN2;
  }
  else // R
  {
    motor2.vmotor = vp;
    IN1 = BIN2;
    IN2 = BIN1;
  }

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

void speed_control(void *pvParameters) 
{
  MotorVars *m = (MotorVars *)pvParameters;  // point to the right motor

  double t = 0;
  double T = 0.001; // Periodo de muestreo                               ; // 240 deg/s

  // Parámetros del controlador PI
  double Ti = 0.0625;

  double I_ant = 0;
  double I = 0;
  double e = 0;
  double u = 0;
  
  while(1)
  {
    e = m->vel_d - m->vel_ang_median;
    I = I_ant + e*T;
    u = m->Kp * (e + (1/Ti)*I);
    
    // Send voltage to the correct motor
    if (m->id == 1) // L
    {
      env_volt(u, 1);
    }
    else // m->id == 2 (R)
    {
      env_volt(u, 2);
    }
    I_ant = I;
    vTaskDelay(1);
  }
}

void printSpeed(void *pvParameters)
{
  const uint16_t delay_ms = 10; // 1 ms
  uint16_t t = 0;
  char msg_temp[64];
  while (1)
  {
    Serial.print(t);
    Serial.print("  ");
    Serial.print("Motor_L");
    Serial.print("  ");
    Serial.print(motor1.vmotor);
    Serial.print("  ");
    Serial.print(motor1.vel_ang_median);
    Serial.print("  ");
    Serial.print("Motor_R");
    Serial.print("  ");
    Serial.print(motor2.vmotor);
    Serial.print("  ");
    Serial.println(motor2.vel_ang_median);
    t += delay_ms;
      // // --- MQTT publish ---
      // client.loop(); // keep connection alive
      // snprintf(msg_temp, sizeof(msg_temp),
      //          "{\"m1\":%.3f,\"m2\":%.3f}",
      //          motor1.vel_ang_filt,
      //          motor2.vel_ang_filt);
      // client.publish(TOPIC, msg_temp); 
    vTaskDelay(delay_ms); 
  }
}

void move_forward(double distance_mm, double vel_mm_s)
{
  // Update move_state
  move_state.active = true;
  move_state.initialized = false;
  move_state.target_distance = distance_mm;

  move_state.target_velocity_deg_s = vel_mm_s * 360.0 / (PI * DIAMETER);
}

void navigation_task(void *pvParameters)
{
    double traveled = 0;
    while (1)
    {
        if (move_state.active)
        {
            if (!move_state.initialized) // First-time initialization
            {
                move_state.start_l = motor1.contador;
                move_state.start_r = motor2.contador;
                motor1.vel_d = move_state.target_velocity_deg_s;
                motor2.vel_d = move_state.target_velocity_deg_s;
                move_state.initialized = true;
            }

            int16_t delta_l = motor1.contador - move_state.start_l;
            int16_t delta_r = motor2.contador - move_state.start_r;

            traveled = ((delta_l * motor1.ticks_to_mm) +
                        (delta_r * motor2.ticks_to_mm)) / 2.0;

            if (traveled >= move_state.target_distance)
            {
                motor1.vel_d = 0;
                motor2.vel_d = 0;

                move_state.active = false;
                move_state.initialized = false;
            }
        }
        vTaskDelay(20);
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

  motor1.ticks_to_mm = (PI * DIAMETER) / motor1.resolucion;
  motor2.ticks_to_mm = (PI * DIAMETER) / motor2.resolucion;

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
  xTaskCreatePinnedToCore(navigation_task, "Navigation", 4000, NULL, 1, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(speed_control, "Speed_Control_M1", 4000, &motor1, 2, NULL, APP_CPU_NUM); // Speed control motor 1
  xTaskCreatePinnedToCore(speed_control, "Speed_Control_M2", 4000, &motor2, 2, NULL, APP_CPU_NUM); // Speed control motor 1
  xTaskCreatePinnedToCore(printSpeed, "printSpeed", 4000, NULL, 1, NULL, APP_CPU_NUM); // Print speeds
  
  Serial.println("Setup done!");

  move_forward(100, 20);
}

void loop() {}