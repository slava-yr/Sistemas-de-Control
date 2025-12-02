/*****************************
  Proyecto Sistemas de Control 2025-2
  HMI web TENSIÓN – TIEMPO – VELOCIDAD
  Motor IZQ (motor1) y Motor DER (motor2)

  Actualizado: 02/12/2025
*****************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

/* ====== CONFIGURAR WIFI ====== */
const char* ssid     = "JOSE FUENTES_2.4G";
const char* password = "#87654321#";

/* ====== SERVIDOR WEB ====== */
WebServer server(80);

/* Motor Driver Pins */
// Motor IZQ (motor1)
#define PWMA 23 
#define AIN1 18  
#define AIN2 19 

// Motor DER (motor2)
#define BIN1 5 
#define BIN2 17 
#define PWMB 16 

/* Encoder Pins */
// Encoder motor IZQ
#define EncAM1 14
#define EncBM1 27
// Encoder motor DER
#define EncAM2 26
#define EncBM2 25

// Filtro pasabajos
#define wc 50
#define Tf 0.005
#define Resolucion 1801
#define VM 11.1

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
  int numbuffer;
  double vmotor;
  const uint16_t resolucion;
  double Kp; // For PI
  uint8_t id; // Identificador del motor
}MotorVars;

// Variables globales de motores
MotorVars motor1 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1080, 0.014, 1};
MotorVars motor2 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1083, 0.0142, 2};

/* ====== PÁGINA HTML ====== */
const char MAIN_page[] PROGMEM = R"=====( 
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<title>Tensión - Tiempo - Velocidad</title>
<style>
  html, body {
    height: 100%;
    margin: 0;
    padding: 0;
  }
  body {
    background: #b7f41a;                /* Verde fosforescente */
    font-family: Arial, sans-serif;
    display: flex;
    flex-direction: column;
  }

  .tabla-container {
    width: 95%;
    margin: 20px auto 0 auto;
  }

  table.panel {
    width: 100%;
    border-collapse: collapse;
  }

  table.panel th,
  table.panel td {
    border: 4px solid #000000;
    padding: 24px;
    text-align: center;
  }

  table.panel th {
    background: #c5ff3a;               /* Verde un poco más claro */
    font-size: 28px;
    font-weight: bold;
  }

  table.panel td {
    background: #ffffff;
    font-size: 24px;
  }

  .nombre {
    position: fixed;
    right: 40px;
    bottom: 20px;
    border-radius: 18px;
    border: 3px solid #000000;
    padding: 6px 20px;
    background: #d8ff5e;
    font-size: 16px;
  }
</style>
</head>
<body>

  <div class="tabla-container">
    <table class="panel">
      <thead>
        <tr>
          <th>TENSIÓN</th>
          <th>TIEMPO</th>
          <th>VELOCIDAD</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td id="tension">--</td>
          <td id="tiempo">--</td>
          <td id="velocidad">--</td>
        </tr>
      </tbody>
    </table>
  </div>

  <div class="nombre">José Emmanuel Fuentes Manrique</div>

  <script>
    async function actualizar() {
      try {
        const r = await fetch('/status');
        if (!r.ok) return;
        const data = await r.json();

        // Tensión: motor IZQ y motor DER
        document.getElementById('tension').textContent =
          "Izq: " + data.tension_izq.toFixed(2) + " V   |   " +
          "Der: " + data.tension_der.toFixed(2) + " V";

        // Tiempo (común)
        document.getElementById('tiempo').textContent =
          data.tiempo.toFixed(2) + " s";

        // Velocidad: motor IZQ y motor DER
        document.getElementById('velocidad').textContent =
          "Izq: " + data.vel_izq.toFixed(2) + " °/s   |   " +
          "Der: " + data.vel_der.toFixed(2) + " °/s";

      } catch (e) {
        console.log("Error en actualización:", e);
      }
    }

    setInterval(actualizar, 500);
    actualizar();
  </script>
</body>
</html>
)=====";

/* ====== FUNCIONES DE MOTORES Y FILTRO ====== */

void isr_func(MotorVars *m, int enc, int resolution) {
  uint32_t now = micros();
  m->periodo = now - m->tiempo;
  m->tiempo = now;

  if (digitalRead(enc) == 1) {
    m->vel_ang = 360.0 / (m->periodo * 1e-6 * resolution);
    m->contador++;
  } else {
    m->vel_ang = -360.0 / (m->periodo * 1e-6 * resolution);
    m->contador--;
  }

  // Filtro mediana
  if (((m->vel_ang<m->vel_1)&&(m->vel_ang>=m->vel_2))||((m->vel_ang>=m->vel_1)&&(m->vel_ang<m->vel_2)))
    m->vel_ang_median = m->vel_ang;
  else if (((m->vel_1<m->vel_ang)&&(m->vel_1>=m->vel_2))||((m->vel_1>=m->vel_ang)&&(m->vel_1<m->vel_2)))
    m->vel_ang_median = m->vel_1;
  else
    m->vel_ang_median = m->vel_2;

  m->posicion = 360.0 * m->contador / resolution;
  m->vel_2 = m->vel_1;
  m->vel_1 = m->vel_ang;
}

void IRAM_ATTR ISR_FUN_M1() { isr_func(&motor1, EncAM1, Resolucion); }
void IRAM_ATTR ISR_FUN_M2() { isr_func(&motor2, EncAM2, Resolucion); }

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
  double T = 0.001; // Periodo de muestreo
  double vel_d = 200; // 200 deg/s

  // Parámetros del controlador PI
  double Ti = 0.0625;

  double I_ant = 0;
  double I = 0;
  double e = 0;
  double u = 0;
  
  while(1)
  {
    e = vel_d - m->vel_ang_median;
    I = I_ant + e*T;
    u = m->Kp * (e + (1/Ti)*I);

    if (u < 0) u = 0; // *Saturar para evitar el error por el encoder 
    
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
  const uint16_t delay_ms = 5; // 1 ms
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
    vTaskDelay(delay_ms); 
  }
}

/* ====== HANDLERS WEB ====== */

void handleRoot() { 
  server.send_P(200, "text/html", MAIN_page); 
}

// /status -> JSON con tensiones y velocidades de IZQ y DER
void handleStatus() {
  unsigned long now = millis();
  double t_s = now / 1000.0;

  String json = "{";
  json += "\"tension_izq\":" + String(motor1.vmotor,       3) + ",";
  json += "\"tension_der\":" + String(motor2.vmotor,       3) + ",";
  json += "\"tiempo\":"      + String(t_s,                  3) + ",";
  json += "\"vel_izq\":"     + String(motor1.vel_ang_median, 3) + ",";
  json += "\"vel_der\":"     + String(motor2.vel_ang_median, 3);
  json += "}";

  server.send(200, "application/json", json);
}

/* ====== SETUP ====== */
void setup() {
  Serial.begin(115200);
  Serial.println("\nSetup...");

  // Pines
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(EncAM1, INPUT_PULLUP); pinMode(EncBM1, INPUT_PULLUP);
  pinMode(EncAM2, INPUT_PULLUP); pinMode(EncBM2, INPUT_PULLUP);

  // PWM
  ledcSetup(1, 20000, 10);
  ledcSetup(2, 20000, 10);
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  }
  Serial.println();
  Serial.print("Conectado! IP: ");
  Serial.println(WiFi.localIP());

  // WebServer
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("Servidor HTTP iniciado");

  // Interrupciones
  attachInterrupt(EncAM1, ISR_FUN_M1, RISING);
  attachInterrupt(EncAM2, ISR_FUN_M2, RISING);

  // Tasks
  xTaskCreatePinnedToCore(speed_control, "Speed_Control_M1", 4000, &motor1, 2, NULL, APP_CPU_NUM); // Speed control motor 1
  xTaskCreatePinnedToCore(speed_control, "Speed_Control_M2", 4000, &motor2, 2, NULL, APP_CPU_NUM); // Speed control motor 1
  xTaskCreatePinnedToCore(printSpeed, "printSpeed", 4000, NULL, 1, NULL, APP_CPU_NUM); // Print speeds

  Serial.println("Setup done!");
}

/* ====== LOOP ====== */
void loop() {
  server.handleClient();
}
