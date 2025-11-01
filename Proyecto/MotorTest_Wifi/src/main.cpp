/*****************************
  Código de prueba de los dos motores para el proyecto de Sistemas de Control 2025-2
  Revisado: 31/10/2025
*********************************/

// MOTOR 1: L
// MOTOR 2: R

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

/* ====== CONFIGURAR WIFI ====== */
const char* ssid     = "iPhone de Jose";
const char* password = "Jj12345678";

/* ====== SERVIDOR WEB ====== */
WebServer server(80);
WiFiClient sseClient;
bool sseConnected = false;
unsigned long lastSSESend = 0;

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
#define wc 50
#define Tf 0.005
#define Resolucion 1801
#define VM 11.1

typedef struct {
  double periodo;
  double tiempo;
  int32_t contador;
  double posicion;
  double vel_ang, vel_1, vel_2, vel_ang_median, vel_ang_filt; 
  double y_k_1;
  double vmotor;
  uint8_t id;
} MotorVars;

// Variables globales
MotorVars motor1 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
MotorVars motor2 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2};

/* ====== PÁGINA HTML ====== */
const char MAIN_page[] PROGMEM = R"=====( 
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<title>ESP32 - Motores en tiempo real</title>
<style>
  body {
    background: #111;
    color: #0f0;
    font-family: Arial, sans-serif;
    padding: 20px;
  }
  h1 { color: #0f9; }
  .card {
    border: 1px solid #0f0;
    border-radius: 10px;
    padding: 15px;
    margin-bottom: 15px;
    background: #0004;
  }
  .label { color: #aaa; font-size: 0.8rem; }
  .value { font-size: 2rem; font-weight: bold; }
  .row { display: flex; gap: 30px; flex-wrap: wrap; }
</style>
</head>
<body>
  <h1>Monitor de Motores - ESP32</h1>
  <p>Valores leídos en tiempo real (como el monitor serie).</p>
  <div class="row">
    <div class="card" style="flex:1;">
      <h2>Motor 1</h2>
      <div class="label">Voltaje (V)</div>
      <div id="m1_v" class="value">--</div>
      <div class="label">Velocidad filtrada (°/s)</div>
      <div id="m1_w" class="value">--</div>
      <div class="label">Posición (°)</div>
      <div id="m1_p" class="value">--</div>
    </div>
    <div class="card" style="flex:1;">
      <h2>Motor 2</h2>
      <div class="label">Voltaje (V)</div>
      <div id="m2_v" class="value">--</div>
      <div class="label">Velocidad filtrada (°/s)</div>
      <div id="m2_w" class="value">--</div>
      <div class="label">Posición (°)</div>
      <div id="m2_p" class="value">--</div>
    </div>
  </div>

  <script>
    const es = new EventSource('/events');
    es.onmessage = (e) => {
      try {
        const data = JSON.parse(e.data);
        document.getElementById('m1_v').textContent = data.m1_v.toFixed(2);
        document.getElementById('m1_w').textContent = data.m1_w.toFixed(2);
        document.getElementById('m1_p').textContent = data.m1_p.toFixed(1);
        document.getElementById('m2_v').textContent = data.m2_v.toFixed(2);
        document.getElementById('m2_w').textContent = data.m2_w.toFixed(2);
        document.getElementById('m2_p').textContent = data.m2_p.toFixed(1);
      } catch (err) { console.log("Error al parsear:", err, e.data); }
    };
  </script>
</body>
</html>
)=====";

/* ====== FUNCIONES ====== */

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

void env_volt(double vp, uint8_t motor_id) {
  uint32_t Duty;
  uint8_t IN1, IN2, ledchannel;
  if (motor_id == 1) { IN1 = AIN1; IN2 = AIN2; ledchannel = 1; }
  else { IN1 = BIN2; IN2 = BIN1; ledchannel = 2; }

  if (vp > VM) vp = VM;
  else if (vp < -VM) vp = -VM;

  if (vp >= 0) {
    Duty = (uint32_t)((1-vp/VM)*1023);
    ledcWrite(ledchannel,1023-Duty);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    Duty = (uint32_t)((1+vp/VM)*1023);
    ledcWrite(ledchannel,1023-Duty);
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  }
}

void low_pass_filter(void *pvParameters) {
  MotorVars *m = (MotorVars *)pvParameters;
  while (1) {
    double x_k = m->vel_ang_median;
    double y_k = (1.0 / (wc * Tf + 1.0)) * (m->y_k_1 + wc * Tf * x_k);
    m->y_k_1 = y_k;
    m->vel_ang_filt = y_k;
    vTaskDelay(5);
  }
}

void enviar_voltaje(void *pvParameters) {
  MotorVars *m = (MotorVars *)pvParameters;
  m->vmotor = 9.0;
  if (m->id == 1) env_volt(m->vmotor, 1);
  else env_volt(m->vmotor, 2);
  while (1) vTaskDelay(25);
}

void handleRoot() { server.send_P(200, "text/html", MAIN_page); }

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
  ledcSetup(1,20000,10);
  ledcSetup(2,20000,10);
  ledcAttachPin(PWMA,1);
  ledcAttachPin(PWMB,2);

  attachInterrupt(EncAM1, ISR_FUN_M1, RISING);
  attachInterrupt(EncAM2, ISR_FUN_M2, RISING);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("Conectado! IP: ");
  Serial.println(WiFi.localIP());

  // WebServer
  server.on("/", handleRoot);
  server.on("/events", []() {
    sseClient = server.client();
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/event-stream");
    sseConnected = true;
    Serial.println("Cliente SSE conectado");
  });
  server.begin();
  Serial.println("Servidor HTTP iniciado");

  // Tasks
  xTaskCreatePinnedToCore(low_pass_filter, "LPF_M1", 4000, &motor1, 1, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(low_pass_filter, "LPF_M2", 4000, &motor2, 1, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(enviar_voltaje, "Enviar_Voltaje_M1", 4000, &motor1, 2, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(enviar_voltaje, "Enviar_Voltaje_M2", 4000, &motor2, 2, NULL, APP_CPU_NUM);

  Serial.println("Setup done!");
}

/* ====== LOOP ====== */
void loop() {
  server.handleClient();
  unsigned long now = millis();
  if (now - lastSSESend >= 500) {
    lastSSESend = now;
    if (sseConnected && sseClient.connected()) {
      String payload = "{";
      payload += "\"m1_v\":" + String(motor1.vmotor, 3) + ",";
      payload += "\"m1_w\":" + String(motor1.vel_ang_filt, 3) + ",";
      payload += "\"m1_p\":" + String(motor1.posicion, 3) + ",";
      payload += "\"m2_v\":" + String(motor2.vmotor, 3) + ",";
      payload += "\"m2_w\":" + String(motor2.vel_ang_filt, 3) + ",";
      payload += "\"m2_p\":" + String(motor2.posicion, 3);
      payload += "}";
      sseClient.print("data: "); sseClient.print(payload); sseClient.print("\n\n"); sseClient.flush();
    } else sseConnected = false;
  }
}