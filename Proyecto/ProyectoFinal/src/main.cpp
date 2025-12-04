/*****************************
  Proyecto Sistemas de Control 2025-2
  HMI web TENSIÓN – TIEMPO – VELOCIDAD
  Motor IZQ (motor1) y Motor DER (motor2)

  Actualizado: 03/12/2025
*****************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

/* ====== CONFIGURAR WIFI ====== */
const char* ssid     = "Salvador";
const char* password = "candy123";

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

#define VM 11

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
  double vel_d;
}MotorVars;

// Variables globales de motores
MotorVars motor1 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1080, 0.014, 1, 0};
MotorVars motor2 = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1083, 0.0142, 2, 0};

// Variables globales de posición
double phid, phi;
double x, xd, xdp;
double y, yd, ydp;
bool fin_trayectoria;

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

  <div class="nombre">Grupo Alfa buena maravilla, super onda dinamita, escuadrón lobo.</div>

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

void IRAM_ATTR ISR_FUN_M1() { isr_func(&motor1, EncAM1, motor1.resolucion); }
void IRAM_ATTR ISR_FUN_M2() { isr_func(&motor2, EncAM2, motor2.resolucion); }

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

  m->vel_d = 200; 
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

void generadorTrayectoria(double t)
{
  double V_ref = 0.1; 
  double V_ref_g = 0.05;
  double distancia_s1 = 0.22; 
  double radio_s2 = 0.206; 
  double radio_s4 = 0.206; 
  double distancia_s5 = 0.40; 
  double radio_s6 = 0.125; 
  double radio_s7 = 0.175; 
  double distancia_s8 = 0.20; 

  double dt1, dt2, dt3, dt4, dt5, dt6, dt7; 
  double T1, T2, T3, T4, T5, T6, T7;

  dt1 = distancia_s1 / V_ref;
  dt2 = (PI * radio_s2) / V_ref_g;
  dt3 = (PI * radio_s4) / V_ref_g; 
  dt4 = distancia_s5 / V_ref;
  dt5 = (0.5 * PI * radio_s6) / V_ref_g; 
  dt6 = (0.5 * PI * radio_s7) / V_ref_g;
  dt7 = distancia_s8 / V_ref;

  T1 = dt1;
  T2 = T1 + dt2;
  T3 = T2 + dt3;
  T4 = T3 + dt4;
  T5 = T4 + dt5;
  T6 = T5 + dt6;
  T7 = T6 + dt7;

  double xf1 = distancia_s1; 
  double yf2 = radio_s2*2; 
  double yf3 = radio_s4*2 + yf2; 
  double xf4 = xf1 + distancia_s5; 
  double xf5 = xf4 + radio_s6;
  double yf5 = yf3 - radio_s6;
  
  // Calculated endpoints for the new segments
  double xf6 = xf5 + radio_s7; 
  double yf6 = yf5 - radio_s7; 
  double xf7 = xf6 + distancia_s8;

  double V=V_ref; 
  double t_local;
  double phi_p = 0;
  double theta_start;

  if (t <= T1)
  {
    xd = V_ref * t;
    yd = 0;
    xdp = V_ref; 
    ydp = 0; 
    phid = 0;
  }
  else if (t > T1 && t <= T2)
  {
    t_local = t - T1;
    phi_p = V_ref_g / radio_s2; 
    phid = phi_p * t_local; 
    xd = xf1 + radio_s2*sin(phid);
    yd = radio_s2 - radio_s2*cos(phid);
    xdp = V_ref_g*cos(phid); 
    ydp = V_ref_g*sin(phid);
  }
  else if (t > T2 && t <= T3)
  {
    t_local = t - T2;
    phi_p = V_ref_g / radio_s4;
    theta_start = 3*PI/2;
    phid = theta_start - phi_p * t_local; 
    xd = xf1 + radio_s4*cos(phid);
    yd = yf2 + radio_s4 + radio_s4*sin(phid);
    xdp = -radio_s4 * sin(phid) * (-phi_p);
    ydp = radio_s4 * cos(phid) * (-phi_p);
  }
  else if (t > T3 && t <= T4)
  {
    phid = 0; 
    t_local = t - T3;
    xd = xf1 + V_ref*t_local;
    yd = yf3;
    xdp = V_ref; 
    ydp = 0; 
  }
  else if (t > T4 && t <= T5)
  {
    t_local = t - T4;
    phi_p = -V_ref_g / radio_s6; 
    phid = 0 + phi_p*t_local; 
    xd = xf4 - radio_s6*sin(phid); 
    yd = (yf3 - radio_s6) + radio_s6*cos(phid);
    xdp = V_ref_g*cos(phid); 
    ydp = V_ref_g*sin(phid);
  }
  else if (t > T5 && t <= T6)
  {
    // Segment 6: Left Turn (CCW) from South to East
    t_local = t - T5;
    phi_p = V_ref_g/radio_s7; 
    
    // Angle on the circle (Starts at PI, goes to 3PI/2)
    double theta = PI + phi_p*t_local; 
    
    // Center is at (xf5 + r, yf5)
    xd = (xf5 + radio_s7) + radio_s7*cos(theta); 
    yd = yf5 + radio_s7*sin(theta);
    
    phid = -PI/2 + phi_p*t_local; // Robot heading (-90 to 0)
    xdp = V_ref_g*cos(phid); 
    ydp = V_ref_g*sin(phid);
  }  
  else if (t > T6 && t <= T7)
  {
    // Segment 7: Straight line East
    t_local = t - T6;
    phid = 0;
    xd = xf6 + V*t_local;
    yd = yf6;
    xdp = V; 
    ydp = 0; 
  }

  else
  {
    xd = xf7;
    yd = yf6;
    xdp = 0; 
    ydp = 0;
    phid = 0;  

    fin_trayectoria = true;
  }
}

void seguimientoTrayectoria(void *pvParameters)
{
  // Sigue la trayectoria definida por generadorTrayectoria
  // xd, xdp, yd, ydp, phi actualizados en generadorTrayectoria
  double a = 12.3/100; // En m
  double Tes = 1.5;
  double K1 = 4/Tes, K2 = 4/Tes;
  double x = 0, y = 0, av1 = 0, av2 = 0;

  double r = 6.75/200; // m
  double d = 24.6/100; // m 
  // referencias
  double U_ref, W_ref, wd_ref, wi_ref;
  // Errores
  double xe = 0; 
  double ye = 0;
 
  double t = 0; // t en segundos
  double dt = 100; // dt en ms

  double vrobot, wrobot, phi_est;
  while(1)
  {
    generadorTrayectoria(t); // Actualizar valores
    
    if (fin_trayectoria)
    {
      motor1.vel_d = 0;
      motor2.vel_d = 0;
    }

    else
    {
      // Estimaciones de posición y trayectoria
      vrobot = (r/2.0) * (motor2.vel_ang_median + motor1.vel_ang_median)*PI/180;
      wrobot = ((r/d) * (motor2.vel_ang_median - motor1.vel_ang_median))*PI/180;
      phi_est += wrobot * dt/1000;

      if (phi_est > PI) phi_est -= 2*PI;
      if (phi_est < -PI) phi_est += 2*PI;
      
      x += vrobot * cos(phi_est) * dt/1000;
      y += vrobot * sin(phi_est) * dt/1000;
      phi = phi_est;

      // Error
      xe = xd - x; 
      ye = yd - y;

      // Control de trayectoria
      av1 = K1*xe + xdp;
      av2 = K2*ye + ydp;

      U_ref = cos(phi)*av1 + sin(phi)*av2;
      W_ref = -sin(phi)/a*av1 + cos(phi)/a*av2;

      // Actualizar velocidad deseada para los motores
      motor1.vel_d = (U_ref*2/r - W_ref*d/r)/2*180/PI;
      motor2.vel_d = (U_ref*2/r + W_ref*d/r)/2*180/PI;
    }
    
    t += dt/1000; // Actualizar el tiempo
    vTaskDelay(dt); 
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
  json += "\"tension_der\":" + String(phi,       3) + ",";
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
  xTaskCreatePinnedToCore(seguimientoTrayectoria, "SeguimientoTrayectoria", 4000, NULL, 1, NULL, APP_CPU_NUM); // Seguimiento de la trayectoria
  // xTaskCreatePinnedToCore(printSpeed, "printSpeed", 4000, NULL, 1, NULL, APP_CPU_NUM); // Print speeds

  Serial.println("Setup done!");
}

/* ====== LOOP ====== */
void loop() {
  server.handleClient();
}