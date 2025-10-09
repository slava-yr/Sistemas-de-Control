#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define WIFI_SSID "REDPUCP"
#define WIFI_PASSWORD "C9AA28BA93"
#define IN1 32
#define IN2 33
#define SLEEP 25
#define CH_A 34
#define CH_B 35

WiFiClient espClient;
PubSubClient client(espClient);

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
double vel_deseada; 
double posicion; 
double pos_r; // Posición deseada recibida por MQTT
double tetad = 100; // Posición deseada

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

void Mqtt_Callback(char* topicx, byte* Data, unsigned int DataLen){
	String RecievedData = String((char*)Data, DataLen);
	String Topic = String((char*)topicx);

	if (Topic == "xspaceserver/prueba"){
    tetad = RecievedData.toDouble();
		Serial.println(RecievedData);
	}
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

void lazo_interno(void *pvParameters) // PI velocity control
{
  double t = 0;
  double T = 0.01;
  double Kp_i = 0.0597;
  double Ti = 0.1;

  // Variables de la regla de control
  double I_ant = 0;
  double e = 0;
  double I = 0;
  double u;

  while (1)
  {
    e = (vel_deseada - velangf2); // error de velocidad
    I = I_ant + e*T;

    u = Kp_i*(e+ (1/Ti)*I);
    env_volt(u);

    char msg_temp[20];
    client.loop(); // Mantener la conexión MQTT activa
    sprintf(msg_temp, "%f", posicion);
    client.publish("the_xspacer/mivalor", msg_temp);
    
    I_ant = I;
    Serial.print(t);
    Serial.print("  ");
    Serial.print(u);
    Serial.print("  ");
    Serial.print(velangf2); // Show filtered speed
    Serial.print("  ");
    Serial.println(posicion); // Show position
    t=t+10;
    vTaskDelay(10);
  }
}

void lazo_externo(void *pvParameters)
{
  double e;
  double Kp_e = 1.61; // Ganancia proporcional externa

  while(1)
  {
    e = (tetad - posicion); // errar en la posición

    vel_deseada = Kp_e*e; // Señal de referencia para el lazo interno
    vTaskDelay(30); // Período de muestreo de 30 ms 
  }
}

void setup() 
{
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

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
  	delay(1000);
  	Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi ");

  // MQTT
  client.setServer("www.xspace.pe", 1883); 
	while (!client.connected())
  { 
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client")) 
      Serial.println("connected");
    else
    {
      Serial.print("failed with state ");
    	Serial.print(client.state());
      delay(1000);
    }
  }

  client.setCallback(Mqtt_Callback);
	client.subscribe("xspaceserver/prueba");

  xTaskCreatePinnedToCore(filtrar_vel," ", 4000, NULL, 3 , NULL, 1); // Low Pass Filter
  xTaskCreatePinnedToCore(lazo_interno," ", 4000, NULL, 2 , NULL, 1); // PI speed control
  xTaskCreatePinnedToCore(lazo_externo," ", 4000, NULL, 2 , NULL, 1); // Proportional position control
}

void loop() {}

