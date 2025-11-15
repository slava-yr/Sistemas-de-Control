/* 
  Obtener la resolución del encoder dando 1 vuelta al motor.
  El 09/10/25 se realizaron 10 pruebas.
  La media es 1081.7, mediana 1081.
  Se toma resolución de 1081 pulsos por vuelta.
*/


#include <Arduino.h> 
/* Encoder Pins */
// Encoder motor 1 
#define EncAM1 14
#define EncBM1 27
// Encoder motor 2
#define EncAM2 26
#define EncBM2 25

int32_t counter = 0;
void IRAM_ATTR ISR_FUN_M1() // Calcula el periodo
{
  uint8_t a = digitalRead(EncAM2);
  uint8_t b = digitalRead(EncBM2);
  Serial.print(a);
  Serial.print("  ");
  Serial.println(b);
  if (a)
    counter++;
  else
    counter--;
}

void print_counter(void *pvParameters)
{    
  while(1)
  {
    Serial.println(counter);
    vTaskDelay(500);
  }
}

void setup() {
  pinMode(EncAM2, INPUT_PULLUP);
  pinMode(EncBM2, INPUT_PULLUP);
  attachInterrupt(EncBM2, ISR_FUN_M1, RISING); // Triggered by encoder

  // xTaskCreatePinnedToCore(print_counter, "", 4000, NULL, 1, NULL, APP_CPU_NUM);
  Serial.begin(115200);
  Serial.println("Setup done!");
}




void loop() {
}