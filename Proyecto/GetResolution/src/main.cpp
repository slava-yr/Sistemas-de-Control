/* 
  Obtener la resolución del encoder dando 1 vuelta al motor.
  El 09/10/25 se realizaron 10 pruebas.
  La media es 1081.7, mediana 1081.
  Se toma resolución de 1081 pulsos por vuelta.
*/


#include <Arduino.h> 
#define M1_CH_A 15 // Encoder Motor 1

uint32_t counter = 0;
void IRAM_ATTR ISR_FUN_M1() // Calcula el periodo
{
  counter++;
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
  pinMode(M1_CH_A, INPUT_PULLUP);
  attachInterrupt(M1_CH_A, ISR_FUN_M1, RISING); // Triggered by encoder

  xTaskCreatePinnedToCore(print_counter, "", 4000, NULL, 1, NULL, APP_CPU_NUM);
  Serial.begin(115200);
  Serial.println("Setup done!");
}




void loop() {
}