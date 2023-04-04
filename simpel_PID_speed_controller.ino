#include <Arduino_FreeRTOS.h>
#include "stream_buffer.h"

#define BUFFER_SIZE 128 // Buffer size for the serial
#define Q1 34; // Q1 pin number
#define Q2 35; // Q2 pin number

int prevtime=0;
float prev_e = 0;
float prev_int = 0;
int PWMvalue = 0;
float PID[3] = {0.5, 0.3, 0.5}


volatile int encoderCount = 0; // Initialize encoder count as a global variable

void encoderISR() {
  if (digitalRead(Q2) == HIGH) {
    encoderCount++; // Count up if pin Q2 is also high
  }
  else {
    encoderCount--; // Count down if pin Q2 is low
  }
}

// Create tasks for the freeRTOS
void read_task(void *pvParameter);
void motorControl(void *pvParameter)

StreamBufferHandle_t streamBuffer;

void setup() {
  // Create the streaming buffer
  streamBuffer = xStreamBufferCreate(BUFFER_SIZE, 1);
  
  // put your setup code here, to run once:
  pinMode(Q1, INPUT_PULLUP); // Set pin Q1 as input with pull-up resistor
  pinMode(Q2, INPUT_PULLUP); // Set pin Q2 as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(Q1), encoderISR, RISING); // Attach interrupt to pin Q1
  xTaskCreate(motorControl, "motorControl", 128, NULL, 1, NULL);
  xTaskCreate(read_task, "read_task", 128, NULL, 2, NULL);

 }


void loop() {
  // put your main code here, to run repeatedly:

}

static void UI(void* pvParameters)
{
  while(1)
  { 
    
  }
}



static void motorControl(void* pvParameters)
{
  while(1)
   { 
    unsigned long cur = millis();
    unsigned long dt = cur - prevtime;
    prevtime = cur;


    float circum=0.5; //in meter
    float encResolution = 360/3; // in degrees
    currSpeed =  (encoderCount*circum)/encResolution/(100/portTICK_PERIOD_MS);    
    
    float error = ref-currSpeed;
    float de=(error-prev_e)/dt;

    float integral_e = prev_int + dt*(e+e_prev)/2;
    prev_e = error;
    prev_int = integral_e;

    v_hat = PID[0]*error+PID[1]*de+PID[2]*integral_e;
    
    // Convert v_hat to PWM value
    
    vTaskDelay(100/portTICK_PERIOD_MS);
  } 

}

void read_task(void *parameter) {
  char buffer[BUFFER_SIZE];
  
  while (true) {
    // Read data from the buffer
    size_t bytesRead = xStreamBufferReceive(streamBuffer, buffer, BUFFER_SIZE, portMAX_DELAY);
    
    // Print the data to the serial console
    Serial.write(buffer, bytesRead);
  }
}