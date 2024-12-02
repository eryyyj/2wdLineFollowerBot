#include <QTRSensors.h>

int NUM_SENSORS = 6;
#define NUM_SAMPLES_PER_SENSOR 3
#define EMITTER_PIN 2

QTRSensorsRC qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
