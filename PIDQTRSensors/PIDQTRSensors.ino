#include <QTRSensors.h>
#include <AFMotor.h>

QTRSensors qtr;

// QTR Sensor setup
#define NUM_SENSORS 6
#define NUM_SAMPLES_PER_SENSOR 3
#define EMITTER_PIN 2
#define BLACK_THRESHOLD 100

qtr.setTypeA();
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor Driver Setup
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);

// PID vars
const double KP = 1000;
const double KD = 1000;
double lastError = 1000;
const int GOAL = 5500;
const unsigned char MAX_SPEED = 255;

int P;
int I;
int D;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  calibrateLineSensor();
  Serial.println("finished calibrating");
}

void loop() {
  if (sensorValues[2] > BLACK_THRESHOLD && sensorValues[3] > BLACK_THRESHOLD || sensorValues[0] > BLACK_THRESHOLD && sensorValues[1] > BLACK_THRESHOLD && sensorValues[4] > BLACK_THRESHOLD && sensorValues[5] > BLACK_THRESHOLD){
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    delay(200);
  }
  Serial.println(sensorValues[NUM_SENSORS]);
  // put your main code here, to run repeatedly:
  unsigned int position = qtra.readLine(sensorValues);

  // compute error from line
  int error = GOAL - position;

  // computes motor adustment
  int adjustment = KP*error + KD*(error - lastError);

  // store error for next increment
  lastError = error;

  // adjust motors
  motor2.setSpeed(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));
  motor3.setSpeed(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
}

void calibrateLineSensor(){
  delay(500);
  Serial.println(sensorValues[NUM_SENSORS]);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 3000; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}