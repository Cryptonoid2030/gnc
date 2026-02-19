#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
// SDA A4 - SCL A5 sensor pinout

float reference = 0; 
float samplePer = 0.06; 


float kp = 1.0;
float kd = 1.0;

float threshold = 5; 
float fc = 5; 
// float alpha = 0.2; 
float alpha = samplePer / (samplePer + 1/(2*PI*fc)); 


float errors[3] = {0, 0, 0};
int counter = 0; 

void setup(void) {
  Serial.begin(9600);
  if(!bno.begin()) {
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void updateError(float newValue, float values[]) {
  values[2] = values[1];
  values[1] = values[0];
  values[0] = newValue;
}

void loop(void) {
  unsigned long iterationStart = millis();

  sensors_event_t event; 
  bno.getEvent(&event);

  float angle = event.orientation.x; 
  float error = angle - reference;
  
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (counter >= 2) {
    updateError(error, errors);

    // now define the control action
    float u = alpha * (kp * errors[0] + (kd / samplePer) * (errors[0] - errors[1])) +
              (1 - alpha) * (kp * errors[1] + (kd / samplePer) * (errors[1] - errors[2]));

    if (abs(u) > threshold) {
      if (u > 0) {
        Serial.println("Right on"); // connect this to a solenoid GPIO
      } else {
        Serial.println("Left on"); // connect this to a solenoid GPIO 
      }
    }
  } else {
    errors[2 - counter] = error;
    counter++;
  }

  while ((millis() - iterationStart) < samplePer * pow(10, 3)) {
    delay(1); // small delay (may have to increase for faster sampling)
  }
}
