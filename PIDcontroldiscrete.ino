#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float reference = 0; // euler angle for reference
float samplePer = 0.06; // select a sampling period consistent > clk & sensor sampling

// tuneable values
float kp = 1.0;
float kd = 1.0;

float threshold = 5; // sensitivity before controller action
float fc = 5; // 5 hz cutoff low pass filter
// float alpha = 0.2; // hard coded low pass filter
float alpha = samplePer / (samplePer + 1/(2*PI*fc)); // automatic low pass filter based on cutoff frequency

// loop variables (no need to change)
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
    float u = alpha * (kp * errors[0] + (kd / samplePer) * (errors[0] - errors[1])) + // current error
              (1 - alpha) * (kp * errors[1] + (kd / samplePer) * (errors[1] - errors[2])); // previous error

    if (abs(u) > threshold) {
      if (u > 0) {
        Serial.println("Right on"); // connect this to a solenoid GPIO
      } else {
        Serial.println("Left on"); // connect this to a solenoid GPIO 
      }
    }
  } else {
    // error buffer is empty. start filling now
    errors[2 - counter] = error;
    counter++;
  }

  while ((millis() - iterationStart) < samplePer * pow(10, 3)) {
    delay(1); // small delay (may have to increase for faster sampling)
  }
}