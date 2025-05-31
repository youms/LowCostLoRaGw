/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-gas-sensor
 */

//#define AO_PIN A0  // Arduino's pin connected to AO pin of the MQ2 sensor
#define Threshold 400

#define MQ2pin 0

float sensorValue;  //variable to store sensor value

void setup() {
  // initialize serial communication
  Serial.begin(38400);
  Serial.println("Warming up the MQ2 sensor");
  //delay(20000);  // wait for the MQ2 to warm up
}

void loop() {
  sensorValue = analogRead(MQ2pin);  // read analog input pin 0

  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);

  if (sensorValue > Threshold) {
    Serial.print(" | Gas detected!");
  }

  Serial.println("");
  delay(2000);  // wait 2s for next reading
}
