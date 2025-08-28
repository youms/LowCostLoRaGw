#include "my_temp_sensor_code.h"

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NOMEMCLATURE, HERE TC WOULD MEAN TEMPERATURE IN CELCIUS FOR INSTANCE
// USE A MAXIMUM OF 3 CHARACTERS
// 
// The node will be sending for instance \!TC/23.5 

char nomenclature_str[4]="TC";
//////////////////////////////////////////////////////////////////

// DS18B20 instances
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

///////////////////////////////////////////////////////////////////
// ADD HERE SOME INITIALIZATION CODE
// HERE WE JUST DECLARE VALUE_PIN_READ AS INPUT PIN

// DS18B20 INITIALIZATION
void sensor_Init() {
  // Initialize DS18B20
  ds18b20.begin();
  
  // Optional: Set resolution (9-12 bits, 12 = highest accuracy)
  ds18b20.setResolution(12);
  
  // Optional: Power control pin
 // pinMode(PIN_POWER, OUTPUT);
  
  Serial.println(F("DS18B20 initialized"));
  Serial.print(F("Found "));
  Serial.print(ds18b20.getDeviceCount());
  Serial.println(F(" DS18B20 sensors"));
}
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE WAY YOU READ A VALUE FROM YOUR SPECIFIC SENSOR
// HERE IT IS AN EXAMPLE WITH THE TMP36 SIMPLE ANALOG TEMPERATURE SENSOR

// READ DS18B20 TEMPERATURE
double sensor_getValue() {
  // Power on sensor if using power control
 // digitalWrite(PIN_POWER, HIGH);
 // delay(100);  // Wait for sensor to power up
  
  // Request temperature reading
  ds18b20.requestTemperatures();
  
  // Read temperature in Celsius
  double sensor_value = ds18b20.getTempCByIndex(0);
  
  // Check if reading is valid
  if (sensor_value == DEVICE_DISCONNECTED_C) {
    Serial.println(F("DS18B20 disconnected or error"));
    sensor_value = -999.0;  // Error value
  } else {
    Serial.print(F("DS18B20 reading: "));
    Serial.print(sensor_value);
    Serial.println(F("°C"));
  }
  
  // Power off sensor if using power control
 // digitalWrite(PIN_POWER, LOW);
  
  return sensor_value;
}
///////////////////////////////////////////////////////////////////

float random_value() {
  int wholePart = random(20, 29); // Generates 20, 21, ..., 28
  int decimalPart = random(0, 100); // Generates 0, 1, ..., 99
  float sensor_value = (float)wholePart + ((float)decimalPart / 100.0);

  return sensor_value;
}
