#ifndef MY_GAS_SENSOR_CODE
#define MY_GAS_SENSOR_CODE

#include "Arduino.h"

// MQ-9 Gas Sensor functions
void gas_sensor_Init();
float gas_sensor_getValue();  // Returns CO concentration in ppm
int gas_sensor_getLPG();      // Returns LPG concentration in ppm  
int gas_sensor_getMethane();  // Returns CH4 concentration in ppm
int gas_sensor_getPropane();  // Returns C3H8 concentration in ppm
int gas_sensor_getHydrogen(); // Returns H2 concentration in ppm
int gas_sensor_getSmoke();    // Returns Smoke concentration in ppm
float gas_sensor_getRsRoRatio(); // Returns Rs/Ro ratio
bool gas_sensor_isSafe();     // Returns safety assessment
void gas_sensor_getFallbackValues(); // Returns clean air fallback values

///////////////////////////////////////////////////////////////////
// MQ-9 SENSOR PIN AND PARAMETERS CONFIGURATION
#define MQ9_PIN A0                    // Analog pin for MQ-9 sensor
#define RL_VALUE 5.0                  // Load resistance in KOhms
#define RO_CLEAN_AIR_FACTOR 9.9       // Rs/Ro ratio in clean air for LPG
#define CALIBRATION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5

// Gas type constants
#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2
#define GAS_METHANE 3
#define GAS_PROPANE 4
#define GAS_HYDROGEN 5

// Safety thresholds (ppm)
#define CO_DANGER_THRESHOLD 1000
#define CO_WARNING_THRESHOLD 200
#define CO_CAUTION_THRESHOLD 50
#define COMBUSTIBLE_DANGER_THRESHOLD 10000
#define COMBUSTIBLE_WARNING_THRESHOLD 5000
#define COMBUSTIBLE_CAUTION_THRESHOLD 1000
///////////////////////////////////////////////////////////////////

#if defined ARDUINO_AVR_PRO || defined ARDUINO_AVR_MINI || defined ARDUINO_SAM_DUE || defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__ || defined __SAMD21G18A__
  // if you have a Pro Mini running at 5V, then change here
  // these boards work in 3.3V
  #define VOLTAGE_SCALE 3300.0
#else // ARDUINO_AVR_NANO || defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560 
  // also for all other boards, so change here if required.
  #define VOLTAGE_SCALE 5000.0
#endif

// Global variables (declared as extern for use across files)
extern float Ro_gas;  // Sensor resistance in clean air

// Internal function declarations
float MQCalibration(int mq_pin);
float MQRead(int mq_pin);
float MQResistanceCalculation(int raw_adc);
int MQGetGasPercentage(float rs_ro_ratio, int gas_id);
int MQGetPercentage(float rs_ro_ratio, float *curve);
void assessSafety(int co_ppm, int total_combustible_ppm);
bool is_sensor_connected();  // Sensor connection detection

#endif