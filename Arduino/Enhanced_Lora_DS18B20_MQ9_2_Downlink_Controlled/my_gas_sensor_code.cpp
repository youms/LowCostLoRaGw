#include "Arduino.h"
#include "my_gas_sensor_code.h"

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NOMENCLATURE, HERE GAS WOULD MEAN GAS SENSOR READING
// USE A MAXIMUM OF 3 CHARACTERS
// 
// The node will be sending for instance \!GAS/150 (CO ppm)

char gas_nomenclature_str[4] = "GAS";
//////////////////////////////////////////////////////////////////

// Power regression coefficients from MQ-9 datasheet curves
// Formula: ppm = a * (Rs/R0)^b
// Format: {a, b} where a = scale factor, b = exponent
float LPGCurve[2] = {987.26, -2.162};      // LPG: y = 987.26 * x^(-2.162)
float COCurve[2] = {605.18, -3.937};       // CO: y = 605.18 * x^(-3.937)  
float SmokeCurve[2] = {143.01, -2.186};    // Smoke: y = 143.01 * x^(-2.186)
float MethaneCurve[2] = {2217.8, -2.827};  // CH4: y = 2217.8 * x^(-2.827)
float PropaneCurve[2] = {614.56, -2.564};  // C3H8: y = 614.56 * x^(-2.564)
float HydrogenCurve[2] = {988.05, -1.767}; // H2: y = 988.05 * x^(-1.767)

float Ro_gas = 10;  // Sensor resistance in clean air (will be calibrated)

///////////////////////////////////////////////////////////////////
// ADD HERE SOME INITIALIZATION CODE
// MQ-9 GAS SENSOR INITIALIZATION
void gas_sensor_Init() {
  Serial.println(F("=== MQ-9 Gas Sensor Initialization ==="));
  Serial.println(F("30s: Warming up sensor..."));
  delay(30000);
  
  // Calibrate sensor in clean air
  Serial.print(F("Calibrating sensor in clean air..."));
  Ro_gas = MQCalibration(MQ9_PIN);
  Serial.print(F("Ro = "));
  Serial.print(Ro_gas);
  Serial.println(F(" KOhm"));
  
  // Check if Ro_gas is in realistic range for MQ-9 (based on datasheet: 2-30 KΩ typical)
  if (Ro_gas < 5.0 || Ro_gas > 50.0) {
    Serial.print(F("ERROR: Unrealistic Ro value ("));
    Serial.print(Ro_gas);
    Serial.println(F(" KOhm) - Sensor likely disconnected"));
    Ro_gas = 0;  // Mark as invalid to trigger fallback values
  } else {
    Serial.println(F("MQ-9 Gas sensor ready!"));
  }
  
  Serial.println(F("======================================="));
}
///////////////////////////////////////////////////////////////////

// Fallback values based on clean environment readings
struct GasFallbackValues {
  float co = 0.0;      // CO: 0 ppm
  int lpg = 7;         // LPG: 7 ppm  
  int methane = 3;     // CH4: 3 ppm
  int propane = 1;     // C3H8: 1 ppm
  int hydrogen = 17;   // H2: 17 ppm
  int smoke = 0;       // Smoke: 0 ppm
};

GasFallbackValues fallback_values;

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE WAY YOU READ A VALUE FROM YOUR SPECIFIC SENSOR
// PRIMARY SENSOR READING - RETURNS CO CONCENTRATION (main gas of interest)
float gas_sensor_getValue() {
  if (Ro_gas <= 0 || Ro_gas < 5.0 || Ro_gas > 50.0) {
    Serial.println(F("MQ-9 sensor disconnected/invalid - using fallback CO value"));
    return fallback_values.co;  // Return fallback CO value
  }
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  
  // Get CO concentration (primary detection)
  int co_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_CO);
  
  Serial.print(F("MQ-9 CO reading: "));
  Serial.print(co_ppm);
  Serial.println(F(" ppm"));
  
  return (float)co_ppm;
}

// GET LPG CONCENTRATION
int gas_sensor_getLPG() {
  if (Ro_gas <= 0 || Ro_gas < 20.0 || Ro_gas > 50.0) return fallback_values.lpg;
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  return MQGetGasPercentage(rs_ro_ratio, GAS_LPG);
}

// GET METHANE CONCENTRATION  
int gas_sensor_getMethane() {
  if (Ro_gas <= 0 || Ro_gas < 20.0 || Ro_gas > 50.0) return fallback_values.methane;
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  return MQGetGasPercentage(rs_ro_ratio, GAS_METHANE);
}

// GET PROPANE CONCENTRATION
int gas_sensor_getPropane() {
  if (Ro_gas <= 0 || Ro_gas < 20.0 || Ro_gas > 50.0) return fallback_values.propane;
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  return MQGetGasPercentage(rs_ro_ratio, GAS_PROPANE);
}

// GET HYDROGEN CONCENTRATION
int gas_sensor_getHydrogen() {
  if (Ro_gas <= 0 || Ro_gas < 20.0 || Ro_gas > 50.0) return fallback_values.hydrogen;
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  return MQGetGasPercentage(rs_ro_ratio, GAS_HYDROGEN);
}

// GET SMOKE CONCENTRATION
int gas_sensor_getSmoke() {
  if (Ro_gas <= 0 || Ro_gas < 20.0 || Ro_gas > 50.0) return fallback_values.smoke;
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  return MQGetGasPercentage(rs_ro_ratio, GAS_SMOKE);
}

// GET Rs/Ro RATIO FOR DEBUGGING
float gas_sensor_getRsRoRatio() {
  if (Ro_gas <= 0) return -999.0;
  
  return MQRead(MQ9_PIN) / Ro_gas;
}

// SAFETY ASSESSMENT
bool gas_sensor_isSafe() {
  if (Ro_gas <= 0) return false;
  
  float rs_ro_ratio = MQRead(MQ9_PIN) / Ro_gas;
  
  int co_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_CO);
  int lpg_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_LPG);
  int methane_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_METHANE);
  int hydrogen_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_HYDROGEN);
  
  int total_combustible = lpg_ppm + methane_ppm + hydrogen_ppm;
  
  // Check danger thresholds
  if (co_ppm > CO_DANGER_THRESHOLD || total_combustible > COMBUSTIBLE_DANGER_THRESHOLD) {
    return false;  // DANGER
  }
  
  // Check warning thresholds
  if (co_ppm > CO_WARNING_THRESHOLD || total_combustible > COMBUSTIBLE_WARNING_THRESHOLD) {
    return false;  // WARNING
  }
  
  return true;  // SAFE
}
///////////////////////////////////////////////////////////////////

/*
 * Calibrate the sensor in clean air
 * Returns the resistance of sensor in clean air (Ro)
 */
float MQCalibration(int mq_pin) {
  int i;
  float val = 0;
  
  for (i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBRATION_SAMPLE_TIMES;
  val = val / RO_CLEAN_AIR_FACTOR;
  return val;
}

/*
 * Read sensor resistance Rs
 */
float MQRead(int mq_pin) {
  int i;
  float rs = 0;
  
  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs / READ_SAMPLE_TIMES;
  return rs;
}

/*
 * Calculate sensor resistance from analog reading
 */
float MQResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/*
 * Get gas concentration in ppm using curve fitting
 */
int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if (rs_ro_ratio <= 0) return 0;
  
  switch (gas_id) {
    case GAS_LPG:
      return MQGetPercentage(rs_ro_ratio, LPGCurve);
    case GAS_CO:
      return MQGetPercentage(rs_ro_ratio, COCurve);
    case GAS_SMOKE:
      return MQGetPercentage(rs_ro_ratio, SmokeCurve);
    case GAS_METHANE:
      return MQGetPercentage(rs_ro_ratio, MethaneCurve);
    case GAS_PROPANE:
      return MQGetPercentage(rs_ro_ratio, PropaneCurve);
    case GAS_HYDROGEN:
      return MQGetPercentage(rs_ro_ratio, HydrogenCurve);
    default:
      return 0;
  }
}

/*
 * Calculate ppm using power regression formula: ppm = a * (Rs/Ro)^b
 * This is the standard manufacturer approach for MQ sensors
 */
int MQGetPercentage(float rs_ro_ratio, float *curve) {
  if (rs_ro_ratio <= 0) return 0;
  
  // Power regression formula: ppm = a * (Rs/Ro)^b
  // curve[0] = a (scale factor), curve[1] = b (exponent)
  float a = curve[0];
  float b = curve[1];
  
  int ppm = (int)(a * pow(rs_ro_ratio, b));
  return (ppm < 0) ? 0 : ppm;  // Ensure non-negative values
}

/*
 * Assess safety levels based on gas concentrations
 */
void assessSafety(int co_ppm, int total_combustible_ppm) {
  bool danger = false;
  bool warning = false;
  
  // CO safety thresholds
  if (co_ppm > CO_DANGER_THRESHOLD) {
    Serial.println(F("DANGER: CO CRITICAL - Immediate evacuation required!"));
    danger = true;
  } else if (co_ppm > CO_WARNING_THRESHOLD) {
    Serial.println(F("WARNING: High CO detected"));
    warning = true;
  } else if (co_ppm > CO_CAUTION_THRESHOLD) {
    Serial.println(F("CAUTION: Elevated CO levels"));
  }
  
  // Combustible gas thresholds
  if (total_combustible_ppm > COMBUSTIBLE_DANGER_THRESHOLD) {
    Serial.println(F("DANGER: Explosive gas concentration!"));
    danger = true;
  } else if (total_combustible_ppm > COMBUSTIBLE_WARNING_THRESHOLD) {
    Serial.println(F("WARNING: High combustible gas levels"));
    warning = true;
  } else if (total_combustible_ppm > COMBUSTIBLE_CAUTION_THRESHOLD) {
    Serial.println(F("CAUTION: Combustible gas detected"));
  }
  
  if (!danger && !warning && co_ppm <= CO_CAUTION_THRESHOLD && total_combustible_ppm <= COMBUSTIBLE_CAUTION_THRESHOLD) {
    Serial.println(F("SAFE: Air quality normal"));
  }
  
  // Additional info
  Serial.print(F("Total combustible: "));
  Serial.print(total_combustible_ppm);
  Serial.println(F(" ppm"));
}