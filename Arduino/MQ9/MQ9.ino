/*
 * MQ-9 Gas Sensor Serial Output
 * Detects CO, Methane, LPG, Propane, and Hydrogen
 * Outputs gas concentrations to serial monitor
 * 
 * Sensor operates in dual-temperature cycle:
 * - Low temp (1.5V): Detects CO
 * - High temp (5.0V): Detects combustible gases
 */

#define SENSOR_PIN A0          // Analog pin for MQ-9 sensor
#define RL_VALUE 5.0           // Load resistance in KOhms
#define RO_CLEAN_AIR_FACTOR 9.9 // Rs/Ro ratio in clean air for LPG
#define CALIBARAION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5

// Gas types
#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2
#define GAS_METHANE 3
#define GAS_PROPANE 4
#define GAS_HYDROGEN 5

// Power regression coefficients from MQ-9 datasheet curves
// Formula: ppm = a * (Rs/R0)^b
// Format: {a, b} where a = scale factor, b = exponent
float LPGCurve[2] = {987.26, -2.162};      // LPG: y = 987.26 * x^(-2.162)
float COCurve[2] = {605.18, -3.937};       // CO: y = 605.18 * x^(-3.937)  
float SmokeCurve[2] = {143.01, -2.186};    // Smoke: y = 143.01 * x^(-2.186)
float MethaneCurve[2] = {2217.8, -2.827};  // CH4: y = 2217.8 * x^(-2.827)
float PropaneCurve[2] = {614.56, -2.564};  // C3H8: y = 614.56 * x^(-2.564)
float HydrogenCurve[2] = {988.05, -1.767}; // H2: y = 988.05 * x^(-1.767)

float Ro = 10;  // Sensor resistance in clean air (will be calibrated)

void setup() {
  Serial.begin(9600);
  Serial.println("=== MQ-9 Gas Sensor Initialization ===");
  Serial.println("Warming up sensor...");
  
  // Calibrate sensor in clean air
  Serial.print("Calibrating sensor in clean air...");
  Ro = MQCalibration(SENSOR_PIN);
  Serial.print("Ro = ");
  Serial.print(Ro);
  Serial.println(" KOhm");
  
  if (Ro == 0) {
    Serial.println("ERROR: Calibration failed! Check connections.");
    while(1); // Stop execution
  }
  
  Serial.println("Sensor ready!");
  Serial.println("===========================================");
  Serial.println();
  delay(2000);
}

void loop() {
  float rs_ro_ratio = MQRead(SENSOR_PIN) / Ro;
  
  // Display raw sensor data
  Serial.println("--- MQ-9 Gas Sensor Readings ---");
  Serial.print("Raw ADC: ");
  Serial.println(analogRead(SENSOR_PIN));
  
  float sensor_volt = analogRead(SENSOR_PIN) * (5.0 / 1023.0);
  Serial.print("Voltage: ");
  Serial.print(sensor_volt, 3);
  Serial.println(" V");
  
  Serial.print("Rs/Ro Ratio: ");
  Serial.println(rs_ro_ratio, 3);
  
  // Calculate gas concentrations
  Serial.println("\n--- Gas Concentrations ---");
  
  // CO detection (primary detection at low temp)
  int co_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_CO);
  Serial.print("CO: ");
  Serial.print(co_ppm);
  Serial.println(" ppm");
  
  // LPG detection
  int lpg_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_LPG);
  Serial.print("LPG: ");
  Serial.print(lpg_ppm);
  Serial.println(" ppm");
  
  // Methane detection (high temp)
  int methane_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_METHANE);
  Serial.print("CH4: ");
  Serial.print(methane_ppm);
  Serial.println(" ppm");
  
  // Propane detection (high temp)
  int propane_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_PROPANE);
  Serial.print("C3H8: ");
  Serial.print(propane_ppm);
  Serial.println(" ppm");
  
  // Hydrogen detection
  int hydrogen_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_HYDROGEN);
  Serial.print("H2: ");
  Serial.print(hydrogen_ppm);
  Serial.println(" ppm");
  
  // Smoke detection
  int smoke_ppm = MQGetGasPercentage(rs_ro_ratio, GAS_SMOKE);
  Serial.print("Smoke: ");
  Serial.print(smoke_ppm);
  Serial.println(" ppm");
  
  // Safety assessment
  Serial.println("\n--- Safety Assessment ---");
  assessSafety(co_ppm, lpg_ppm + methane_ppm + propane_ppm + hydrogen_ppm);
  
  Serial.println("=====================================\n");
  delay(3000); // Update every 3 seconds
}

/*
 * Calibrate the sensor in clean air
 * Returns the resistance of sensor in clean air (Ro)
 */
float MQCalibration(int mq_pin) {
  int i;
  float val = 0;
  
  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;
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
  if (co_ppm > 1000) {
    Serial.println("⚠️  DANGER: CO CRITICAL - Immediate evacuation required!");
    danger = true;
  } else if (co_ppm > 200) {
    Serial.println("⚠️  WARNING: High CO detected");
    warning = true;
  } else if (co_ppm > 50) {
    Serial.println("⚠️  CAUTION: Elevated CO levels");
  }
  
  // Combustible gas thresholds
  if (total_combustible_ppm > 10000) {
    Serial.println("🔥 DANGER: Explosive gas concentration!");
    danger = true;
  } else if (total_combustible_ppm > 5000) {
    Serial.println("🔥 WARNING: High combustible gas levels");
    warning = true;
  } else if (total_combustible_ppm > 1000) {
    Serial.println("🔥 CAUTION: Combustible gas detected");
  }
  
  if (!danger && !warning && co_ppm <= 50 && total_combustible_ppm <= 1000) {
    Serial.println("✅ SAFE: Air quality normal");
  }
  
  // Additional info
  Serial.print("Total combustible: ");
  Serial.print(total_combustible_ppm);
  Serial.println(" ppm");
}