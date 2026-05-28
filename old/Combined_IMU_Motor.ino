#include <Wire.h>

#define BMI160_ADDR 0x68
#define CMD_REG    0x7E
#define ACC_X_LSB  0x12

float baseX = 0;
float baseY = 0;
float baseZ = 0;

#define RPWM_PIN  13
#define LPWM_PIN  14
#define R_EN_PIN  11
#define L_EN_PIN  12

#define POT_PIN   4

#define START_BTN_PIN  35
#define STOP_BTN_PIN   36
#define DIR_BTN_PIN    37

#define PWM_FREQ  1000
#define PWM_RES   8

bool motorRunning = false;
bool motorForwardDirection = true;

void writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom((uint16_t)BMI160_ADDR, (uint8_t)2);

  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();

  return (int16_t)((msb << 8) | lsb);
}

float rawToG(int16_t raw) {
  return raw / 16384.0;
}

void calibrateBaseline() {
  const int sampleCount = 100;

  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;

  Serial.println("Calibrating IMU baseline...");
  Serial.println("Jangan gerakkan sensor...");

  for (int i = 0; i < sampleCount; i++) {
    int16_t accX_raw = read16(ACC_X_LSB);
    int16_t accY_raw = read16(ACC_X_LSB + 2);
    int16_t accZ_raw = read16(ACC_X_LSB + 4);

    sumX += rawToG(accX_raw);
    sumY += rawToG(accY_raw);
    sumZ += rawToG(accZ_raw);

    delay(10);
  }

  baseX = sumX / sampleCount;
  baseY = sumY / sampleCount;
  baseZ = sumZ / sampleCount;

  Serial.println("Baseline selesai:");
  Serial.print("baseX: "); Serial.print(baseX, 3);
  Serial.print(" g | baseY: "); Serial.print(baseY, 3);
  Serial.print(" g | baseZ: "); Serial.print(baseZ, 3);
  Serial.println(" g");
}

void motorStop() {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
}

void motorForward(int speedPWM) {
  ledcWrite(RPWM_PIN, speedPWM);
  ledcWrite(LPWM_PIN, 0);
}

void motorReverse(int speedPWM) {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, speedPWM);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(40, 41);

  writeRegister(CMD_REG, 0x11);
  delay(100);
  Serial.println("BMI160 Started");

  calibrateBaseline();

  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(STOP_BTN_PIN, INPUT_PULLUP);
  pinMode(DIR_BTN_PIN, INPUT_PULLUP);

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RES);

  motorStop();

  Serial.println("Motor Control Initialized.");
}

void loop() {
  int16_t accX_raw = read16(ACC_X_LSB);
  int16_t accY_raw = read16(ACC_X_LSB + 2);
  int16_t accZ_raw = read16(ACC_X_LSB + 4);

  float accX_g = rawToG(accX_raw);
  float accY_g = rawToG(accY_raw);
  float accZ_g = rawToG(accZ_raw);

  float deltaX = accX_g - baseX;
  float deltaY = accY_g - baseY;
  float deltaZ = accZ_g - baseZ;

  float vibration = sqrt(
    deltaX * deltaX +
    deltaY * deltaY +
    deltaZ * deltaZ
  );

  float roll = atan2(accY_g, accZ_g) * 180.0 / PI;
  float pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / PI;

  float accX_ms2 = accX_g * 9.80665;
  float accY_ms2 = accY_g * 9.80665;
  float accZ_ms2 = accZ_g * 9.80665;

  int potValue = analogRead(POT_PIN);
  int pwmValue = map(potValue, 0, 4095, 0, 180);

  if (digitalRead(START_BTN_PIN) == LOW) {
    motorRunning = true;
    Serial.println("START pressed");
    delay(300);
  }

  if (digitalRead(STOP_BTN_PIN) == LOW) {
    motorRunning = false;
    motorStop();
    Serial.println("STOP pressed");
    delay(300);
  }

  if (digitalRead(DIR_BTN_PIN) == LOW) {
    motorRunning = false;
    motorStop();

    Serial.println("DIR pressed - motor stopped before changing direction");
    delay(1000);

    motorForwardDirection = !motorForwardDirection;

    Serial.print("Direction changed to: ");
    Serial.print(motorForwardDirection ? "FORWARD" : "REVERSE");

    delay(300);
  }

  if (motorRunning) {
    if (motorForwardDirection) {
      motorForward(pwmValue);
    } else {
      motorReverse(pwmValue);
    }
  } else {
    motorStop();
  }

  Serial.print("Data Accelerometer BaseLine!!\n");
  if (vibration >= 0.03) {
    Serial.print("GETARAN | ");
  } else {
    Serial.print("DIAM    | ");
  }

  Serial.print("vibration: ");
  Serial.print(vibration, 3);
  Serial.print(" g | ");

  Serial.print("dX: ");
  Serial.print(deltaX, 3);
  Serial.print(" | dY: ");
  Serial.print(deltaY, 3);
  Serial.print(" | dZ: ");
  Serial.println(deltaZ, 3);

  Serial.print("Data Accelerometer m/s², pitch, and roll!!\n");
  Serial.print("Acc X: ");
  Serial.print(accX_ms2, 2);
  Serial.print(" m/s2 | Y: ");
  Serial.print(accY_ms2, 2);
  Serial.print(" m/s2 | Z: ");
  Serial.print(accZ_ms2, 2);
  Serial.print(" m/s2");

  Serial.print(" || Pitch: ");
  Serial.print(pitch, 2);
  Serial.print(" deg | Roll: ");
  Serial.print(roll, 2);
  Serial.println(" deg");

  Serial.print("Pot: ");
  Serial.print(potValue);
  Serial.print(" | PWM: ");
  Serial.print(pwmValue);
  Serial.print(" | Motor: ");
  Serial.print(motorRunning ? "RUNNING" : "STOP");
  Serial.print(" | Direction: ");
  Serial.println(motorForwardDirection ? "FORWARD" : "REVERSE");

  delay(100);
}
