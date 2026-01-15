 #ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// ========== ⚡ PRESET CALIBRATION MODE ⚡ ==========
const bool USE_PRESET_CALIBRATION = true; 

// ========== 📝 CALIBRATION VALUES 📝 ==========
const int PRESET_MIN_VALUES[8] = {580, 551, 511, 406, 523, 496, 426, 564};
const int PRESET_MAX_VALUES[8] = {957, 972, 970, 963, 969, 966, 965, 979};

// ========== ⚡ VOLTAGE COMPENSATION SETTINGS ⚡ ==========
const float x = 11.7;
const float REFERENCE_VOLTAGE = 4.05;           
const float CURRENT_BATTERY_VOLTAGE = x/3;     
const float VOLTAGE_MULTIPLIER = CURRENT_BATTERY_VOLTAGE / REFERENCE_VOLTAGE;

// ========== PIN DEFINITIONS ==========
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define MOTOR_STBY 5

#define CALIBRATE_BTN 11
#define START_BTN 12
#define STATUS_LED 13

// ========== CONFIGURATION ==========
const bool IS_BLACK_LINE = true;
const uint8_t NUM_SENSORS = 8;

// ========== PID PARAMETERS ==========
float Kp = 12.0;
float Kd = 15.0;
float Ki = 0.0;

// ========== BASE SPEED SETTINGS ==========
const int BASE_MAX_SPEED = 110;
const int BASE_MIN_SPEED = 40;
const int BASE_TURN_SPEED = 200;

// ========== COMPENSATED SPEEDS ==========
const int MAX_SPEED = (int)(BASE_MAX_SPEED * VOLTAGE_MULTIPLIER);
const int MIN_SPEED = (int)(BASE_MIN_SPEED * VOLTAGE_MULTIPLIER);
const int TURN_SPEED = (int)(BASE_TURN_SPEED * VOLTAGE_MULTIPLIER);

// ========== SENSOR CONFIGURATION ==========
const int8_t sensorWeight[NUM_SENSORS] = {20, 8, 2, 1, -1, -2, -8, -20};

// ========== GLOBAL VARIABLES ==========
int P, D, I;
int previousError = 0;
int PIDvalue = 0;
int lastPIDvalue = 0;
double error = 0;
double lastError = 0;

int leftSpeed, rightSpeed;
int currentSpeed = MIN_SPEED;

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int threshold[NUM_SENSORS];
int sensorValue[NUM_SENSORS];
bool sensorArray[NUM_SENSORS];

bool onLine = false;
int activeSensors = 0;

// ========== SETUP ==========
void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);
  
  pinMode(CALIBRATE_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  
  // Low battery warning
  if (CURRENT_BATTERY_VOLTAGE < 3.3) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(200);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
    }
  }
  
  if (USE_PRESET_CALIBRATION) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      minValues[i] = PRESET_MIN_VALUES[i];
      maxValues[i] = PRESET_MAX_VALUES[i];
      threshold[i] = (minValues[i] + maxValues[i]) / 2;
    }
    // Blink once to show ready
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
  }
}

// ========== MAIN LOOP ==========
void loop() {
  if (!USE_PRESET_CALIBRATION) {
    // Wait for calibration button
    while (digitalRead(CALIBRATE_BTN)) {
      delay(10);
    }
    delay(1000);
    
    digitalWrite(STATUS_LED, HIGH); // LED ON during calibration
    calibrateSensors();
    digitalWrite(STATUS_LED, LOW);  // LED OFF when done
    
    // Removed Serial printing of values
  }
  
  // Wait for start button
  while (digitalRead(START_BTN)) {
    delay(10);
  }
  delay(1000);
  
  currentSpeed = MIN_SPEED;
  previousError = 0;
  lastError = 0;
  lastPIDvalue = 0;
  I = 0;
  
  while (true) {
    readLineSensors();
    
    if (onLine) {
      // Only accelerate if error is small (straight line)
      if (abs(error) < 5 && currentSpeed < MAX_SPEED) {
        currentSpeed++;
      } else if (abs(error) > 10 && currentSpeed > MIN_SPEED) {
        currentSpeed -= 2; // Decelerate on curves
      }
      
      followLine();
      digitalWrite(STATUS_LED, HIGH);
    } else {
      // Immediately reduce speed when line is lost
      currentSpeed = MIN_SPEED;
      recoverLine();
      digitalWrite(STATUS_LED, LOW);
    }
  }
}

// ========== LINE FOLLOWING ==========
void followLine() {
  error = 0;
  activeSensors = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i];
      activeSensors++;
    }
  }
  
  if (activeSensors > 0) {
    error = error / activeSensors;
  }
  
  // Boost PID response for extreme errors (sharp turns)
  float adaptiveKp = Kp;
  float adaptiveKd = Kd;
  
  if (abs(error) > 10) {  // Sharp turn detected
    adaptiveKp = Kp * 1.5;  // Increase proportional gain
    adaptiveKd = Kd * 1.5;  // Increase derivative gain
    currentSpeed = MIN_SPEED + 10; // Force slowdown
  }
  
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (adaptiveKp * P) + (Ki * I) + (adaptiveKd * D);
  previousError = error;
  
  lastError = error;
  lastPIDvalue = PIDvalue;
  
  leftSpeed = currentSpeed + PIDvalue;
  rightSpeed = currentSpeed - PIDvalue;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  motor2run(leftSpeed);
  motor1run(rightSpeed);
}

// ========== RECOVERY (STRICTLY LAST KNOWN DIRECTION) ==========
void recoverLine() {
  // If line was last seen on the LEFT (lastError > 0)
  if (lastError > 0) {
    motor1run(0);          // Right motor OFF
    motor2run(TURN_SPEED); // Left motor TURN
  } 
  // If line was last seen on the RIGHT (lastError < 0)
  else if (lastError < 0) {
    motor1run(TURN_SPEED); // Right motor TURN
    motor2run(0);          // Left motor OFF
  } 
  else {
    motor1run(0);
    motor2run(TURN_SPEED);
  }
}

// ========== SENSOR FUNCTIONS ==========
void calibrateSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  unsigned long calibStart = millis();
  while (millis() - calibStart < 3000) {
    motor1run(70);
    motor2run(-70);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      int reading = analogRead(i);
      if (reading < minValues[i]) minValues[i] = reading;
      if (reading > maxValues[i]) maxValues[i] = reading;
    }
  }
  
  motor1run(0);
  motor2run(0);
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }
}

void readLineSensors() {
  onLine = false;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int rawValue = analogRead(i);
    
    if (IS_BLACK_LINE) {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 1000, 0);
    }
    
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > 500);
    
    if (sensorArray[i]) {
      onLine = true;
    }
  }
}

// ========== MOTOR CONTROL ==========
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}
