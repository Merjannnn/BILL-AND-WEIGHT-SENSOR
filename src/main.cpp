#include <Arduino.h>
#include <HX711.h>

#define DT_PIN 22
#define SCK_PIN 23

HX711 scale;
float calibration_factor = -21100.0;
float currentWeightKg = 0.0;
float targetWeightKg = 0.0;
bool weightReadingAvailable = false;
unsigned long lastWeightReadMs = 0;
const unsigned long WEIGHT_READ_INTERVAL_MS = 100;
const byte WEIGHT_AVERAGE_SAMPLES = 3;
float weightSamples[WEIGHT_AVERAGE_SAMPLES] = {0};
byte weightSampleCount = 0;
byte nextWeightSample = 0;

// ---- Bill / Coin Acceptor pins ----
const byte billPin = 3;
const byte coinPin = 2;

const unsigned long DEBOUNCE_US = 15000;    // 15ms bounce rejection
const unsigned long PULSE_TIMEOUT_MS = 300; // pulse train "done" after this silence

volatile int billPulse = 0;
volatile unsigned long lastBillEdge = 0;
volatile unsigned long lastBillPulseTime = 0;

volatile int coinPulse = 0;
volatile unsigned long lastCoinEdge = 0;
volatile unsigned long lastCoinPulseTime = 0;

long totalMoney = 0;

// ---- BTS7960 dispenser motors: one auger per rice type ----
#define RPWM1 5
#define LPWM1 6
#define R_EN1 7
#define L_EN1 8
#define RPWM2 9
#define LPWM2 10
#define R_EN2 11
#define L_EN2 12
#define RPWM3 44
#define LPWM3 45
#define R_EN3 46
#define L_EN3 47
const int DISPENSE_SPEED = 255;
const int FINE_DISPENSE_SPEED = 110;
const float FINE_APPROACH_KG = 0.20;

// ---- Overall cycle state machine ----
enum CycleState
{
  IDLE,
  DISPENSING,
  MOTOR_TEST
};

CycleState cycleState = IDLE;
String selectedRiceType = "";
byte selectedMotor = 0;
unsigned long stateStartMs = 0;
const unsigned long MOTOR_TEST_DURATION_MS = 5000;
const unsigned long DISPENSE_TIMEOUT_MS = 30000;
bool fineDispenseMode = false;

// ---- Serial command buffer ----
String serialBuffer = "";

void stopAllDispenseMotors();
void startSelectedDispenseMotor();
void handleBillPulses();
void handleCoinPulses();
void handleSerialCommands();
void processCommand(String cmd);
void runCycleStateMachine();
void startDispense(String riceType, float kg);
void updateWeightReading();
void setSelectedDispenseSpeed(int speed);

// ============================================================
// ISRs
// ============================================================
void billISR()
{
  unsigned long now = micros();
  if (now - lastBillEdge > DEBOUNCE_US)
  {
    billPulse++;
    lastBillPulseTime = millis();
  }
  lastBillEdge = now;
}

void coinISR()
{
  unsigned long now = micros();
  if (now - lastCoinEdge > DEBOUNCE_US)
  {
    coinPulse++;
    lastCoinPulseTime = millis();
  }
  lastCoinEdge = now;
}

// ============================================================
// Setup
// ============================================================
void setup()
{
  Serial.begin(9600);

  // Bill / Coin
  pinMode(billPin, INPUT_PULLUP);
  pinMode(coinPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(billPin), billISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(coinPin), coinISR, FALLING);

  // Initialize and zero the load cell using the reference calibration.
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);
  if (scale.wait_ready_timeout(1000))
  {
    scale.tare();
    Serial.println("=== 50KG LOAD CELL READY ===");
  }
  else
  {
    Serial.println("WARNING:HX711_NOT_READY");
  }

  // Configure all three rice dispenser drivers.
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(R_EN1, OUTPUT);
  pinMode(L_EN1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(R_EN2, OUTPUT);
  pinMode(L_EN2, OUTPUT);
  pinMode(RPWM3, OUTPUT);
  pinMode(LPWM3, OUTPUT);
  pinMode(R_EN3, OUTPUT);
  pinMode(L_EN3, OUTPUT);
  digitalWrite(R_EN1, LOW);
  digitalWrite(L_EN1, LOW);
  digitalWrite(R_EN2, LOW);
  digitalWrite(L_EN2, LOW);
  digitalWrite(R_EN3, LOW);
  digitalWrite(L_EN3, LOW);
  stopAllDispenseMotors();

  Serial.println("READY");
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
  handleBillPulses();
  handleCoinPulses();
  handleSerialCommands();
  updateWeightReading();
  runCycleStateMachine();
}

void updateWeightReading()
{
  unsigned long now = millis();
  if (now - lastWeightReadMs < WEIGHT_READ_INTERVAL_MS || !scale.is_ready())
    return;

  float sampleKg = scale.get_units(1);
  if (sampleKg < 0)
    sampleKg = 0;

  weightSamples[nextWeightSample] = sampleKg;
  nextWeightSample = (nextWeightSample + 1) % WEIGHT_AVERAGE_SAMPLES;
  if (weightSampleCount < WEIGHT_AVERAGE_SAMPLES)
    weightSampleCount++;

  float totalWeightKg = 0;
  for (byte index = 0; index < weightSampleCount; index++)
    totalWeightKg += weightSamples[index];
  currentWeightKg = totalWeightKg / weightSampleCount;
  weightReadingAvailable = weightSampleCount == WEIGHT_AVERAGE_SAMPLES;
  lastWeightReadMs = now;

  Serial.print("Weight: ");
  Serial.print(currentWeightKg, 2);
  Serial.println(" kg");

  if (cycleState == DISPENSING)
  {
    Serial.print("WEIGHT:");
    Serial.println(currentWeightKg * 1000.0, 2);
  }
}

// ============================================================
// Bill / Coin handling
// ============================================================
void processBillPulse()
{
  noInterrupts();
  int count = billPulse;
  billPulse = 0;
  interrupts();

  Serial.print("Bill Pulse: ");
  Serial.println(count);

  if (count == 10)
  {
    totalMoney += 100;
    Serial.println("Bill: PHP 100");
  }
  else if (count == 20)
  {
    totalMoney += 200;
    Serial.println("Bill: PHP 200");
  }
  else if (count == 25)
  {
    totalMoney += 250;
    Serial.println("Bill: PHP 250");
  }
  else if (count == 30)
  {
    totalMoney += 300;
    Serial.println("Bill: PHP 300");
  }
  else if (count == 50)
  {
    totalMoney += 500;
    Serial.println("Bill: PHP 500");
  }
  else
  {
    Serial.print("Unknown Bill (pulse count: ");
    Serial.print(count);
    Serial.println(") - test this denomination to add its mapping");
  }

  Serial.print("Total Money: PHP ");
  Serial.println(totalMoney);
}

void processCoinPulse()
{
  noInterrupts();
  int count = coinPulse;
  coinPulse = 0;
  interrupts();

  Serial.print("Coin Pulse: ");
  Serial.println(count);

  if (count == 9)
  {
    totalMoney += 10;
    Serial.println("Coin: PHP 10");
  }
  else if (count == 18)
  {
    totalMoney += 20;
    Serial.println("Coin: PHP 20");
  }
  else if (count == 27)
  {
    totalMoney += 30;
    Serial.println("Coin: PHP 30");
  }
  else
  {
    Serial.print("Unknown Coin (pulse count: ");
    Serial.print(count);
    Serial.println(")");
  }

  Serial.print("Total Money: PHP ");
  Serial.println(totalMoney);
}

void handleBillPulses()
{
  if (billPulse > 0 && (millis() - lastBillPulseTime > PULSE_TIMEOUT_MS))
  {
    processBillPulse();
  }
}

void handleCoinPulses()
{
  if (coinPulse > 0 && (millis() - lastCoinPulseTime > PULSE_TIMEOUT_MS))
  {
    processCoinPulse();
  }
}

// ============================================================
// Serial command handling
// ============================================================
void handleSerialCommands()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (serialBuffer.length() > 0)
      {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    }
    else
    {
      serialBuffer += c;
    }
  }
}

void processCommand(String cmd)
{
  cmd.trim();

  if (cmd.startsWith("MOTOR_TEST:"))
  {
    if (cycleState != IDLE)
    {
      Serial.println("ERROR:Cycle already in progress");
      return;
    }

    byte motor = (byte)cmd.substring(11).toInt();
    if (motor < 1 || motor > 3)
    {
      Serial.println("ERROR:Use MOTOR_TEST:1, MOTOR_TEST:2, or MOTOR_TEST:3");
      return;
    }

    selectedMotor = motor;
    cycleState = MOTOR_TEST;
    stateStartMs = millis();
    startSelectedDispenseMotor();
    Serial.print("MOTOR_TEST_START:");
    Serial.println(selectedMotor);
  }
  else if (cmd == "STOP_MOTORS")
  {
    stopAllDispenseMotors();
    cycleState = IDLE;
    selectedMotor = 0;
    Serial.println("MOTOR_TEST_STOPPED");
  }
  else if (cmd == "GET_TOTAL")
  {
    Serial.print("TOTAL:");
    Serial.println(totalMoney);
  }
  else if (cmd == "RESET_TOTAL")
  {
    totalMoney = 0;
    Serial.println("TOTAL:0");
  }
  else if (cmd.startsWith("DISPENSE:"))
  {
    if (cycleState != IDLE)
    {
      Serial.println("ERROR:Cycle already in progress");
      return;
    }

    // Expected format: DISPENSE:<riceType>:<kg>
    String rest = cmd.substring(9);
    int sepIndex = rest.lastIndexOf(':');
    if (sepIndex == -1)
    {
      Serial.println("ERROR:Expected DISPENSE:<riceType>:<kg>");
      return;
    }

    String riceType = rest.substring(0, sepIndex);
    float kg = rest.substring(sepIndex + 1).toFloat();

    if (riceType.length() == 0)
    {
      Serial.println("ERROR:Missing rice type");
      return;
    }
    if (kg <= 0)
    {
      Serial.println("ERROR:Invalid dispense weight");
      return;
    }

    startDispense(riceType, kg);
  }
  else
  {
    Serial.print("ERROR:Unknown command ");
    Serial.println(cmd);
  }
}

// ============================================================
// Cycle state machine
// ============================================================
void startDispense(String riceType, float kg)
{
  String normalizedType = riceType;
  normalizedType.toLowerCase();

  if (normalizedType == "jasmine")
    selectedMotor = 1;
  else if (normalizedType == "sinandomeng")
    selectedMotor = 2;
  else if (normalizedType == "brown" || normalizedType == "brown rice" || normalizedType == "pinawa")
    selectedMotor = 3;
  else
  {
    Serial.print("ERROR:Unknown rice type ");
    Serial.println(riceType);
    return;
  }

  selectedRiceType = riceType;
  targetWeightKg = kg;
  if (!scale.wait_ready_timeout(1000))
  {
    Serial.println("ERROR:HX711_NOT_READY");
    return;
  }

  scale.tare();
  currentWeightKg = 0;
  weightReadingAvailable = false;
  weightSampleCount = 0;
  nextWeightSample = 0;
  fineDispenseMode = false;
  for (byte index = 0; index < WEIGHT_AVERAGE_SAMPLES; index++)
    weightSamples[index] = 0;
  lastWeightReadMs = 0;
  cycleState = DISPENSING;
  stateStartMs = millis();

  Serial.print("DISPENSE_START:");
  Serial.print(selectedRiceType);
  Serial.print(":");
  Serial.println(kg, 2);

  startSelectedDispenseMotor();
}

void runCycleStateMachine()
{
  unsigned long now = millis();

  switch (cycleState)
  {
  case DISPENSING:
    if (weightReadingAvailable && !fineDispenseMode &&
        currentWeightKg >= targetWeightKg - FINE_APPROACH_KG)
    {
      fineDispenseMode = true;
      setSelectedDispenseSpeed(FINE_DISPENSE_SPEED);
      Serial.println("DISPENSE_FINE_MODE");
    }

    if (weightReadingAvailable && currentWeightKg >= targetWeightKg)
    {
      stopAllDispenseMotors();
      Serial.println("MOTOR_STOPPED");
      Serial.print("DISPENSE_DONE:");
      Serial.println(currentWeightKg, 2);
      Serial.println("CYCLE_DONE");
      cycleState = IDLE;
      selectedRiceType = "";
      selectedMotor = 0;
      fineDispenseMode = false;
    }
    else if (now - stateStartMs >= DISPENSE_TIMEOUT_MS)
    {
      stopAllDispenseMotors();
      Serial.println("ERROR:DISPENSE_TIMEOUT");
      cycleState = IDLE;
      selectedRiceType = "";
      selectedMotor = 0;
      fineDispenseMode = false;
    }
    break;

  case MOTOR_TEST:
    if (now - stateStartMs >= MOTOR_TEST_DURATION_MS)
    {
      stopAllDispenseMotors();
      selectedMotor = 0;
      cycleState = IDLE;
      Serial.println("MOTOR_TEST_DONE");
    }
    break;

  case IDLE:
  default:
    break;
  }
}

// ============================================================
// Motor helpers
// ============================================================
void stopAllDispenseMotors()
{
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 0);
  digitalWrite(RPWM1, LOW);
  digitalWrite(LPWM1, LOW);
  digitalWrite(R_EN1, LOW);
  digitalWrite(L_EN1, LOW);

  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 0);
  digitalWrite(RPWM2, LOW);
  digitalWrite(LPWM2, LOW);
  digitalWrite(R_EN2, LOW);
  digitalWrite(L_EN2, LOW);

  analogWrite(RPWM3, 0);
  analogWrite(LPWM3, 0);
  digitalWrite(RPWM3, LOW);
  digitalWrite(LPWM3, LOW);
  digitalWrite(R_EN3, LOW);
  digitalWrite(L_EN3, LOW);
}

void startSelectedDispenseMotor()
{
  stopAllDispenseMotors();

  if (selectedMotor == 1)
  {
    digitalWrite(R_EN1, HIGH);
    digitalWrite(L_EN1, HIGH);
    digitalWrite(LPWM1, LOW);
    analogWrite(RPWM1, DISPENSE_SPEED);
  }
  else if (selectedMotor == 2)
  {
    digitalWrite(R_EN2, HIGH);
    digitalWrite(L_EN2, HIGH);
    digitalWrite(LPWM2, LOW);
    analogWrite(RPWM2, DISPENSE_SPEED);
  }
  else if (selectedMotor == 3)
  {
    digitalWrite(R_EN3, HIGH);
    digitalWrite(L_EN3, HIGH);
    digitalWrite(LPWM3, LOW);
    analogWrite(RPWM3, DISPENSE_SPEED);
  }

  Serial.print("MOTOR_START:");
  Serial.println(selectedMotor);
}

void setSelectedDispenseSpeed(int speed)
{
  if (selectedMotor == 1)
    analogWrite(RPWM1, speed);
  else if (selectedMotor == 2)
    analogWrite(RPWM2, speed);
  else if (selectedMotor == 3)
    analogWrite(RPWM3, speed);
}