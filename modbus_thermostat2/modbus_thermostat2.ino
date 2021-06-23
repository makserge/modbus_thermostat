#include <ModbusSlave.h>
#include <OneButton.h>
#include <IWatchdog.h>

#define SLAVE_ID 50
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA3
#define RS485_TX_PIN PA2
#define RS485_TX_ENABLE_PIN PA4

#define THERMOSTAT_ON_PIN PA6
#define THERMOSTAT_LED_PIN PB1
#define THERMOSTAT_OUTPUT_PIN PA0
#define EXTRA_OUTPUT_PIN PA1

#define NTC_EXT_PIN PA5
#define NTC_INT_PIN PA7

const uint16_t NTC_REFERENCE_RESISTANCE = 10000;
const uint16_t NTC_NOMINAL_RESISTANCE = 10000;
const uint8_t NTC_NOMINAL_TEMPERATURE = 25;
const uint16_t NTC_B_VALUE = 3950;
const uint16_t NTC_ADC_RESOLUTION = 1023;
const uint8_t NTC_NUM_SAMPLES = 5;

const uint8_t THERMOSTAT_ON = 0;
const uint8_t THERMOSTAT_TEMP = 1;
const uint8_t MIN_FLOOR_TEMP = 2;
const uint8_t MAX_FLOOR_TEMP = 3;
const uint8_t FLOOR_TEMP = 4;
const uint8_t AIR_TEMP = 5;

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t HOLDING_COUNT = 6;

const uint8_t THERMOSTAT_STATE = 0;
const uint8_t EXTRA_OUT_STATE = 1;

uint8_t outputState[2] = { LOW, LOW };
uint16_t holdingRegister[HOLDING_COUNT] = { LOW, 25, 20, 40, 0, 0 }; //{ THERMOSTAT_ON, THERMOSTAT_TEMP, MIN_FLOOR_TEMP, MAX_FLOOR_TEMP, FLOOR_TEMP, AIR_TEMP }

float floorTemp, airTemp;
uint16_t lastAirTemp;

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton thermostatButton(THERMOSTAT_ON_PIN, true, false);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(THERMOSTAT_STATE, outputState[THERMOSTAT_STATE]);
  slave.writeCoilToBuffer(EXTRA_OUT_STATE, outputState[EXTRA_OUT_STATE]);
  return STATUS_OK;
}

void setOutput(uint8_t pin, uint8_t value) {
  digitalWrite(pin, value);
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastThermostatState = outputState[THERMOSTAT_STATE];
  uint8_t lastExtraOutState = outputState[EXTRA_OUT_STATE];

  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[THERMOSTAT_STATE] != lastThermostatState) {
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  } 
  if (outputState[EXTRA_OUT_STATE] != lastExtraOutState) {
    setOutput(EXTRA_OUTPUT_PIN, outputState[EXTRA_OUT_STATE]);
  } 
  return STATUS_OK;
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, HERTZ_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

float readNTC(int pin) {
  uint8_t i;
  uint16_t sample;
  float steinhart, average = 0;

  for (i = 0; i < NTC_NUM_SAMPLES; i++) {
    sample = analogRead(pin);
    average += sample;
    delay(10);
  }
  average /= NTC_NUM_SAMPLES;
  average = NTC_ADC_RESOLUTION / average - 1;
  average = NTC_REFERENCE_RESISTANCE / average;

  steinhart = average / NTC_NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= NTC_B_VALUE;
  steinhart += 1.0 / (NTC_NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  return steinhart - 273.15;
}

void updateSensors() {
  airTemp = readNTC(NTC_INT_PIN);
  holdingRegister[AIR_TEMP] = airTemp * 10;
  if (holdingRegister[AIR_TEMP] != lastAirTemp) {
    lastAirTemp = holdingRegister[AIR_TEMP];
  }
  
  floorTemp = readNTC(NTC_EXT_PIN);
  
  holdingRegister[FLOOR_TEMP] = floorTemp * 10;
 
  if (holdingRegister[THERMOSTAT_ON]) {
    if ((floorTemp <= holdingRegister[MAX_FLOOR_TEMP]) && (airTemp < holdingRegister[THERMOSTAT_TEMP])) {
      outputState[THERMOSTAT_STATE] = 1;
    } else {
      outputState[THERMOSTAT_STATE] = 0;
    }
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  }
}
 
/**
 * Handle Read Holding Registers (FC=03)
 */
uint8_t readHolding(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < HOLDING_COUNT; i++) {
    slave.writeRegisterToBuffer(i, holdingRegister[i]);
  }
  return STATUS_OK;
}

/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 */
uint8_t writeHolding(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    holdingRegister[i + address] = slave.readRegisterFromBuffer(i);
  }
  if (holdingRegister[THERMOSTAT_TEMP] > holdingRegister[MAX_FLOOR_TEMP]) {
    holdingRegister[THERMOSTAT_TEMP] = holdingRegister[MAX_FLOOR_TEMP];
  } else if (holdingRegister[THERMOSTAT_TEMP] < holdingRegister[MIN_FLOOR_TEMP]) {
    holdingRegister[THERMOSTAT_TEMP] = holdingRegister[MIN_FLOOR_TEMP];
  }
  setOutput(THERMOSTAT_LED_PIN, holdingRegister[THERMOSTAT_ON]);
  if (!holdingRegister[THERMOSTAT_ON]) {
    outputState[THERMOSTAT_STATE] = LOW;
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  }
  return STATUS_OK;
}

void clickThermostatButton() {
  holdingRegister[THERMOSTAT_ON] = !holdingRegister[THERMOSTAT_ON];
  setOutput(THERMOSTAT_LED_PIN, holdingRegister[THERMOSTAT_ON]);
  if (!holdingRegister[THERMOSTAT_ON]) {
    outputState[THERMOSTAT_STATE] = LOW;
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  }  
}

void initButtons() {
  pinMode(THERMOSTAT_ON_PIN, INPUT);
  pinMode(EXTRA_OUTPUT_PIN, OUTPUT);
  pinMode(THERMOSTAT_LED_PIN, OUTPUT);
  pinMode(THERMOSTAT_OUTPUT_PIN, OUTPUT);

  thermostatButton.attachClick(clickThermostatButton);
}

void setupNTC() {
  pinMode(NTC_INT_PIN, INPUT);
  pinMode(NTC_EXT_PIN, INPUT);
}

void setup() {
  initButtons();
  setupNTC();
  
  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readHolding;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHolding;
  
  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);
  
  initPeriodicalTimer();
  
  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  thermostatButton.tick();
  slave.poll();
  IWatchdog.reload();
}
