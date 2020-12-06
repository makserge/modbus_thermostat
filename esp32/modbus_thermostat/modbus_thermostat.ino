#include <IWatchdog.h>
#include "ModbusSlave.h"

#define SLAVE_ID 1
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA3
#define RS485_TX_PIN PA2
#define RS485_TX_ENABLE_PIN PA4

#define UART_RX_PIN PA10
#define UART_TX_PIN PA9
#define UART_BAUDRATE 9600

#define THERMOSTAT_OUTPUT_PIN PB1
#define EXTRA_OUTPUT_PIN PA5

#define NTC_INT_PIN PA1
#define NTC_EXT_PIN PA0

const uint16_t NTC_REFERENCE_RESISTANCE = 10000;
const uint16_t NTC_NOMINAL_RESISTANCE = 10000;
const uint8_t NTC_NOMINAL_TEMPERATURE = 25;
const uint16_t NTC_B_VALUE = 3950;
const uint16_t NTC_ADC_RESOLUTION = 1023;
const uint8_t NTC_NUM_SAMPLES = 5;

const uint8_t MAX_MODBUS_ADDRESS = 255;

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

uint8_t outputState[2] = { LOW, HIGH };
uint16_t holdingRegister[HOLDING_COUNT] = { HIGH, 25, 20, 40, 0, 0 }; //{ THERMOSTAT_ON, THERMOSTAT_TEMP, MIN_FLOOR_TEMP, MAX_FLOOR_TEMP, FLOOR_TEMP, AIR_TEMP }

float floorTemp, airTemp;
uint16_t lastAirTemp;

const uint8_t SERIAL_POWER = 1;
const uint8_t SERIAL_TEMP = 2;
const uint8_t SERIAL_MODBUS = 3;

const uint8_t SERIAL_BUFFER_LENGTH = 15;
char inSerialChar;
uint8_t serialBufferPos;
char *serialToken;
char serialBuffer[SERIAL_BUFFER_LENGTH];
char serialDelim[2];
char *serialLast;

const char *SERIAL_SEND_STATUS = "1";
const char *SERIAL_SEND_UPDATE = "2~%d~%d";
const char *SERIAL_SEND_AIR_TEMP = "3~%d";

const uint8_t POWER_OFF = 0;
const uint8_t POWER_ON = 1;

HardwareSerial Serial2(UART_RX_PIN, UART_TX_PIN);
Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);

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
    sendSerial(SERIAL_SEND_AIR_TEMP, holdingRegister[AIR_TEMP]);
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
  if (!holdingRegister[THERMOSTAT_ON]) {
    outputState[THERMOSTAT_STATE] = LOW;
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  }
  sendSerial(SERIAL_SEND_UPDATE, holdingRegister[THERMOSTAT_ON], holdingRegister[THERMOSTAT_TEMP]);
  return STATUS_OK;
}

void setup() {
  setupNTC();
  setupSerialCommand();
  delay(2000);
  getSettings();
  
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
  readSerial();
  slave.poll();
  IWatchdog.reload();
}

void setupNTC() {
  pinMode(NTC_INT_PIN, INPUT);
  pinMode(NTC_EXT_PIN, INPUT);
}

void setupSerialCommand() {
  Serial2.begin(UART_BAUDRATE);
  strncpy(serialDelim, "~", 2);
  clearSerialBuffer();
}

void clearSerialBuffer() {
  for (uint8_t i = 0; i < SERIAL_BUFFER_LENGTH; i++) {
    serialBuffer[i] = '\0';
  }
  serialBufferPos = 0;
  delay(5);
}

void readSerial() {
/*  
1~0-1
2~16-28
3~1-254
*/ 
  while (Serial2.available() > 0) {
    uint8_t serialCommand;
    inSerialChar = Serial2.read();
    if (inSerialChar == '\n') {
      serialBufferPos = 0;
      serialToken = strtok_r(serialBuffer, serialDelim, &serialLast);
      if (serialToken == NULL) {
        return;
      }
      serialCommand = atoi(serialToken);
      switch (serialCommand) {
        case SERIAL_POWER:
          processPower();
          break;
        case SERIAL_TEMP:
          processTemp();
          break;
        case SERIAL_MODBUS:
          processModbus();
          break;   
      }
      clearSerialBuffer();
    }
    serialBuffer[serialBufferPos++] = inSerialChar;
    serialBuffer[serialBufferPos] = '\0';
  }
}

void processPower() {
  char *param;
  param = serialNextParam();
  if (param != NULL) {
    uint8_t power = atol(param);
    if ((power == POWER_ON) || (power == POWER_OFF)) {
      holdingRegister[THERMOSTAT_ON] = power;
      if (!power) {
        outputState[THERMOSTAT_STATE] = LOW;
        setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
      }
    }  
  }
}

void processTemp() {
  char *param;
  param = serialNextParam();
  if ((param != NULL)) {
    uint8_t temp = atol(param);
    if (temp > holdingRegister[MAX_FLOOR_TEMP]) {
      temp = holdingRegister[MAX_FLOOR_TEMP];
    } else if (temp < holdingRegister[MIN_FLOOR_TEMP]) {
      temp = holdingRegister[MIN_FLOOR_TEMP];
    }
    holdingRegister[THERMOSTAT_TEMP] = temp;
  }
}

void processModbus() {
  uint8_t value;
  char *param;
  param = serialNextParam();
  if (param != NULL) {
    value = atol(param);
    if (value > MAX_MODBUS_ADDRESS) {
      value =  MAX_MODBUS_ADDRESS;
    }  
    slave.setUnitAddress(value);
  }
}

char *serialNextParam() {
  return strtok_r(NULL, serialDelim, &serialLast);
}

void sendSerial(const char *command) {
  char buf[15];
  sprintf(buf, command);
  Serial2.println(buf);
}

void sendSerial(const char *command, int value) {
  char buf[15];
  sprintf(buf, command, value);
  Serial2.println(buf);
}

void sendSerial(const char *command, int value, int value2) {
  char buf[15];
  sprintf(buf, command, value, value2);
  Serial2.println(buf);
}

void getSettings() {
  sendSerial(SERIAL_SEND_STATUS);
}
