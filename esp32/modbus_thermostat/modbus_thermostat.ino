#include "ModbusSlave.h"
#include <OneWire.h>
#include <IWatchdog.h>

#define SLAVE_ID 1
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA3
#define RS485_TX_PIN PA2
#define RS485_TX_ENABLE_PIN PA4

#define UART_RX_PIN PA10
#define UART_TX_PIN PA9

#define THERMOSTAT_OUTPUT_PIN PB1

#define DS18B20_EXT_PIN PA0
#define DS18B20_INT_PIN PA1
#define DS18B20_PRECISION 11

#define SERIAL_BAUDRATE 9600

const uint8_t THERMOSTAT_ON = 0;
const uint8_t THERMOSTAT_TEMP = 1;
const uint8_t MAX_FLOOR_TEMP = 2;
const uint8_t FLOOR_TEMP = 3;
const uint8_t AIR_TEMP = 4;

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t HOLDING_COUNT = 5;

uint8_t outputState = LOW;
uint16_t holdingRegister[HOLDING_COUNT] = { 0, 25, 40, 0, 0 }; //{ THERMOSTAT_ON, THERMOSTAT_TEMP, MAX_FLOOR_TEMP, FLOOR_TEMP, AIR_TEMP }

float floorTemp, airTemp, lastAirTemp;

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

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneWire dsInt(DS18B20_INT_PIN);
OneWire dsExt(DS18B20_EXT_PIN);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(0, outputState);
  return STATUS_OK;
}

void setOutput(uint8_t pin, uint8_t value) {
  digitalWrite(pin, value);
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastThermostatState = outputState;

  outputState = slave.readCoilFromBuffer(0);

  if (outputState != lastThermostatState) {
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState);
  } 
  return STATUS_OK;
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, HERTZ_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

void initDS(OneWire ds) {
  ds.reset();
  ds.write(0xCC);
  ds.write(0x4E);
  ds.write(0);
  ds.write(0);
  ds.write(DS18B20_PRECISION << 5);
  ds.write(0x48);
}

float readDS(OneWire ds) {
  uint8_t data[2];

  ds.reset();
  ds.write(0xCC);
  ds.write(0xBE);
  data[0] = ds.read();
  data[1] = ds.read();

  ds.reset();
  ds.write(0xCC);
  ds.write(0x44);

  int16_t raw = (data[1] << 8) | data[0];
  return (float)raw / 16.0;
}

void updateSensors() {
  airTemp = readDS(dsInt);
  holdingRegister[AIR_TEMP] = airTemp * 10;
   if (airTemp != lastAirTemp) {
    lastAirTemp = airTemp;
    sendSerial(SERIAL_SEND_AIR_TEMP, holdingRegister[AIR_TEMP]);
  }
  
  floorTemp = readDS(dsExt);
  
  if (floorTemp == 85) {//initial value
    return;
  }
  holdingRegister[FLOOR_TEMP] = floorTemp * 10;
 
  if (holdingRegister[THERMOSTAT_ON]) {
    if ((floorTemp <= holdingRegister[MAX_FLOOR_TEMP]) && (airTemp < holdingRegister[THERMOSTAT_TEMP])) {
      outputState = 1;
    } else {
      outputState = 0;
    }
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState);
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
  if (!holdingRegister[THERMOSTAT_ON]) {
    outputState = LOW;
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState);
  }
  sendSerial(SERIAL_SEND_UPDATE, holdingRegister[THERMOSTAT_ON], holdingRegister[THERMOSTAT_TEMP]);
  return STATUS_OK;
}

void setup() {
  initDS(dsInt);
  initDS(dsExt);
  initPeriodicalTimer();

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readHolding;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHolding;

  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);

  setupSerialCommand();

  delay(2000);
  getSettings();

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  readSerial();
  slave.poll();
  IWatchdog.reload();
}

void setupSerialCommand() {
  Serial1.begin(SERIAL_BAUDRATE);
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
  while (Serial1.available() > 0) {
    uint8_t serialCommand;
    inSerialChar = Serial1.read();
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
    holdingRegister[THERMOSTAT_ON] = atol(param);
    if (!holdingRegister[THERMOSTAT_ON]) {
      outputState = LOW;
      setOutput(THERMOSTAT_OUTPUT_PIN, outputState);
    }
  }
}

void processTemp() {
  char *param;
  param = serialNextParam();
  if (param != NULL) {
    holdingRegister[THERMOSTAT_TEMP] = atol(param);
  }
}

void processModbus() {
  uint8_t value;
  char *param;
  param = serialNextParam();
  if (param != NULL) {
    value = atol(param);
    slave.setUnitAddress(value);
  }
}

char *serialNextParam() {
  return strtok_r(NULL, serialDelim, &serialLast);
}

void sendSerial(const char *command) {
  char buf[15];
  sprintf(buf, command);
  Serial1.println(buf);
}

void sendSerial(const char *command, int value) {
  char buf[15];
  sprintf(buf, command, value);
  Serial1.println(buf);
}

void sendSerial(const char *command, int value, int value2) {
  char buf[15];
  sprintf(buf, command, value, value2);
  Serial1.println(buf);
}

void getSettings() {
  sendSerial(SERIAL_SEND_STATUS);
}
