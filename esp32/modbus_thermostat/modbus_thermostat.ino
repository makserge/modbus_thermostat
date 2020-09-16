#include <ModbusSlave.h>
#include <OneWire.h>
#include <IWatchdog.h>

#define SLAVE_ID 41
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

float floorTemp, airTemp;

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

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  slave.poll();
  IWatchdog.reload();
}
