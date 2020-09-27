#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>

#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold24pt7b.h"
#include "usergraphics.h"
#include "Bounce2.h"

#define POWER_BUTTON T7//27
#define MODE_BUTTON T4//13
#define UP_BUTTON   T5//12
#define DOWN_BUTTON T6//14

#define TFT_CS   5
#define TFT_DC   25//4
#define TFT_MOSI 23
#define TFT_CLK  18
#define TFT_RST  17//22
#define TFT_MISO 19
#define TFT_LED  2//15  

#define DEBOUNCE_INTERVAL 25 //in ms

#define EEPROM_SIZE 2
#define EEPROM_TEMP 0
#define EEPROM_MODBUS_ID 1

#define WDT_TIMEOUT 5

#define ILI9341_ULTRA_DARKGREY 0x632C      

#define DEFAULT_ID 1

#define MIN_TEMPERATURE 18
#define MAX_TEMPERATURE 28

enum { MODE_MAIN, MODE_SETTINGS };
enum { BOOT, COOLING, TEMP_OK, HEATING };

const uint8_t SERIAL_STATUS = 1;
const uint8_t SERIAL_UPDATE = 2;
const uint8_t SERIAL_ROOM_TEMP = 3;

#define SERIAL_BAUDRATE 9600
const uint8_t SERIAL_BUFFER_LENGTH = 15;
char inSerialChar;
uint8_t serialBufferPos;
char *serialToken;
char serialBuffer[SERIAL_BUFFER_LENGTH];
char serialDelim[2];
char *serialLast;

const char *SERIAL_SEND_POWER = "1~%d";
const char *SERIAL_SEND_TEMP = "2~%d";
const char *SERIAL_SEND_MODBUS = "3~%d";

uint8_t thermostatMode = BOOT;
 
uint8_t roomTemperature = 21;
uint8_t setTemperature = 20;

uint8_t mode = MODE_MAIN;
uint8_t modbusId = DEFAULT_ID;  // ID / address for modbus
 
uint8_t power = 0;

#define DISPLAY_TIMER_TIMEOUT 255
uint8_t displayTimer = 0;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Bounce powerButton = Bounce();
Bounce modeButton = Bounce();
Bounce upButton = Bounce();
Bounce downButton = Bounce();
 
void setup() {
  setupEEPROM();
   
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  tft.begin();
  tft.setRotation(1);

  setupButtons();
  setupWDT();
  setupSerialCommand();
  drawMainScreen();
  resetDisplayTimer();
}

void loop() {
  detectButtons();
  readSerial();
  checkDisplayTimer();
  delay(100);
  esp_task_wdt_reset();
}

void resetDisplayTimer() {
  displayTimer = DISPLAY_TIMER_TIMEOUT;
}

void checkDisplayTimer() {
  if (displayTimer) {
    displayTimer--;
    digitalWrite(TFT_LED, HIGH);
  } else {
    digitalWrite(TFT_LED, LOW);
  }
}

void setupEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_TEMP) != 255) {
    setTemperature = EEPROM.read(EEPROM_TEMP);
  }
  if (EEPROM.read(EEPROM_MODBUS_ID) != 255) {
    modbusId = EEPROM.read(EEPROM_MODBUS_ID);
  }
}

void setupButtons() {
  powerButton.attach(POWER_BUTTON);
  powerButton.interval(DEBOUNCE_INTERVAL);
  
  modeButton.attach(MODE_BUTTON);
  modeButton.interval(DEBOUNCE_INTERVAL);

  upButton.attach(UP_BUTTON);
  upButton.interval(DEBOUNCE_INTERVAL);

  downButton.attach(DOWN_BUTTON);
  downButton.interval(DEBOUNCE_INTERVAL);
}

void setupWDT() {
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
}

void detectButtons() {
  powerButton.update();
  modeButton.update();
  upButton.update();
  downButton.update();

  if (powerButton.changed() && powerButton.read()) {
    resetDisplayTimer();
    
    if (power == 1) {
      power = 0;
      drawPowerOffScreen();
    } else {
      power = 1;
      thermostatMode = BOOT;
      mode = MODE_MAIN;
      drawMainScreen();
    }
    sendSerial(SERIAL_SEND_POWER, power);
  }
  else if (modeButton.changed() && modeButton.read()) {
    resetDisplayTimer();
    
    if (mode == MODE_MAIN) {
      mode = MODE_SETTINGS;
      drawOptionScreen();
    } else {
      thermostatMode = BOOT;
      mode = MODE_MAIN; 
      drawMainScreen();
      sendSerial(SERIAL_SEND_MODBUS, modbusId);
    }
  }
  if (mode == MODE_MAIN) {
    if (upButton.changed() && upButton.read()) {
      resetDisplayTimer();
      
      if (setTemperature < MAX_TEMPERATURE) {
        setTemperature++;
      }
      saveEEPROM(EEPROM_TEMP, setTemperature);
      updateSetTemp();
      updateCircleColor();
      sendSerial(SERIAL_SEND_TEMP, setTemperature);
    } else if (downButton.changed() && downButton.read()) {
      resetDisplayTimer();
      
      if (setTemperature > MIN_TEMPERATURE) {
        setTemperature--;
      }
      saveEEPROM(EEPROM_TEMP, setTemperature);
      updateSetTemp();
      updateCircleColor();
      sendSerial(SERIAL_SEND_TEMP, setTemperature);
    }
  } else {
    if (upButton.changed() && upButton.read()) {
      resetDisplayTimer();
      
      if (modbusId < 255) {
        modbusId++;
      }
      saveEEPROM(EEPROM_MODBUS_ID, modbusId);
      updateModbusAddr();
    } else if (downButton.changed() && downButton.read()) {
      resetDisplayTimer();
      
      if (modbusId > 1) {
        modbusId--;
      }
      saveEEPROM(EEPROM_MODBUS_ID, modbusId);
      updateModbusAddr();
    }
  }
}

void saveEEPROM(int address, int value) {
  EEPROM.write(address, value);
  EEPROM.commit();
}

void drawMainScreen() {
  tft.fillScreen(ILI9341_BLACK);
  
  updateCircleColor();
  updateSetTemp();
}

void drawPowerOffScreen() {
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold9pt7b);  
  
  tft.setCursor(50, 120);
  tft.print("Power off");
}

void drawOptionScreen() {
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold9pt7b);  
  
  tft.setCursor(50, 120);
  tft.print("MODBUS address");
  
  updateModbusAddr();
}

void updateSetTemp() {
  int16_t x1, y1;
  uint16_t w, h;
  String curValue = String(setTemperature);
  int strLen =  curValue.length() + 1; 
  char charArray[strLen];
  curValue.toCharArray(charArray, strLen);
  tft.fillRect(120, 81, 60, 50, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  tft.getTextBounds(charArray, 80, 130, &x1, &y1, &w, &h);
  tft.setCursor(168 - w, 115);
  tft.print(charArray);
}

void updateRoomTemp() {
  int16_t x1, y1;
  uint16_t w, h;
  String curValue = String(roomTemperature);
  int strLen =  curValue.length() + 1; 
  char charArray[strLen];
  curValue.toCharArray(charArray, strLen);
  tft.fillRect(86, 185, 30, 21, ILI9341_ULTRA_DARKGREY);
  tft.setTextColor(ILI9341_WHITE, ILI9341_ULTRA_DARKGREY);
  tft.setFont(&FreeSansBold12pt7b);
  tft.getTextBounds(charArray, 90, 205, &x1, &y1, &w, &h);
  tft.setCursor(106 - w, 205);
  tft.print(charArray);
}

void updateCircleColor() {
  // HEATING 
  if ((roomTemperature < setTemperature) && (thermostatMode != HEATING)) {
    thermostatMode = HEATING;
    drawCircles();
  }
  // COOLING 
  if ((roomTemperature > setTemperature) && (thermostatMode != COOLING)) {
    thermostatMode = COOLING;
    drawCircles();
  }
  // Temperature ok 
  if ((roomTemperature == setTemperature) && (thermostatMode != TEMP_OK)) {
    thermostatMode = TEMP_OK;
    drawCircles();
  }
}

void updateModbusAddr() {
  tft.fillRect(230, 100, 60, 45, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(235, 130);
  tft.print(modbusId);
}

void drawCircles() {
  uint16_t color;
  if (roomTemperature < setTemperature) {
    color = ILI9341_RED; //heating
  } else if (roomTemperature > setTemperature) {
    color = ILI9341_BLUE; //cooling    
  } else {
    color = ILI9341_GREEN; // Temperature ok
  }
  for (uint8_t i = 0; i < 10; i++) {
    tft.drawCircle(170, 105, 80 + i, color);
  }
  tft.fillCircle(110, 185, 40, ILI9341_ULTRA_DARKGREY);

  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setCursor(180, 85);
  tft.print("o");
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(190, 115);
  tft.print("C");

  tft.setTextColor(ILI9341_WHITE, ILI9341_ULTRA_DARKGREY);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setCursor(125, 205);
  tft.print("C");
  tft.drawCircle(119, 189, 2, ILI9341_WHITE);
  tft.drawCircle(119, 189, 3, ILI9341_WHITE);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setCursor(85, 175);
  tft.print("Room");
  updateRoomTemp();
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
1
2~0-1~18-28
3~0-50
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
        case SERIAL_STATUS:
          processStatus();
          break;
        case SERIAL_UPDATE:
          processUpdate();
          break;
        case SERIAL_ROOM_TEMP:
          processRoomTemp();
          break;   
      }
      clearSerialBuffer();
    }
    serialBuffer[serialBufferPos++] = inSerialChar;
    serialBuffer[serialBufferPos] = '\0';
  }  
}

void processStatus() {
  sendSerial(SERIAL_SEND_MODBUS, modbusId);
  sendSerial(SERIAL_SEND_TEMP, setTemperature);
}

void processUpdate() {
  int number;
  char *param;

  param = serialNextParam();

  if (param != NULL) {
    power = atol(param);
    if (power == 1) {
      drawPowerOffScreen();
    } else {
      thermostatMode = BOOT;
      mode = MODE_MAIN;
      drawMainScreen();
    }

    param = serialNextParam();
    if (param != NULL) {
        setTemperature = atol(param);
        updateSetTemp();
        updateCircleColor();
    }
  }
}

void processRoomTemp() {
  char *param;
  param = serialNextParam();
  if (param != NULL) {
    roomTemperature = atol(param) / 10;
    updateRoomTemp();
    updateCircleColor();
  }
}

char *serialNextParam() {
  return strtok_r(NULL, serialDelim, &serialLast);
}

void sendSerial(const char *command, int value) {
  char buf[15];
  sprintf(buf, command, value);
  Serial1.println(buf);
}
