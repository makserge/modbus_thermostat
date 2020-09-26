#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold24pt7b.h"
#include "SimpleModbusSlave.h"
#include "usergraphics.h"
#include "Bounce2.h"

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

#define ILI9341_ULTRA_DARKGREY 0x632C      

#define BAUDRATE 9600
#define DEFAULT_ID 1
enum 
{     
  // just add or remove registers 
  // The first register starts at address 0
  ROOM_TEMP,  // measured room temp from external sensor
  SET_TEMP,   // set-point temperature by user, 
  DISP_ONOFF, // timer for display automatic off function (0 switch backlight off, >0 set timer for automatic off)
  TOTAL_ERRORS,
  // leave this one
  TOTAL_REGS_SIZE 
  // total number of registers for function 3 and 16 share the same register array
}; 

#define MAX_TEMPERATURE 28  
#define MIN_TEMPERATURE 18
enum { MODE_MAIN, MODE_SETTINGS };
enum { BOOT, COOLING, TEMP_OK, HEATING };  // Thermostat modes

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Bounce modeButton = Bounce();
Bounce upButton = Bounce();
Bounce downButton = Bounce();

uint8_t thermostatMode = BOOT;
 
uint8_t roomTemperature = 21;
uint8_t setTemperature = 20;

uint8_t mode = MODE_MAIN;
uint8_t modbusId = DEFAULT_ID;  // ID / address for modbus
 
unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array 
 
void setup() {
  holdingRegs[ROOM_TEMP] = roomTemperature;
  holdingRegs[SET_TEMP] = setTemperature;
  holdingRegs[DISP_ONOFF] = 255;
  
  /* parameters(long baudrate, 
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned char low latency)
                
     The transmit enable pin is used in half duplex communication to activate a MAX485 or similar
     to deactivate this mode use any value < 2 because 0 & 1 is reserved for Rx & Tx.
     Low latency delays makes the implementation non-standard
     but practically it works with all major modbus master implementations.
  */
  modbus_configure(BAUDRATE, modbusId, 0, TOTAL_REGS_SIZE, 0); 
  
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  tft.begin();
  tft.setRotation(1);

  setupButtons();

  drawMainScreen();
}

void loop() {
  detectButtons();
 
  //automatic display BL timeout
  if (holdingRegs[DISP_ONOFF]) {
    holdingRegs[DISP_ONOFF]--;
    digitalWrite(TFT_LED, HIGH);
  } else {
    digitalWrite(TFT_LED, LOW);
  }

  modbusProcessing(); 
  delay(100); 
}

void modbusProcessing() {
  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs); 

  // update of variables by Modbus
  if (holdingRegs[ROOM_TEMP] != roomTemperature) {
     if ((holdingRegs[ROOM_TEMP] > 50) || (holdingRegs[ROOM_TEMP] < 5)) {
       holdingRegs[ROOM_TEMP] = roomTemperature;
     } else {
       roomTemperature = holdingRegs[ROOM_TEMP];
       updateRoomTemp();
       updateCircleColor();
     }
  }
  if (holdingRegs[SET_TEMP] != setTemperature) {
     if ((holdingRegs[SET_TEMP] > MAX_TEMPERATURE) || (holdingRegs[SET_TEMP] < MIN_TEMPERATURE)) {
       holdingRegs[SET_TEMP] = setTemperature;
     } else {
       setTemperature = holdingRegs[SET_TEMP];
       updateSetTemp();
       updateCircleColor();
     }
  }
}

void setupButtons() {
  modeButton.attach(MODE_BUTTON);
  modeButton.interval(DEBOUNCE_INTERVAL);

  upButton.attach(UP_BUTTON);
  upButton.interval(DEBOUNCE_INTERVAL);

  downButton.attach(DOWN_BUTTON);
  downButton.interval(DEBOUNCE_INTERVAL);
}

void detectButtons() {
  modeButton.update();
  upButton.update();
  downButton.update();

  if (modeButton.changed() && modeButton.read()) {
    holdingRegs[DISP_ONOFF] = 255;
    
    if (mode == MODE_MAIN) {
      mode = MODE_SETTINGS;
      drawOptionScreen();
    } else {
      modbus_configure(BAUDRATE, modbusId, 0, TOTAL_REGS_SIZE, 0);
      thermostatMode = BOOT;
      mode = MODE_MAIN; 
      drawMainScreen(); 
    }
  }
  if (mode == MODE_MAIN) {
    if (upButton.changed() && upButton.read()) {
      holdingRegs[DISP_ONOFF] = 255;
      
      if (setTemperature < MAX_TEMPERATURE) setTemperature++;
      holdingRegs[SET_TEMP] = setTemperature;
      updateSetTemp();
      updateCircleColor();
    }
    if (downButton.changed() && downButton.read()) {
      holdingRegs[DISP_ONOFF] = 255;
      
      if (setTemperature > MIN_TEMPERATURE) setTemperature--;
      holdingRegs[SET_TEMP] = setTemperature;
      updateSetTemp();
      updateCircleColor();
    }
  } else {
    if (upButton.changed() && upButton.read()) {
      holdingRegs[DISP_ONOFF] = 255;
      
      if (modbusId < 255) modbusId++;
      updateModbusAddr();
    } 
    if (downButton.changed() && downButton.read()) {
      holdingRegs[DISP_ONOFF] = 255;
      
      if (modbusId > 1) modbusId--;
      updateModbusAddr();
    }
  }
}

void drawMainScreen() {
  tft.fillScreen(ILI9341_BLACK);
  
  updateCircleColor();
  updateSetTemp();
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
