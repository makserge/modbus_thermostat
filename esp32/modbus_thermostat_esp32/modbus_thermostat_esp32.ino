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

#define TFT_CS   5
#define TFT_DC   25//4
#define TFT_MOSI 23
#define TFT_CLK  18
#define TFT_RST  17//22
#define TFT_MISO 19
#define TFT_LED  2//15  

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
enum { PM_MAIN, PM_OPTION, PM_CLEANING };  // Program modes
enum { BOOT, COOLING, TEMP_OK, HEATING };  // Thermostat modes

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

int X,Y;
uint8_t thermostatMode = BOOT;
 
uint8_t roomTemperature = 21;
uint8_t setTemperature = 20;

uint8_t mode = PM_MAIN;         // program mode
uint8_t modbusId = DEFAULT_ID;  // ID / address for modbus
bool touchPressed = false;
uint8_t timerCleaning = 0;
 
unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array 
 
void setup() {
  holdingRegs[ROOM_TEMP] = roomTemperature;
  holdingRegs[SET_TEMP] = setTemperature;
  holdingRegs[DISP_ONOFF] = 255;
  
  #ifndef _debug 
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
  #endif

  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);    // HIGH to Turn on;

  tft.begin();

  drawMainScreen();
}

void loop() {
  if (touchEvent()== true) { 
    if (touchPressed == false) {
      if (holdingRegs[DISP_ONOFF]) detectButtons();
      holdingRegs[DISP_ONOFF] = 255; // reset BL timer       
    }
    touchPressed = true;    
  } else {
    touchPressed = false;
  }

  //automatic display BL timeout
  if (holdingRegs[DISP_ONOFF]) {
    holdingRegs[DISP_ONOFF]--;
    digitalWrite(TFT_LED, HIGH); // Backlight off 
  } else {
    digitalWrite(TFT_LED, LOW); // Backlight on
  }

  cleaningProcessing();
  
  modbusProcessing(); 
  delay(100); 
}

bool touchEvent() {
  return false;  
}

void cleaningProcessing() {
  // idle timer for screen cleaning
  if (mode == PM_CLEANING) {
    if ((timerCleaning % 10) == 0) {
      tft.fillRect(0,0, 100, 60, ILI9341_BLACK);
      tft.setCursor(10, 50);
      tft.print(timerCleaning / 10);
    }
    if (timerCleaning) {
      timerCleaning--;
    } else {
      drawOptionScreen();
      mode = PM_OPTION;
    }
  }  
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

void detectButtons() {
  // in main program
  if (mode == PM_MAIN){
   // button UP
   if ((X>190) && (Y<50)) {
    if (setTemperature < MAX_TEMPERATURE) setTemperature++;
    holdingRegs[SET_TEMP] = setTemperature;
    updateSetTemp();
    updateCircleColor();
   }
   // button DWN
   if ((X>190) && (Y>200 && Y<250)) {
    if (setTemperature > MIN_TEMPERATURE) setTemperature--;
    holdingRegs[SET_TEMP] = setTemperature;
    updateSetTemp();
    updateCircleColor();
   }
   // button gearwheel
   if ((X<60) && (Y<50)) {
    drawOptionScreen();
    mode = PM_OPTION;
   }
  } else if (mode == PM_OPTION){ 
   // button -
   if ((X<110) && (Y<75)) {
    if (modbusId > 0) modbusId--;
    updateModbusAddr();
   }
   // button +
   if ((X>130) && (Y<75)) {
    if (modbusId < 255) modbusId++;
    updateModbusAddr();
   }
   // button screen cleaning
   if ((Y>85) && (Y<155)) {
     tft.fillScreen(ILI9341_BLACK);
     tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
     tft.setFont(&FreeSansBold24pt7b);  
     mode = PM_CLEANING;    
     timerCleaning = 255;
   }
   // button OK
   if (Y>265) {
     thermostatMode = BOOT;
     drawMainScreen();
     modbus_configure(BAUDRATE, modbusId, 0, TOTAL_REGS_SIZE, 0);      
     mode = PM_MAIN;    
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
  
  // Modbus Address adjustment
  tft.setCursor(10, 20);
  tft.print("MODBUS address");
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(30, 65);
  tft.print("-");
  tft.setCursor(190, 65);
  tft.print("+");
  tft.drawLine(5,80,235,80, ILI9341_WHITE);

  // Screen cleaning idle timer
  tft.setFont(&FreeSansBold12pt7b);  
  tft.setCursor(26, 130);
  tft.print("Screen cleaning");
  tft.drawLine(5,160,235,160, ILI9341_WHITE);

  // OK Button
  tft.setFont(&FreeSansBold24pt7b);
  tft.drawLine(5,260,235,260, ILI9341_WHITE);
  tft.setCursor(90, 310);
  tft.print("OK");
  
  updateModbusAddr();
}

void updateSetTemp() {
  int16_t x1, y1;
  uint16_t w, h;
  String curValue = String(setTemperature);
  int strLen =  curValue.length() + 1; 
  char charArray[strLen];
  curValue.toCharArray(charArray, strLen);
  tft.fillRect(70, 96, 60, 50, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  tft.getTextBounds(charArray, 80, 130, &x1, &y1, &w, &h);
  tft.setCursor(123 - w, 130);
  tft.print(charArray);
}

void updateRoomTemp() {
  int16_t x1, y1;
  uint16_t w, h;
  String curValue = String(roomTemperature);
  int strLen =  curValue.length() + 1; 
  char charArray[strLen];
  curValue.toCharArray(charArray, strLen);
  tft.fillRect(36, 200, 30, 21, ILI9341_ULTRA_DARKGREY);
  tft.setTextColor(ILI9341_WHITE, ILI9341_ULTRA_DARKGREY);
  tft.setFont(&FreeSansBold12pt7b);
  tft.getTextBounds(charArray, 40, 220, &x1, &y1, &w, &h);
  tft.setCursor(61 - w, 220);
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
  tft.fillRect(110, 30, 60, 45, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(115, 65);
  tft.print(modbusId);
}

void drawCircles() {
  //draw big circle 
  unsigned char i;
  if (roomTemperature < setTemperature) {
    // heating - red
    for (i = 0; i < 10; i++) {
      tft.drawCircle(120, 120, 80 + i, ILI9341_RED);
    }
  } else if (roomTemperature > setTemperature) {
    // cooling - blue
    for (i = 0; i < 10; i++) {
      tft.drawCircle(120, 120, 80 + i, ILI9341_BLUE);    
    }
  } else {
    // Temperature ok
    for (i = 0; i < 10; i++) {
      tft.drawCircle(120, 120, 80 + i, ILI9341_GREEN);       
    }
  }

  //draw small 
  tft.fillCircle(60, 200, 40, ILI9341_ULTRA_DARKGREY);

  //draw °C in big circle
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setCursor(130, 100);
  tft.print("o");
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(140, 130);
  tft.print("C");

  // draw room and °C in small circle
  tft.setTextColor(ILI9341_WHITE, ILI9341_ULTRA_DARKGREY);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setCursor(75, 220);
  tft.print("C");
  tft.drawCircle(69,204, 2, ILI9341_WHITE);
  tft.drawCircle(69,204, 3, ILI9341_WHITE);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setCursor(35, 190);
  tft.print("Room");
  updateRoomTemp();
}
