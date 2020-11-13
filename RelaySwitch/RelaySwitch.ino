/*********************
   Description:
   Uses a relay to turn off appliance for speicifed amount of time
   
    1. Gets millisecond information from user
    2. Uses push button to increase or decrease timing value
    3. Push select button to set value
    4. Push select putton again to start countdown (5sec.)

   Author: Vikram Seshadri
   05/03/2019
**********************/

#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

bool LCDflag = 0;
bool Displayflag = 0;
int msInterrupt = 20;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define relay A0

void setup() {
  // Debugging output
  Serial.begin(9600);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  int time = millis();
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Relay Switch");
  Serial.println("Welcome to Relay Switch");
  delay (2000);
  lcd.clear();

}

void loop() {

  if (Displayflag ==0){
    DisplayLoop();
    Displayflag = 1;
  }
  lcd.setCursor(0, 0);
  lcd.print("Press Select");
  lcd.setCursor(0, 1);
  lcd.print("To Start");

  uint8_t buttons = lcd.readButtons();
  if (buttons & BUTTON_SELECT) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Turn Off Unit in");

    lcd.setCursor(0, 1);
    lcd.print("5");
    delay (1000);

    lcd.setCursor(0, 1);
    lcd.print("4");
    delay (1000);

    lcd.setCursor(0, 1);
    lcd.print("3");
    delay (1000);

    lcd.setCursor(0, 1);
    lcd.print("2");
    delay (1000);

    lcd.setCursor(0, 1);
    lcd.print("1");
    delay (1000);

    digitalWrite(relay, HIGH);
    delay(msInterrupt);
    digitalWrite(relay, LOW);
    delay(msInterrupt);
    digitalWrite(relay, HIGH);
    delay(msInterrupt);
    lcd.clear();
    delay(500);
    Displayflag = 0;
    LCDflag = 0;
  }
}

void DisplayLoop() {
  while (1) {
    uint8_t buttons = lcd.readButtons();
    if (LCDflag == 0 ) {
      lcd.clear();
      lcd.print("Choose msec.");
      lcd.setCursor(0, 1);
      lcd.print("ms");
      LCDflag = 1;
    }
    lcd.setCursor(5, 1);
    lcd.print(msInterrupt);

    if (buttons & BUTTON_UP) {
      delay(200);
      if (msInterrupt >= 100 & msInterrupt < 600 ) {
        msInterrupt += 50;
      }
      else if (msInterrupt >= 600) {
        msInterrupt = 1000;
        LCDflag = 0;
      }
      else {
        msInterrupt += 10;
      }
      Serial.println(msInterrupt);
    }

    else if (buttons & BUTTON_DOWN) {
      delay(200);
      if (msInterrupt >= 150) {
        if (msInterrupt >= 1000) {
          LCDflag = 0;
        }
        msInterrupt -= 50;


      }
      else if (msInterrupt <= 20) {
        msInterrupt = 20;
      }
      else {
        if (msInterrupt == 100) {
          LCDflag = 0;
        }
        msInterrupt -= 10;
      }
      Serial.println(msInterrupt);
    }

    else if (buttons & BUTTON_SELECT) {
      lcd.clear();
      lcd.print("Relay Delay");
      lcd.setCursor(0, 1);
      lcd.print(String(msInterrupt) + "msec.");
      Serial.println(String(msInterrupt) + "msec.");
      delay(2000);
      lcd.clear();
      break;
    }
  }
  lcd.setBacklight(WHITE);
  lcd.clear();
}
