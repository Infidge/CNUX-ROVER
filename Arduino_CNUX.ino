#include <LiquidCrystal.h>
#include <Wire.h>
#include <MiCS6814-I2C.h>
#include "Adafruit_CCS811.h"
int sensorValue;
int buttonState = 0;
const int buttonPin = 9;
int show=0;
Adafruit_CCS811 ccs;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);


void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  Serial.println("CCS811 test");
  lcd.begin(16, 2);
  Wire.begin();
  lcd.setCursor(0, 0);
  lcd.print("   CNUX ROVER"); 
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Init."); 
  delay(300);
  lcd.setCursor(0, 1);
  lcd.print("Init..."); 
  delay(600);
  lcd.setCursor(0, 1);
  lcd.print("Init........"); 
  delay(700);
  lcd.setCursor(0, 1);
  lcd.print("Init...............");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("CCS811 test");
  delay(200);
  if(!ccs.begin()){
    lcd.setCursor(0,1);
    lcd.println("CCS811 fail");
    while(1);
  }
  pinMode(buttonPin, INPUT);
  // Wait for the sensor to be ready
  while(!ccs.available());
  lcd.clear(); 
  }
  

void loop() {
  buttonState = digitalRead(buttonPin);
  sensorValue = analogRead(0);       // read analog input pin 0

if (buttonState == HIGH) {
    
    show=show+1;
    if(show==2){show=0;}
  }
if(show==0){
Serial.print("AirQua: ");
Serial.print(sensorValue, DEC);               // prints the value read
Serial.println(" PPM");
lcd.setCursor(0,0);
lcd.print("Air.Q=");
lcd.print(sensorValue,DEC);
lcd.print("PPM");
lcd.println("       "); 
lcd.print("  ");
if(ccs.available()){
    if(!ccs.readData()){
      lcd.setCursor(0,1);
      lcd.print("CO2: ");
      lcd.print(ccs.geteCO2());
      lcd.print("PPM");

    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }}else{
    lcd.setCursor(0,1);
    lcd.print("                  ");
    if(ccs.available()){
    if(!ccs.readData()){
      lcd.setCursor(0,0);
      lcd.print("TVOC: ");
      lcd.print(ccs.getTVOC());
      lcd.print("PPM");
      lcd.print("         ");

    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
    
    }}
delay(350);

  
}
