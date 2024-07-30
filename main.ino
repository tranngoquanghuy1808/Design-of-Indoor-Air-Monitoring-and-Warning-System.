//---------Thu vien OLED---------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for SSD1306 display connected using I2C
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include "MQ7.h"

#define A_PIN 33
#define VOLTAGE 5

// init MQ7 device
MQ7 mq7(A_PIN, VOLTAGE);

//DHT 11
#include <Adafruit_Sensor.h>  
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 22 (AM2302)


#define led_red 12
#define led_yellow 14
#define led_green 27

#define buzzer_pin 26


#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6zO8upUHH"
#define BLYNK_TEMPLATE_NAME "bai1"
#define BLYNK_AUTH_TOKEN "vFsObWA8AG7-0KPwRoAb4tw5AYg5sM6H"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "Wi";
char pass[] = "123456789";

float Ppm = 0;
float t = 0;
float h = 0;
float Dust = 0;

BlynkTimer timer;

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
char s[32];

int measurePin = 35;
int ledPower = 16;
 
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
 
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

int warningCO =0;
int warningDust = 0;
int warningTemperature = 0;
int warningHumidity = 0;

void setup(){
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(1000L, myTimerEvent);

  pinMode(buzzer_pin,OUTPUT);
  pinMode(led_red,OUTPUT);
  pinMode(led_yellow,OUTPUT);
  pinMode(led_green,OUTPUT);
  
  digitalWrite(led_yellow,LOW);
  digitalWrite(led_green,LOW);
  digitalWrite(led_red,LOW);

  
  pinMode(ledPower,OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }


	Serial.println("Calibrating MQ7");
	mq7.calibrate();		// calculates R0
	Serial.println("Calibration done!");
  dht.begin();
  delay(20);
}


void loop(){
  Blynk.run();
  timer.run();
  Print_CO(); //Khi CO
  Print_H_T();  //Nhiet do va do am
  Print_PM(); //Bui min 2.5
  Display_Oled();
  ledWarning();
  delay(1000);
}


void myTimerEvent()
{
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
  Blynk.virtualWrite(V2, Ppm);
  Blynk.virtualWrite(V3, Dust);
}



void Buzzer(){
  tone(buzzer_pin,900,900);
  delay(500);
  noTone(buzzer_pin);
}


void Display_Oled(){
  //Display OLED
  display.clearDisplay();
//Temp
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Temp: ");
  display.setCursor(65, 0);
  display.println(t);

//Humidity
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.println("Hum: ");
  display.setCursor(55, 16);
  display.println(h);

//CO
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 32);
  display.println("PPM: ");
  display.setCursor(55, 32);
  display.println(Ppm);

//PM 2.5
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 48);
  display.println("PM: ");
  display.setCursor(55, 48);
  display.println(Dust);
  
  display.display();
}



void ledWarning(){
  if(warningCO == 3 ||warningDust ==3 ||warningTemperature ==3 ||warningHumidity==3){
    digitalWrite(led_red,HIGH);
    Buzzer();
    digitalWrite(led_green,LOW);
    digitalWrite(led_yellow,LOW);
  }else{
    digitalWrite(led_red,LOW);
    if(warningCO == 2  ||warningDust ==2||warningTemperature ==2 ||warningHumidity==2){
      digitalWrite(led_yellow,HIGH);
      digitalWrite(led_green,LOW);
    }else{
      digitalWrite(led_yellow,LOW);
      digitalWrite(led_green,HIGH);
    }
  }
}




void Print_CO(){
  Serial.print("CO: ");
  Serial.print("PPM = "); 
  Ppm =mq7.readPpm();
  Serial.println(Ppm);
  delay(1000);
	Serial.println(""); 	// blank new line
  if(Ppm>0 && Ppm<1000){
    if(Ppm<=80){
      warningCO =1;
    }else if(Ppm>80 && Ppm <=300){
      warningCO =2;
    }else if(Ppm>300) warningCO =3;
  }else warningCO =0;
}




void Print_H_T(){
  delay(delayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    t = event.temperature;
    Serial.print(t);
    delay(1000/4);
    Serial.println(F("°C"));
    if(t>=0 && t<60){
      if(t<=20){
        warningTemperature =2;
      }else if(t<10){
        warningTemperature = 3;
      }else if(t>20 && t <=40){
        warningTemperature =1; 
      }else if(t>40) warningTemperature =3; 
    }else warningTemperature =0;
    
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    h = event.relative_humidity;
    Serial.print(h);
    
    delay(1000/4);
    Serial.println(F("%"));
    if(h>=10 && h<90){
      if(h<40) {
        warningHumidity =2;
      }else  if(h>=40 && h <=70){
        warningHumidity =1;
      }else{
        warningHumidity =2;
      } 
    }else warningHumidity =0;
  }
  
}




void Print_PM(){
  digitalWrite(ledPower,LOW); // Bật IR LED
  delayMicroseconds(samplingTime);  //Delay 0.28ms
  voMeasured = analogRead(measurePin); // Đọc giá trị ADC V0
  delayMicroseconds(deltaTime); //Delay 0.04ms
  digitalWrite(ledPower,HIGH); // Tắt LED
  delayMicroseconds(sleepTime); //Delay 9.68ms
 
  // Tính điện áp từ giá trị ADC
  calcVoltage = voMeasured * (5.0 / 4096); //Điệp áp Vcc của cảm biến (5.0 hoặc 3.3)
 
  // Linear Equation http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = (170 * calcVoltage) - 0.1;

  if (dustDensity < 0 )
  dustDensity = 0;
  if (dustDensity > 500)
  dustDensity = 500;
  String dataString = "";
  dataString += dtostrf(dustDensity*1.0, 5, 2, s);
  Dust = dustDensity*1.0;
  dataString += "ug/m3";
  Serial.print("PM 2.5: ");
  Serial.println(dataString);
  if(Dust>=0 && Dust<600){
    if(Dust<=35){
      warningDust =1;
    }else if(Dust>35 && Dust <=60){
      warningDust =2;
    }else if(Dust>70) warningDust =3;
  }else{
      warningDust = 0;
  }
}
