build_flags = -I/home/gru/Documents/Arduino/hardware/arduino/avr/cores/arduino

#include <LiquidCrystal.h>
#include "DS3231.h"
#include "time.h"
//Adafruit DHT library https://github.com/nethoncho/Arduino-DHT22/tree/a655c6299b019cf53173139dad824466486591c8
#include "DHT.h"
#define DHTPIN 8 // Ich nutze Digital-PIN 8
#define DHTTYPE DHT11
#define sensorPin A1
#define BAT_PIN A2
#define Photo_PIN A3



DHT dht(DHTPIN, DHTTYPE);
 // initialize the library with the numbers of the interface pins
 LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

float h,t;

const int pingPin = 6; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 7; // Echo Pin of Ultrasonic Sensor

const int AnalogPin = A0;  // Analog input pin that the potentiometer is attached to
const int LEDPin = 13; // Analog output pin that the LED is attached to
const int LEDPin1 = 10;
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int val = 0;

float t_max = 0;
float t_min = 20;
bool start = 1;

void setup(){
  // set up the LCD's number of columns and rows:
  dht.begin();
  lcd.begin(16, 2);
  Serial.begin(9600);
    pinMode(10, INPUT);
    pinMode(A0, INPUT);
    pinMode(A2, INPUT);
    lcd.begin(16, 2);
    pinMode(LEDPin, OUTPUT);  // sets the pin as output
    pinMode(LEDPin1, OUTPUT);

}

void loop(){
  int zeit = 2000;
  // Zeit anzeigen

    delay(zeit);
  //Lese die Luftfeuchtigkeit
  float h = dht.readHumidity();

  // Lese die Temperatur in Celsius
  float t = dht.readTemperature();
  
  
  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print("Temperature: ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(t,1);
  lcd.print("\xDF");
  lcd.print("C");
  delay(zeit);

if(start==true){
  t_max = t;
  t_min = t;
  start = 0;
}

 if(t>t_max){
    t_max = t;

  }  
    clear_display();
    lcd.setCursor(0,0);
    lcd.print("Max. Temp.:");
    lcd.setCursor(0,1);
    lcd.print(t_max,1);
    lcd.print("\xDF");
    lcd.print("C");
    delay(zeit);
    
 if(t < t_min){
    t_min = t;
 }
    clear_display();
    lcd.setCursor(0,0);
    lcd.print("Min. Temp.:");
    lcd.setCursor(0,1);
    lcd.print(t_min,1);
    lcd.print("\xDF");
    lcd.print("C");
    delay(zeit);

  
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  //Serial.print(t);
  //Serial.print(h);
  lcd.setCursor(0,0);
  lcd.print("Humidity:");
  lcd.setCursor(0,1);
  lcd.print(h,1);
  lcd.print("%");
  delay(zeit);
  
 // Abstandsmessung
   float duration, cm;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   float cl, clcm;
   
   cl = 20.058*sqrt(t+273.15);
   clcm = 1/(cl*0.0001);
   cm = duration/clcm/2;

   if(cm < 30){
   lcd.setCursor(0,0);
   lcd.print("                ");
   lcd.setCursor(0,1);
   lcd.print("                ");
   lcd.setCursor(0,0);
   lcd.print("Distance:");
   lcd.setCursor(0,1);
   lcd.print(cm,1);
   lcd.print(" cm");
   delay(zeit);
   }else{
   lcd.setCursor(0,0);
   lcd.print("                ");
   lcd.setCursor(0,1);
   lcd.print("                ");
   lcd.setCursor(0,0);
   lcd.print("Distance:");
   lcd.setCursor(0,1);
   lcd.print(cm,0);
   lcd.print(" cm");
   delay(zeit);
   }
  //   if(cm<10){
  //    val = analogRead(AnalogPin);  // read the input pin
  //    analogWrite(LEDPin, val / 2); // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  //    delay(zeit);
  //   }else{
  //    val = analogRead(AnalogPin);
  //    analogWrite(LEDPin, 0);
  //    analogWrite(LEDPin1, val);
  //    delay(zeit);
  //   }
  //   
  //   analogWrite(LEDPin, 0);
  //   analogWrite(LEDPin1, 0);
  
// Lichintensität
  float input_PP, ldr_voltage, resistorVoltage, ldr_resistance, ldr_lux, vout, vin;
  int lux;
  
  input_PP = analogRead(Photo_PIN);
  vout = input_PP*51024;
  lux = sensorLight(input_PP);
  Serial.print("Light: ");
  Serial.println(lux);

  clear_display();
  lcd.setCursor(0,0);
  lcd.print("Light intensity:");
  lcd.setCursor(0,1);
  lcd.print(lux);
  lcd.print(" lux");
  delay(zeit); 

   
  // Batterycheck
  float Messung, Spannung;
  Serial.print("Messung= ");
  Messung = analogRead(BAT_PIN);
  Serial.println(Messung);
  Spannung = Messung * 5 /1024;
  Serial.print("Spannung= ");
  Serial.print(Spannung);
  Serial.println(" V");
  
 if(Spannung > 0 || Spannung < 0){
  clear_display();
   lcd.setCursor(0,0);
   lcd.print("Battery status:");
  if(Spannung >= 1.1){
    lcd.setCursor(0,1);
    lcd.print("                  ");
    lcd.setCursor(0,1);
    lcd.print("U= ");
    lcd.print(Spannung,2);
    lcd.print(" V");
    analogWrite(LEDPin1, 250);
    delay(zeit);
    lcd.setCursor(0,1);
    lcd.print("                  ");
    lcd.setCursor(0,1);
    lcd.print("fine");
  }else if(Spannung > 0 && Spannung < 1.1){
    analogWrite(LEDPin, 250);
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("U= ");
    lcd.print(Spannung,2);
    lcd.print(" V");
    delay(zeit);
    lcd.setCursor(0,1);
    lcd.print("                  ");
    lcd.setCursor(0,1);
    lcd.print("dead");

   }else{   
    lcd.setCursor(0,1);
    lcd.print("                  ");
    lcd.setCursor(0,1);
    lcd.print("false connection");
    analogWrite(LEDPin1, 0);
    analogWrite(LEDPin, 0);
  }
  delay(zeit);
  analogWrite(LEDPin1, 0);
  analogWrite(LEDPin, 0);
 }
}

void clear_display(){
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,0);
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
}

void clear_first_line(){
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,0);
}

void clear_second_line(){
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
}



// Light detector
int sensorLight(int raw){
  // Conversion rule
  float Vout = float(raw) * (float(5) / float(1023));// Conversion analog to voltage
  float RLDR = (10000 * (5 - Vout))/Vout; // Conversion voltage to resistance
  int phys=500/(RLDR/1000); // Conversion resitance to lumen
  return phys;
}
