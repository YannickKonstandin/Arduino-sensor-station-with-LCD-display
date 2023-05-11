#include "DHT.h"
#define DHTPIN 8 // Ich nutze Digital-PIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() 
{
  Serial.begin(9600);
}
void loop() 
{
  //Uncomment whatever type you're using!

  //int readData = DHT.readTemperature(DHTPIN); // DHT11

  float t = dht.readTemperature(); // Gets the values of the temperature
  float h = dht.readHumidity(); // Gets the values of the humidity

  // Printing the results on the serial monitor
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.print(" ");
  Serial.print((char)176);//shows degrees character
  Serial.print("C");
  Serial.print("Humidity = ");
  Serial.print(h);
  Serial.println(" % ");
  Serial.println("");

  delay(2000); // Delays 2 secods
}
