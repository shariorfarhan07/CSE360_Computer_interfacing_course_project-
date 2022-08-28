#define BLYNK_TEMPLATE_ID "TMPLXha9KzzL"
#define BLYNK_DEVICE_NAME "CSE360 project"
#define BLYNK_AUTH_TOKEN "oLyZ-KK7qXpYM0IJDARjIJMGoR_hzk5p"
#define BLYNK_PRINT Serial
#define MQ2pin (15)
#define buzzer (18)


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd (0x27, 20,4);  //

//bmp start
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
//mq start
//#include <Esplora.h>
//#include <MQ2.h>
//int pin = 15;
////int lpg, co, smoke;
//
//MQ2 mq2(pin);

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)


Adafruit_BMP280 bmp; // I2C
//bmp end
// You  get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).



char auth[] =BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "SHF";
char pass[] = "farhan12";

void setupBmp(){
    lcd. begin ();
    unsigned status;
   status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  }
BLYNK_WRITE(V10)
{
  String value = param.asStr();
  lcd.clear();

  lcd. print ( value);
  delay(5000);
}
void setup()
{
  
  Serial.begin(9600);
//  analogRead(pin);
  Blynk.begin(auth, ssid, pass);
//  mq2.begin();
  setupBmp();
  pinMode(MQ2pin,INPUT);
  pinMode(buzzer,OUTPUT);
  
}

void loop()
{
  Blynk.run();
   float sensorVa= analogRead(MQ2pin); // read analog input pin 0
  
  if(sensorVa > 300)
  {
    digitalWrite(buzzer,1);
  }else{
    digitalWrite(buzzer,0);
    }
  
  
//  Serial.println(123);
  
  delay(1000);
  String tempreture="temp:"+ String(bmp.readTemperature())+"*C";
  String Humidity="Humidity:"+ String(bmp.readAltitude(1013.25)/5)+"%";
  String air="pressure:"+ String(bmp.readPressure() )+"Pa";
  String smoke="Somke:"+ String(sensorVa );
//
//  String lpg="lpg:"+ String(mq2.readLPG())+"*C";
//  String co ="co:"+ String(mq2.readCO())+"%";
//  String smoke ="smoke :"+ String(mq2.readSmoke() )+"Pa";
  
  //String co="temp:"+ String(millis())+"C";

  
  Blynk.virtualWrite(V0,tempreture);
  Blynk.virtualWrite(V1, Humidity);
  Blynk.virtualWrite(V3, air);
  Blynk.virtualWrite(V2, smoke);
//  lcd display
  lcd.clear();
  lcd. setCursor (0, 0);
  lcd. print ( tempreture );
  lcd. setCursor (0, 1);
  lcd. print ( Humidity );
  lcd. setCursor (0, 2);
  lcd. print ( air );
  lcd. setCursor (0, 3);
  lcd. print ( smoke );
// end lcd display

//  Blynk.virtualWrite(V2,lpg);
//  Blynk.virtualWrite(V4, co);
//  Blynk.virtualWrite(V5,smoke);
//  timer.run();
}
