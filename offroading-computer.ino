#include <Adafruit_10DOF.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdlib.h>
#include <LiquidCrystal_I2C.h>
#include <RunningAverageEA.h>
#include <OneWire.h>
   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

OneWire ds(2); // DS18S20 Temperature chip i/o

byte outsideTempAddr[8];

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

int altCurrent;

unsigned long timer;

float outsideTempCelsius = 0;

float xAngle = 0;
float yAngle = 0;
float yOffset = 0;
float xOffset = 0;

int altAdjustment = 0;

RunningAverageEA altitudeRA(100);
RunningAverageEA accelerationRA(30);


void setup(void) 
{
  Serial.begin(9600);
  
  lcd.begin(20,4); 
  
  //printWelcomeMessage();
  
  gyro.enableAutoRange(true);
  gyro.begin();
  
  bmp.begin();
  
  mag.enableAutoRange(true);
  mag.begin();

  accel.begin();
  
  if ( !ds.search(outsideTempAddr)) {
      ds.reset_search();
  }

  ds.reset();
  ds.select(outsideTempAddr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
}



void loop(void) 
{  
  timer = micros();
  String altitude = getAltitude();
  String temperature = getTemperature();
  String gyroX = getGyroX();
  String gyroY = getGyroY();
  String heading = getCompass();
  String acceleration = getAcceleration();
  String outsideTemp = getOutsideTemp();
  
  if (digitalRead(4) == HIGH){ 
    lcd.clear();
    delay(1000); //to allow button to be unpressed
    getGyroX();
    getGyroY();// get new values after button press vibrations
    altAdjustment = 230 - altCurrent; //230 ft for my home's altitude
    altitudeRA.clear();
    yOffset = -yAngle;
    xOffset = -xAngle;
    
  }
  if(timer % 5 == 0)
  {
    printScreen(altitude, temperature, gyroX, gyroY, heading, acceleration, outsideTemp);
  }
  
  timer = micros(); 
  delay(10);
}


String getAltitude(void)
{
  sensors_event_t bmpEvent;
  bmp.getEvent(&bmpEvent);
  
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  float altitudeInFeet = bmp.pressureToAltitude(seaLevelPressure, bmpEvent.pressure, outsideTempCelsius);
  altitudeInFeet = (altitudeInFeet* 3.28) + altAdjustment ;

  altCurrent = altitudeInFeet;
  altitudeRA.addValue(altitudeInFeet);
  
  char altitudeString[4];
  dtostrf(altitudeRA.getAverage(),4,0,altitudeString);
  return altitudeString;
}

String getTemperature(void)
{
  sensors_event_t bmpEvent;
  bmp.getEvent(&bmpEvent);
  
  float temperature;
  bmp.getTemperature(&temperature);
  temperature = (temperature * 2) +32;
  
  char temperatureString[4];
  dtostrf(temperature,4,0,temperatureString);
  return temperatureString;
}

String getOutsideTemp(void)
{
  byte outsideTempData[12];

  ds.reset();
  ds.select(outsideTempAddr);    
  ds.write(0xBE); 

  for (byte i = 0; i < 9; i++) {       
    outsideTempData[i] = ds.read();
  }

  int HighByte, LowByte, TReading, SignBit, Tc_100, whole, fract;
  
  LowByte = outsideTempData[0];
  HighByte = outsideTempData[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;
  
  float outsideTemp = Tc_100;
  outsideTemp = (outsideTemp / 100);
  outsideTempCelsius = outsideTemp; 
  outsideTemp = (outsideTemp * 2) + 32;
  
  ds.reset();
  ds.select(outsideTempAddr);
  ds.write(0x44,1);        

  char outsideTempString[4];
  dtostrf(outsideTemp, 3,1, outsideTempString);
  
  return outsideTempString;
}

String getGyroX(void)
{
  
  sensors_event_t gyroEvent; 
  gyro.getEvent(&gyroEvent); 
  
  sensors_event_t acclEvent; 
  accel.getEvent(&acclEvent);
  float accelX = acclEvent.acceleration.y ;
  float gyroValueX = gyroEvent.gyro.x;
  
  int loopTime = ((micros() - timer));
  
  xAngle = kalmanCalculate(accelX, gyroValueX, loopTime, xAngle);
  xAngle += xOffset;
  
  char gyroXString[4];
  dtostrf(xAngle * -10 , 3,1, gyroXString);
  
  return gyroXString;
}

String getGyroY(void)
{
  sensors_event_t gyroEvent; 
  gyro.getEvent(&gyroEvent); 
  
  sensors_event_t acclEvent; 
  accel.getEvent(&acclEvent);
  float accelY = acclEvent.acceleration.x ;
  float gyroValueY = gyroEvent.gyro.y;
  
  int loopTime = ((micros() - timer));
  
  yAngle = kalmanCalculate(accelY, gyroValueY, loopTime, yAngle);
  yAngle += yOffset;
  char gyroYString[4];
  dtostrf(yAngle * 10, 3,1, gyroYString);
  return gyroYString;
}

String getCompass(void)
{
  sensors_event_t magEvent; 
  mag.getEvent(&magEvent);

  float Pi = 3.14159;
  
  float heading = (atan2(magEvent.magnetic.y,magEvent.magnetic.x) * 180) / Pi;
  heading = heading + 90 + 14;
  
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  char compassString[4];
  dtostrf(heading, 4,0,compassString);
  return compassString;
}

String getAcceleration(void)
{
  sensors_event_t acclEvent; 
  accel.getEvent(&acclEvent);
  float value =   abs( acclEvent.acceleration.y  ) ;
  accelerationRA.addValue(value);

  char accelerationString[4];
  dtostrf( accelerationRA.getStandardDeviation() * 100, 4,0,accelerationString);
  return accelerationString;
}




