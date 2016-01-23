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
#include <avr/eeprom.h>
#include <math.h>   
   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

#define ALT_ADJUSTMENT_ADDRESS 3

OneWire ds(2); // DS18S20 Temperature chip i/o

byte outsideTempAddr[8];

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

unsigned long timer;

float outsideTempCelsius = 0;

float GYRO_SENSITIVITY = 0.7;
float xAngle = 0;
float yAngle = 0;
int yOffset = 0;
int xOffset = 10;

float altAdjustment = 0;

RunningAverageEA altitudeRA(30);
RunningAverageEA accelerationRA(30);
RunningAverageEA pitchRA(10);
RunningAverageEA rollRA(10); //more resposive gyros



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
 
  altAdjustment = eeprom_read_dword((uint32_t *) ALT_ADJUSTMENT_ADDRESS ); 
}



void loop(void) 
{  
  timer = micros();
  String altitude = getAltitude();
  String temperature = getTemperature();
  String gyroX = getGyroPitch();
  String gyroY = getGyroRoll();
  String heading = getCompass();
  String acceleration = getAcceleration();
  String outsideTemp = getOutsideTemp();
  
  if (digitalRead(4) == HIGH){ 
    lcd.clear();
    delay(1000); //to allow button to be unpressed
    altAdjustment = -getFloatAltitude();
    altitudeRA.clear();
    
    delay(1000); //to allow button to be unpressed
    
    if (digitalRead(4) == HIGH){ 
     addAltitude();
    }
    
    altitudeRA.clear();
    eeprom_write_dword((uint32_t *) ALT_ADJUSTMENT_ADDRESS, altAdjustment );
  }
  
  if(timer % 5 == 0)
  {
    printScreen(altitude, temperature, gyroX, gyroY, heading, acceleration, outsideTemp);
  }
  
  timer = micros(); 
}

void addAltitude(void) { 
  while (digitalRead(4) == HIGH){ 
    altAdjustment += 100;
    
    float alt = getFloatAltitude();
     
    char altitudeString[4];
    
    dtostrf( alt + altAdjustment ,4,0,altitudeString);

    printScreen(altitudeString, "-", "-", "-", "-", "-", "-");
    delay(300);
  }
}

String getAltitude(void)
{
  float altitudeInFeet = getFloatAltitude();
  int temp = altitudeInFeet + altAdjustment;
  altitudeRA.addValue(temp);
  
  char altitudeString[10];
  dtostrf(altitudeRA.getAverage(),4,0,altitudeString);
  return altitudeString;
}

float getFloatAltitude(void)
{
  sensors_event_t bmpEvent;
  bmp.getEvent(&bmpEvent);
  
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  float altitudeInFeet = bmp.pressureToAltitude(seaLevelPressure, bmpEvent.pressure, outsideTempCelsius);
  altitudeInFeet = (altitudeInFeet * 3.28) ;
  return altitudeInFeet;
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
  dtostrf(outsideTemp, 3, 1, outsideTempString);
  
  return outsideTempString;
}

float smoothGyroX;
float smoothAccelX;

String getGyroPitch(void)
{
  
  sensors_event_t gyroEvent; 
  gyro.getEvent(&gyroEvent); 
  
  sensors_event_t acclEvent; 
  accel.getEvent(&acclEvent);

  
  float accZ=float(acclEvent.acceleration.z) * 0.01; 
  float accY=float(acclEvent.acceleration.y) * 0.01;
  float accelX = atan2(accY,accZ);
  
  float gyroValueX = float(gyroEvent.gyro.x) * 0.5;
  
  smoothAccelX = gyroSmooth(accelX, GYRO_SENSITIVITY , smoothAccelX);
  smoothGyroX = gyroSmooth(gyroValueX, GYRO_SENSITIVITY, smoothGyroX);
  
  int loopTime = (micros() - timer);
  
  xAngle = kalmanCalculate(smoothAccelX, smoothGyroX, loopTime, xAngle);
  
  xAngle = (xAngle * -60) - 5; 

  pitchRA.addValue(xAngle);
  char gyroXString[4];
  dtostrf(pitchRA.getAverage() , 4,0, gyroXString);
  
  
  return gyroXString;
}


float smoothGyroY;
float smoothAccelY;

String getGyroRoll(void)
{
  sensors_event_t gyroEvent; 
  gyro.getEvent(&gyroEvent); 
  
  sensors_event_t acclEvent; 
  accel.getEvent(&acclEvent);
  float accZ = float(acclEvent.acceleration.z) * 0.01; 
  float accX = float(acclEvent.acceleration.x) * 0.01;
  float accelY = atan2(accX,accZ);
  
  float gyroValueY = float(gyroEvent.gyro.y) * 0.5;
  
  smoothAccelY = gyroSmooth(accelY, GYRO_SENSITIVITY , smoothAccelY);
  smoothGyroY = gyroSmooth(gyroValueY, GYRO_SENSITIVITY, smoothGyroY);
   
  int loopTime = (micros() - timer);
  
  yAngle = kalmanCalculate(smoothAccelY, smoothGyroY, loopTime, yAngle);
 
  yAngle = yAngle * 60;
  rollRA.addValue(yAngle);
  
  char gyroYString[4];
  dtostrf(rollRA.getAverage(), 4,0, gyroYString);
  return gyroYString;
}

String getCompass(void)
{
  sensors_event_t magEvent; 
  mag.getEvent(&magEvent);
  
  float heading = (atan2(magEvent.magnetic.y,magEvent.magnetic.x) * 180) / PI;
  heading = heading + 90 + 14; // orientation and declanation
  
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
  float value =   pow(abs( acclEvent.acceleration.y  ), 2.5);
  accelerationRA.addValue(value);

  char accelerationString[4];
  dtostrf( accelerationRA.getStandardDeviation() * 100, 4,0,accelerationString);
  return accelerationString;
}

float gyroSmooth(float data, float filterVal, float smoothedVal){
  if(abs(data - smoothedVal) > 1){ // throw out outliers
    return smoothedVal;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}

