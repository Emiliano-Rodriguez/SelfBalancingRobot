#include <LiquidCrystal.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro; 
const int ENA = 10;
const int ENB = 9;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 6;
const int IN4 = 5;



const int rs = 13, en = 12, d4 =4, d5 = 3, d6=2, d7 = 11;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);
 byte smiley[8] = {
    B00000,
    B10001,
    B00000,
    B00000,
    B10001,
    B01110,
    B00000,
  };

  byte smiley1[8] = {
    B11111,
    B11111,
    B10001,
    B10001,
    B10001,
    B11111,
    B11111,
  };

   byte mouth[8] = {
    B11000,
    B11000,
    B01110,
    B00011,
    B00001,
    B00000,
    B00000,
  };

  byte teeth[8] = {
    B00000,
    B00000,
    B00000,
    B11111,
    B11111,
    B00000,
    B00000,
  };

  byte bouth[8] = {
    B00011,
    B00011,
    B00110,
    B01100,
    B11000,
    B00000,
    B00000,
  };


   byte left[8] = {
    B00000,
    B00000,
    B00000,
    B11000,
    B11100,
    B00110,
    B00011,
    B00011,

  };

  byte rigt[8] = {
    B00000,
    B00000,
    B00000,
    B00011,
    B00111,
    B01100,
    B11000,
    B11000,
  };

  





unsigned long currentTime;
unsigned long timeNow;
unsigned long lastTime;
int output;
double errSum, lastErr;
float cycleTime = 0.005; //in seconds
double kp = 5.1567;              
double ki = 13.5;
double kd = 1.35; 
double accelData;
double error;
double dErr;
double accelTotal;
double gyroData;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int sumgx, sumgy, sumgz;
int sumax, sumay, sumaz;
int ayTotal;
float delayTimeCorrect;

static int setpoint = 0;
static int input = 0;
static int lastInput = 0;

#define OUTPUT_READABLE_ACCELGYRO

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  Serial.begin(115200); 
  accelgyro.initialize();
  for (int i = 1; i < 33; i = i + 1)
  {
    accelgyro.getRotation(&gx, &gy, &gz);
    sumgx = sumgx + gx;
  }
  sumgx = sumgx / 32;
  for (int i = 1; i < 33; i = i + 1)
  {
    accelgyro.getAcceleration(&ax, &ay, &az);
    sumay = sumay + ay;
  }
  sumay = sumay / 32;
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT); 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);   


lcd.createChar(0,smiley);
lcd.createChar(1,smiley1);
lcd.createChar(2,mouth);
lcd.createChar(3,teeth);
lcd.createChar(4,bouth);
lcd.createChar(5,left);
lcd.createChar(6,rigt);
lcd.begin(16,2);

  
}

void loop()
{

  currentTime = micros();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelData = (ay-sumay)>>2;// / by 4
  accelTotal = accelData + setpoint;
  gyroData = (gx-sumgx)>>1;// / by 2
  input = (0.98*((lastInput)+((gyroData)*cycleTime)))+(0.02*(accelTotal));
  /*
   * the above takes the last reading of gyro (lastInput), and then takes the current 
   * reading * cycle time (to work out a turning movement in a time frame)
   * and then adds the two together. this works out how much it has turned since last time
   * this is then multiplied by 98% and then added to 2% of the current accel reading.
   */
  calcOutput();
  MotorL298();
  serialInput();
  serialOutput();
  lastErr = error;
  lastInput = input;
  correctTime();
}
void correctTime()
{
  timeNow = micros();
  delayTimeCorrect = 4000-(timeNow - currentTime);
  delayMicroseconds(delayTimeCorrect);
}  

void calcOutput()
{
  error = setpoint - input;
  error = error;
  errSum = errSum + (error * cycleTime); //intergral part sum of errors past
  double dInput = input - lastInput;
  if(dInput == 0) {
  leftEye();
  RightEye();
  mouth1();
  }
  else {
  leftEye();
  RightEye();
  mouth2();
  }
  output = (kp * error) + (ki * errSum) - (kd * dInput); 
}
  
void MotorL298()
{
  output = (output / 2);
  if (output > 255)
  {
    output = 255;
  }
  if (output < -255)
  {
    output = -255;
  }
  if (output < 0)
  {
  
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, (output*-1));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, (output*-1));
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, output);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, output);
    
  }
}

void serialInput()
{
  if (Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch == '4')
    {
      kp = kp + 0.01;
    }
    else if (ch == '1')
    {
      kp = kp - 0.01;
    }
    else if (ch == '5')
    {
      ki = ki + 0.01;
    }
    else if (ch == '2')
    {
      ki = ki - 0.01;
    }
    else if (ch == '6')
    {
      kd = kd + 0.01;
    }
    else if (ch == '3')
    {
      kd = kd - 0.01;
    }
    else if (ch == 'y')
    {
      //neutral1 = neutral1 + 1;
    }
    else if (ch == 'h')
    {
      //neutral1 = neutral1 - 1;      
    }
    else if (ch == 'u')
    {
      //neutral2 = neutral2 + 1;
    }
    else if (ch == 'j')
    {
      //neutral2 = neutral2 - 1;      
    }
    else if (ch == 'd')
    {
      //moveData = moveData + 1;
    }
    else if (ch == 'c')
    {
     // moveData = moveData - 1;
    }
  }  
}

void serialOutput()
{
  static int k = 0;
  if (k == 0)
  {
    Serial.print("P ");
    Serial.print(kp);
  }
  else if (k == 1)
  {
    Serial.print("  I ");
    Serial.print(ki);
  }  
  else if (k == 2)
  {
    Serial.print("  D ");
    Serial.print(kd);
  }
  else if (k == 3)  
  {
    Serial.print("  Raw= ");
    Serial.print(accelTotal);
  }
  else if (k == 4)  
  {
    Serial.print("  in = ");
    Serial.print(input);

  }  
  else if (k == 5)  
  {
    Serial.print("  out = ");
    Serial.print(output);
  }  
  else if (k == 6)  
  {
    Serial.print(" time = ");
    Serial.print(delayTimeCorrect);
  }  
  else if (k == 7)  
  {
    Serial.print("  now= ");
    Serial.print(timeNow);
  }  
  else if (k == 8)  
  {
    Serial.print("  ?= ");
    //Serial.print(delayTimeCorrect);
  }  
  k = k + 1;
  if (k == 7)
  {
    k = 0;
    Serial.println();
  }  
}  


void leftEye() {
  lcd.setCursor(6,0);
  lcd.write(byte(1));
}

void RightEye() {
  lcd.setCursor(9,0);
  lcd.write(byte(1));
}

void mouth1(){
  lcd.setCursor(5,1);
  lcd.write(byte(2));
  lcd.setCursor(6,1);
  lcd.write(byte(3));
  lcd.setCursor(7,1);
  lcd.write(byte(3));
  lcd.setCursor(8,1);
  lcd.write(byte(3));
  lcd.setCursor(9,1);
  lcd.write(byte(3));
  lcd.setCursor(10,1);
  lcd.write(byte(4));
}

void mouth2(){
  lcd.setCursor(5,1);
  lcd.write(byte(6));
  lcd.setCursor(6,1);
  lcd.write(byte(3));
  lcd.setCursor(7,1);
  lcd.write(byte(3));
  lcd.setCursor(8,1);
  lcd.write(byte(3));
  lcd.setCursor(9,1);
  lcd.write(byte(3));
  lcd.setCursor(10,1);
  lcd.write(byte(5));
}

