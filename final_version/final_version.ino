#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <MPU6050.h>
#include <elapsedMillis.h>

MPU6050 mpu;
elapsedMillis timeElapsed;

SoftwareSerial mySerial(11,12); // rx,tx
//SoftwareSerial gpsSerial(3,2); // rx,tx
//TinyGPS gps; // create gps object 
const int buzzer = 8;
const int sendFlag = 6;
const int speedLimit = 20;
int bugFlag = 0;
float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object  
String serialMessage;

void setup()
{
  mySerial.begin(9600);   // Setting the baud rate of GSM Module  
  //gpsSerial.begin(9600);
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  pinMode(sendFlag,INPUT);
  pinMode(buzzer,OUTPUT);
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  
  checkSettings();
  //timeElapsed = 0;
  //RecieveMessage();
  delay(100);
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}

void loop(){
    Vector rawAccel = mpu.readRawAccel();
    Vector normAccel = mpu.readNormalizeAccel();
    int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    Serial.print(" \nSpeed = ");
    Serial.print(pitch);
    
    serialMessage = "";
    while(mySerial.available()>0){
      char c = mySerial.read();
      Serial.write(c);
      serialMessage = serialMessage + c;
      delay(100);
    }
    if (serialMessage.indexOf("track")>0){
        SendMessage(pitch,2);
      }


//    while(gpsSerial.available()){ // check for gps data 
//      if(gps.encode(gpsSerial.read()))// encode gps data 
//        {  
//          gps.f_get_position(&lat,&lon); // get latitude and longitude 
//        } 
//    }
    //bugFlag = 0;
    
    bugFlag = digitalRead(sendFlag);
    
    if (bugFlag == true and pitch >= speedLimit){
        bugFlag = 0;
        Serial.println("35 seconds delay!");
        delay(35000);
        timeElapsed = 0;
        while(timeElapsed < 5000){
          Serial.println("Checking whether call was picked up!"); 
          bugFlag = digitalRead(sendFlag);
          if (bugFlag == true){
            break;
            }
        }
        if (bugFlag and pitch >= speedLimit){
            Serial.println("Message Sent!");
            digitalWrite(buzzer,HIGH);
            SendMessage(pitch,1);
            delay(1000);
            digitalWrite(buzzer,LOW);
            bugFlag = 0;
       }
      
    }
    delay(1000);
}


 void SendMessage(int pitch,int type)
{
  Serial.println("Sending message to host!");
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+918419831263\"\r"); 
  delay(1000);
  String latitude = String(lat,6); 
  String longitude = String(lon,6); 
  Serial.println(latitude+";"+longitude);
  if (type==1){
    mySerial.println("Using Mobile while Driving. GPS Coordinates : "+latitude+","+longitude+". Speed : "+pitch);  
    }
  else{
    mySerial.println("GPS Coordinates : "+latitude+","+longitude+". Speed : "+pitch);  
    } 
  
  delay(100);
  mySerial.println((char)26); // ASCII code of CTRL+Z
  //delay(40000);
}


 void RecieveMessage()
{
  mySerial.println("AT+CNMI=2,2,0,0,0\r"); // AT Command to receive a live SMS
  delay(100);
 }
 
