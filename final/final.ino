#include <SoftwareSerial.h>
#include <TinyGPS.h>


SoftwareSerial mySerial(9, 10); // rx,tx
SoftwareSerial gpsSerial(4,3); // rx,tx
TinyGPS gps; // create gps object 
const int sendFlag = 6;

float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object  


void setup()
{
  mySerial.begin(9600);   // Setting the baud rate of GSM Module  
  gpsSerial.begin(9600);
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  pinMode(sendFlag,INPUT);
  delay(100);
}


void loop()
{
    while(gpsSerial.available()){ // check for gps data 
      if(gps.encode(gpsSerial.read()))// encode gps data 
        {  
          gps.f_get_position(&lat,&lon); // get latitude and longitude 
        } 
    } 
    
    
    
  if (digitalRead(sendFlag)){
      SendMessage();
  }

  if (mySerial.available()>0)
    Serial.write(mySerial.read());
    delay(100); 
}


 void SendMessage()
{
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+919403148265\"\r"); 
  delay(1000);
  String latitude = String(lat,6); 
  String longitude = String(lon,6); 
  Serial.println(latitude+";"+longitude); 
  mySerial.println("Using Mobile while Driving. GPS Coordinates : "+latitude+","+longitude);
  delay(100);
  mySerial.println((char)26); // ASCII code of CTRL+Z
  delay(1000);
}


 void RecieveMessage()
{
  mySerial.println("AT+CNMI=2,2,0,0,0"); // AT Command to receive a live SMS
  delay(1000);
 }
 
