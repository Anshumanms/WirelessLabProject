#include <SoftwareSerial.h>

SoftwareSerial mySerial(11,12); // rx,tx
const int sendFlag = 6;
String serialMessage;
int pitch = 25,lat=12,lon=23;
void setup()
{
  mySerial.begin(9600);   // Setting the baud rate of GSM Module  
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  pinMode(sendFlag,INPUT);
  delay(100);
}


void loop()
{ 
  RecieveMessage();
  if (digitalRead(sendFlag)){
      //SendMessage();
  }

    serialMessage = "";
    while(mySerial.available()){
      char c = mySerial.read();
      Serial.write(c);
      serialMessage = serialMessage + c;
//      Serial.write(mySerial.read());
      delay(100);
    }
    if (serialMessage.indexOf("track")>0){
        SendMessage(pitch,2);
    }
  delay(1000);
}


 void SendMessage(int pitch,int type){
  
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
  delay(1000);
 }
 
