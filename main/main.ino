#include <SoftwareSerial.h>
#include <ArduinoJson.h>
SoftwareSerial BT(8, 9);

const int INPUT_PIN = A0;

void setup()
{
  Serial.begin(9600);
  
  BT.begin(9600);
  pinMode(A0, INPUT);
}

void loop() 
{
  String serialRx = "";

  if (BT.available())
  {
    serialRx = BTSerialRead();
    
    if(serialRx == "status"){

      const int capacity = JSON_OBJECT_SIZE(6);
      StaticJsonBuffer<capacity> jb;
      JsonObject& obj = jb.createObject();

      obj.set("status", "OK");
      obj.set("batteryLevel", 67);
      obj.set("speed", 14.8);
      obj.set("avgSpeed", 7.5);
      obj.set("tripDistance", 12.1);
      obj.set("odoMiles", analogRead(INPUT_PIN));
      
      String serialTx = "";
      obj.printTo(serialTx);
            
      BTSerialWrite(serialTx);
      
    }
  }

  delay(100);
}

String BTSerialRead()
{
  String outMessage = "";
  while (BT.available() > 0) {
    char inChar = BT.read();
    outMessage.concat(inChar);
  }

  return outMessage;  
}

void BTSerialWrite(String outMessage)
{
  if (outMessage != "") {
    BT.println(outMessage);
    char* CharString;
    outMessage.toCharArray(CharString, outMessage.length());
    BT.write(CharString);
  }
}
  
