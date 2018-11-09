/* Bluetooth */
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

/* Battery */
#define BATTERY_PIN A7
#define BATTERY_PIN_SAMPLES 20
#define BATTERY_PIN_VOLTAGE_DIVIDER 16
#define ANALOG_PIN_BASE_VOLTAGE 3.94
#define MIN_BATTERY_VOLTAGE 43.2
#define MAX_BATTERY_VOLTAGE 50.4

/* Speed */
#define SPEED_SENSOR_PIN 2
#define CIRCUMFERENCE 2068

/* Battery */
int batteryPercentage = 0;
float batteryVoltage = 0.0;
unsigned int batteryPinSamplesSum = 0;
unsigned char batteryPinSamplesCount = 0;
unsigned long batteryVoltageSampleLastUpdate;
unsigned long batteryVoltageLastPrint;

/* Speed */
volatile float currentSpeed, avgSpeed, tripDistance, odoMiles;
volatile unsigned long speedSensorLastUpdate;

/* Bluetooth */
volatile unsigned long statusResponseSentAt;
SoftwareSerial BT(8, 9); // RX, TX

void setup()
{
  pinMode(BATTERY_PIN, INPUT);
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speedSensor, FALLING);

  BT.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  batterySensor();
  sendStatus();

  if (millis() - speedSensorLastUpdate > 1500)
  {
    currentSpeed = 0;
  }
}

void batterySensor()
{
  if ((batteryPinSamplesCount < BATTERY_PIN_SAMPLES) && (millis() - batteryVoltageSampleLastUpdate > 50))
  {
    batteryPinSamplesSum += analogRead(BATTERY_PIN);
    batteryPinSamplesCount++;
    batteryVoltageSampleLastUpdate = millis();
  }

  if ((batteryPinSamplesCount == BATTERY_PIN_SAMPLES))
  {
    batteryVoltage = (((float)batteryPinSamplesSum / (float)BATTERY_PIN_SAMPLES * ANALOG_PIN_BASE_VOLTAGE) / 1024.0) * BATTERY_PIN_VOLTAGE_DIVIDER;
    batteryPercentage = map(batteryVoltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE, 0, 100);
    if (batteryPercentage < 0)
    {
      batteryPercentage = 0;
    }

    avgSpeed = batteryVoltage;

    batteryPinSamplesCount = 0;
    batteryPinSamplesSum = 0;
  }
}

void speedSensor()
{
  unsigned long currentMillis = millis();

  if (currentMillis - speedSensorLastUpdate > 80)
  {
    tripDistance = tripDistance + (CIRCUMFERENCE / 1000000 * 0.621371);
    odoMiles = tripDistance;

    currentSpeed = CIRCUMFERENCE / 1000 / ((float)(currentMillis - speedSensorLastUpdate) / 1000) * 3.6 * 0.621371; // the current speed in miles

    if(tripDistance > 0)
    {
      avgSpeed = tripDistance / (((float)(currentMillis) / 1000) * (1 / 3600));
    }else{
      avgSpeed = 0;  
    }
    
    speedSensorLastUpdate = millis();
  }
}

void sendStatus()
{
  if(millis() - statusResponseSentAt > 500)
  {
    const int capacity = JSON_OBJECT_SIZE(6);
    StaticJsonBuffer<capacity> jb;
    JsonObject& obj = jb.createObject();
  
    obj.set("status", "OK");
    obj.set("batteryLevel", batteryPercentage);
  
    obj.set("tripDistance", tripDistance);
    obj.set("odoMiles", odoMiles);
  
    obj.set("currentSpeed", currentSpeed);
    obj.set("avgSpeed", avgSpeed);
  
    String serialTx = "";
    obj.printTo(serialTx);
  
    BT.println(serialTx);

    statusResponseSentAt = millis();
  }
}

String BTSerialRead()
{
  String outMessage = "";
  while (BT.available() > 0)
  {
    char inChar = BT.read();
    outMessage.concat(inChar);
  }

  return outMessage;
}
