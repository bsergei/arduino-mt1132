#include <SoftwareSerial.h>
//MT1132 DEMO

int sensorPin1 = A0; // POT VALUE READ
int sensorPin2 = A1; // POT VALUE READ

int PIN_RX  = 5;    // RX PIN (connect to TX on noolite controller)
int PIN_TX  = 4;    // TX PIN (connect to RX on noolite controller)
int led = 13;       // LED PIN
int PIN_BIND = 8;   // BIND BUTTON
int PIN_UNBIND = 6; // UNBIND BUTTON

byte CHANNEL = 0; //channel address 0...31 (MT1132)

SoftwareSerial mySerial(PIN_RX, PIN_TX); // RX, TX pin

void setup() {
  pinMode(led, OUTPUT);
  pinMode(PIN_UNBIND, INPUT_PULLUP);
  pinMode(PIN_BIND, INPUT_PULLUP);
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);

  mySerial.begin(9600);
  //Serial.begin(19200);
}


void on()
{
  sendCommand(CHANNEL, 2, 0, 0);
}

void off()
{
  sendCommand(CHANNEL, 0, 0, 0);
}

void bind()
{
  sendCommand(CHANNEL, 15, 0, 0);
}

void unbind()
{
  sendCommand(CHANNEL, 9, 0, 0);
}

void sendCommand(byte channel, byte command, byte data, byte format)
{
  byte buf[12];

  for (byte i = 0; i < 12; i++) {
    buf[i] = 0;
  }

  buf[0] = 85;
  buf[1] = B01010000;
  buf[2] = command;
  buf[3] = format;
  buf[5] = channel;
  buf[6] = data;

  int checkSum = 0;
  for (byte i = 0; i < 10; i++) {
    checkSum += buf[i];
  }

  buf[10] = lowByte(checkSum);
  buf[11] = 170;

  for (byte i = 0; i < (12); i++)
  {
    mySerial.write(buf[i]);
  }
}

int readAnalogPin(int pin)
{
  int sum;
  sum = 0;
  for (int i = 0; i < 10; i++) {
    int val = analogRead(pin);
    sum += val;
  }

  return sum / 10;
}

int targetSensorValue = 0;
int currentSensorValue = 0;
unsigned long dimmerBusy = 0;

void updateDimmer()
{
  unsigned long time = millis();
  long isBusy = time - dimmerBusy;
  if (isBusy < 0)
    return;

  if (targetSensorValue != currentSensorValue)
  {
    sendCommand(CHANNEL, 6, targetSensorValue, 1);
    dimmerBusy = millis() + abs(targetSensorValue - currentSensorValue) * 8;
    currentSensorValue = targetSensorValue;
  }
}

float temp_sensorValue1 = 0;
float temp_sensorValue2 = 0;

void loop() {

  float sensorValue1 = (readAnalogPin(sensorPin1) / 1024.0 * 156); // 0..155
  float sensorValue2 = (readAnalogPin(sensorPin2) / 1024.0 * 156); // 0..155
    
  float delta1 = abs(temp_sensorValue1 - sensorValue1);
  if (delta1 >= 1.5) {
    temp_sensorValue1 = sensorValue1;
    targetSensorValue = temp_sensorValue1;
    //Serial.println(targetSensorValue);
  }
  
  float delta2 = abs(temp_sensorValue2 - sensorValue2);
  if (delta2 >= 1.5) {
    temp_sensorValue2 = sensorValue2;
    targetSensorValue = temp_sensorValue2;
    //Serial.println(targetSensorValue);
  }

  updateDimmer();
}

