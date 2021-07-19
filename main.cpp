#include <Arduino.h>

// void setup() {
//   Serial.begin(115200);
//   delay(1000); // give me time to bring up serial monitor
//   Serial.println("ESP32 Touch Test");
// }

// void loop() {
//   Serial.println(touchRead(4));  // get value of Touch 0 pin = GPIO 4
//   delay(1000);
// }

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
// const int potPin = 34;
// double temp = 0;

// // variable for storing the potentiometer value
// int potValue = 0;

// void setup()
// {
//   Serial.begin(115200);
//   delay(1000);
// }

// void loop()
// {
//   // Reading potentiometer value
//   potValue = analogRead(potPin);
//   temp = 3.3 * (potValue / 4095.0);
//   Serial.println(temp);
//   delay(500);
// }

// const int ledPin = 16; // 16 corresponds to GPIO16

// // setting PWM properties
// const int freq = 5000;
// const int ledChannel = 0;
// const int resolution = 8;

// void setup()
// {
//   // configure LED PWM functionalitites
//   ledcSetup(ledChannel, freq, resolution);

//   // attach the channel to the GPIO to be controlled
//   ledcAttachPin(ledPin, ledChannel);
// }

// void loop()
// {
//   // increase the LED brightness
//   for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++)
//   {
//     // changing the LED brightness with PWM
//     ledcWrite(ledChannel, dutyCycle);
//     delay(15);
//   }

//   // decrease the LED brightness
//   for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--)
//   {
//     // changing the LED brightness with PWM
//     ledcWrite(ledChannel, dutyCycle);
//     delay(15);
//   }
// }

/*
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void setup()
{
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  // notify changed value
  if (deviceConnected)
  {
    pCharacteristic->setValue((uint8_t *)&value, 4);
    pCharacteristic->notify();
    value++;
    delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}
*/
/*
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
int counter = 0;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class writeCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);

      Serial.println();
      Serial.println("*********");
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void setup()
{
  Serial.begin(115200);

  Serial.println("1- Download and install an BLE scanner app in your phone");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to MyESP32");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");
  Serial.println("5- See the magic =)");

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->setCallbacks(new writeCallbacks());
  pServer->setCallbacks(new MyServerCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  if (deviceConnected)
  {
    std::string value = pCharacteristic->getValue();
    //int temp = stoi(value);
    if (counter == 0)
    {
      value = "NaOH";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    if (counter == 1)
    {
      value = "Sample";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    if (counter == 2)
    {
      value = "Pump";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    if (counter == 3)
    {
      value = "Piezo";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    if (counter == 4)
    {
      value = "Fluidification";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    if (counter == 5)
    {
      value = "Filtre";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    if (counter == 6)
    {
      value = "Heater";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      counter++;
      Serial.println(counter);
      delay(3);
    }
    //Serial.println(value);
    // pCharacteristic->setValue((uint8_t*)&value, 4);
    //int status = value.toInt();
    //String doblado = (String)doble; // Lo convierte en String.

    //pCharacteristic->setValue(doblado.c_str()); // Pone el numero doble
    //pCharacteristic->notify();
    //delay(5); // bluetooth stack will go into congestion, if too many packets are sent.
  }
  // put your main code here, to run repeatedly:
  //delay(2000);
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Lost connection, start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    //String value = "connected";
    //pCharacteristic->setValue("connected");
    //pCharacteristic->notify();
    delay(3);
  }
}
*/
/* 
#include <driver/dac.h>
//Define Variables we'll be connecting to
//#define THERMISTORNOMINAL 100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually a = 3.488*10(^-3))
#define ACOEFFICIENT 3.448e-3
// the value of the 'other' resistor
#define HEATERRESISTOR 10000
#define SERIESRESISTOR 1000

double Input, Output, Setpoint;
double ITerm, lastInput;
unsigned long lastTime;
int DAC_voltage = 232;

void Compute()
{
  double kp = 15, ki = 3, kd = 0;
  double outMax = 255, outMin = 0;
  int SampleTime = 1000; //1 sec

  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    //Compute all the working error variables
   
    double error = Setpoint - Input;
    ITerm += (ki * error);
    if (ITerm > outMax)
      ITerm = outMax;
    else if (ITerm < outMin)
      ITerm = outMin;
    double dInput = (Input - lastInput);

    //Compute PID Output
    Output = kp * error + ITerm - kd * dInput;
    if (Output > outMax)
      Output = outMax;
    else if (Output < outMin)
      Output = outMin;
    //Remember some variables for next time
    lastInput = Input;
    lastTime = now;
  }
}

double Temperature(int RawADC, int DAC_voltage)
{
  // convert the value to resistance
  double current = (DAC_voltage / 4095.0) * 3.3;
  current /= SERIESRESISTOR;
  //double steinhart = (1023.0/RawADC) - 1;
  double temperature = (RawADC / 4095.0) * 3.3;
  temperature /= current;
  temperature /= HEATERRESISTOR;
  temperature -= 1;
  temperature /= ACOEFFICIENT;
  temperature += (TEMPERATURENOMINAL + 273.15);
  temperature -= 273.15;
  //  steinhart = SERIESRESISTOR / steinhart;
  steinhart = steinhart / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;    // convert to C
  //return temperature;
}

void setup()
{
  Serial.begin(115200);
  //initialize the variables we're linked to
  Setpoint = 100;
  //DAC_voltage = 232;
  dac_output_enable(DAC_GPIO26_CHANNEL);
  dac_output_voltage(DAC_GPIO26_CHANNEL, DAC_voltage);
}

void loop()
{
  //Read the temperature for the PID
  Input = Temperature(analogRead(27), DAC_voltage);
  //Perform the PID control functionn
  Compute();
  //Write the output to the PWM
  dac_output_enable(DAC_GPIO25_CHANNEL);
  dac_output_voltage(DAC_GPIO26_CHANNEL, Output);
  Serial.print("Temperature: ");
  Serial.println(Input);
  delay(10);
  //The program is finished after 3 min
  if (millis() >= 180001)
  {
    while (1)
    {
      Serial.println("Done");
      delay(5000);
    }
  }
} 
*/
/*
#include "WiFi.h"

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);

  Serial.println(WiFi.macAddress());
}

void loop() {}
*/
/*
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);

      Serial.println();
      Serial.println("*********");
    }
  }
};

void setup()
{
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_READ);

  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  if (deviceConnected)
  {
    String datastring = "ESP32: ";
    datastring += txValue;
    Serial.printf("*** Sent Value: %d ***\n", txValue);
    pCharacteristic->setValue(datastring.c_str());
    pCharacteristic->notify();
    txValue++;
  }os
  delay(1000);
}
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool flag = false;
float txValue = 0;
const int readPin = 32; // Use GPIO number. See ESP32 board pinouts
const int LED = 2;      // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

//std::string rxValue; // Could also make this a global var to access it in loop()

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    digitalWrite(LED, LOW);
    ESP.restart();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {
      Serial.println("*********");
      Serial.print("Received Value: ");

      for (int i = 0; i < rxValue.length(); i++)
      {
        Serial.print(rxValue[i]);
      }

      Serial.println();

      // Do stuff based on the command received from the app
      if (rxValue.find("A") != -1)
      {
        Serial.print("Turning ON!");
        digitalWrite(LED, HIGH);
        flag = true;
      }
      else if (rxValue.find("B") != -1)
      {
        Serial.print("Turning OFF!");
        digitalWrite(LED, LOW);
      }

      Serial.println();
      Serial.println("*********");
    }
  }
};

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  // Create the BLE Device
  BLEDevice::init("myESP32"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  if (deviceConnected)
  {
    // Fabricate some arbitrary junk for now...
    txValue = analogRead(readPin) / 3.456; // This could be an actual sensor reading!

    // Let's convert the value to a char array:
    char txString[8];                 // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer

    if (flag)
    {
      pCharacteristic->setValue(txString);
      pCharacteristic->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
  }
  delay(1000);
}
