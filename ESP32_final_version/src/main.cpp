#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/dac.h>

#define durationSleep 60
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually a = 3.488*10(^-3))
#define ACOEFFICIENT 3.448e-3
// the value of the 'other' resistor
#define HEATERRESISTOR 10000
#define SERIESRESISTOR 1000

void Compute();
void sendBle();
void startBle();

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
const int threshold = 50;
bool flag = false;
float txValue = 0;
const int NaOH_1 = 24;
double sensNaOH_1;
const int NaOH_2 = 8;
double sensNaOH_2;
const int SPUTUM = 16;
double sensSPUTUM;
const int PUMP = 6;
const int PIEZO = 23;
double sensB_filter;
const int B_filter = 26;
double sensA_filter;
const int A_filter = 13;
const int heat_sens = 12;
double Input, Setpoint;
double ITerm, lastInput;
int DAC_voltage = 232;
int counter = -1;

const int LED = 2; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

double Temperature(int RawADC)
{
  // convert the value to resistance
  double current = (DAC_voltage / 4095.0) * 3.3;
  current /= SERIESRESISTOR;
  // convert resistance to temperature
  double temperature = (RawADC / 4095.0) * 3.3;
  temperature /= current;
  temperature /= HEATERRESISTOR;
  temperature -= 1;
  temperature /= ACOEFFICIENT;
  temperature += (TEMPERATURENOMINAL + 273.15);
  temperature -= 273.15; // convert to C
  return temperature;
}

void Compute()
{
  double kp = 15, ki = 3, kd = 0;
  double outMax = 255, outMin = 0;
  int SampleTime = 1000; //1 sec
  unsigned long lastTime;
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
    DAC_voltage = kp * error + ITerm - kd * dInput;
    if (DAC_voltage > outMax)
      DAC_voltage = outMax;
    else if (DAC_voltage < outMin)
      DAC_voltage = outMin;
    //Remember some variables for next time
    lastInput = Input;
    lastTime = now;
  }
}

void sendBle()
{
  if (deviceConnected)
  {
    std::string value = pCharacteristic->getValue();
    switch (counter)
    {
    case 0:
    {
      if (flag)
      {
        value = "NaOH1";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      else
      {
        value = "NaOH1 failed";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 1:
    {
      if (flag)
      {
        value = "NaOH2";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      else
      {
        value = "NaOH2 failed";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 2:
    {
      if (flag)
      {
        value = "Sputum";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      else
      {
        value = "Sputum failed";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 3:
    {
      if (flag)
      {
        value = "Pump";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 4:
    {
      if (flag)
      {
        value = "Piezo";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 5:
    {
      if (flag)
      {
        value = "Fluidification";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      else
      {
        value = "Fluidification failed";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 6:
    {
      if (flag)
      {
        value = "Filtre";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      else
      {
        value = "Filtre failed";
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        Serial.println(counter);
        delay(50);
      }
      break;
    }

    case 7:
    {
      value = "Heater";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      Serial.println(counter);
      delay(50);
      break;
    }

    case 8:
    {
      txValue = Input;

      char txString[8];
      dtostrf(txValue, 1, 2, txString);
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      Serial.println(counter);
      delay(50);
      break;
    }

    case 9:
    {
      value = "Done";
      pCharacteristic->setValue(value);
      pCharacteristic->notify();
      Serial.println(counter);
      delay(50);
      break;
    }

    default:
      break;
    }
  }
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    digitalWrite(LED, HIGH);
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    digitalWrite(LED, LOW);
    esp_sleep_enable_timer_wakeup(durationSleep * 1000000);
    esp_deep_sleep_start();
  };
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
      if (rxValue.find("A1") != -1)
      {
        counter = 0;
      }
      else if (rxValue.find("A2") != -1)
      {
        counter = 1;
      }
      else if (rxValue.find("B") != -1)
      {
        counter = 2;
      }
      else if (rxValue.find("C") != -1)
      {
        counter = 3;
      }
      else if (rxValue.find("D") != -1)
      {
        counter = 5;
      }
      else if (rxValue.find("E") != -1)
      {
        counter = 6;
      }
      else if (rxValue.find("F") != -1)
      {
        counter = 7;
      }
      else if (rxValue.find("G") != -1)
      {
        counter = 9;
      }
    }
    Serial.println();
    Serial.println("*********");
  }
};

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  startBle();

  int _try = 0;
  while (!deviceConnected)
  {
    if (_try >= 5)
    {
      esp_sleep_enable_timer_wakeup(durationSleep * 1000000);
      esp_deep_sleep_start();
    }
    _try++;
    delay(500);
  }
}

void loop()
{
  if (deviceConnected)
  {
    switch (counter)
    {
    case 0:
    {
      Serial.print("Sensing NaOH1");
      sensNaOH_1 = analogRead(NaOH_1);
      if (sensNaOH_1 > threshold)
      {
        Serial.println("NaOH1 available");
        flag = true;
        sendBle();
      }
      else
      {
        flag = false;
        sendBle();
      }
      counter = -1;
      break;
    }

    case 1:
    {
      Serial.print("Sensing NaOH2");
      sensNaOH_1 = analogRead(NaOH_1);
      if (sensNaOH_1 > threshold)
      {
        Serial.println("NaOH2 available");
        flag = true;
        sendBle();
      }
      else
      {
        flag = false;
        sendBle();
      }
      counter = -1;
      break;
    }

    case 2:
    {
      Serial.print("Sensing Sputum");
      sensSPUTUM = analogRead(SPUTUM);
      if (sensSPUTUM > threshold)
      {
        Serial.println("Sputum available");
        flag = true;
        sendBle();
      }
      else
      {
        flag = false;
        sendBle();
      }
      counter = -1;
      break;
    }

    case 3:
    {
      Serial.print("Starting Pump");
      sensA_filter = analogRead(A_filter);
      unsigned long t_1 = millis();
      unsigned long t_0 = 0;
      int time_spent = (t_1 - t_0);
      counter = 3;
      flag = true;
      sendBle();
      while (sensA_filter < threshold || time_spent < 180001)
      {
        ledcSetup(0, 1000, 8);
        ledcAttachPin(PUMP, 0);
        ledcWrite(PUMP, 255);
        t_0 = t_1;
        t_1 = millis();
        time_spent = (t_1 - t_0);
        if (time_spent > 90000)
        {
          Serial.print("Starting ¨Piezo");
          ledcSetup(0, 5000, 8);
          ledcAttachPin(PIEZO, 0);
          ledcWrite(PIEZO, 255);
          flag = true;
          counter = 4;
          sendBle();
        }
      }
      ledcWrite(PUMP, 0);
      ledcWrite(PIEZO, 0);
      counter = -1;
      break;
    }

    case 4:
    {
      delay(1);
      break;
    }

    case 5:
    {
      Serial.print("Sensing A filter");
      sensA_filter = analogRead(sensA_filter);
      if (sensA_filter > threshold)
      {
        Serial.println("A filter reached");
        flag = true;
        sendBle();
      }
      else
      {
        flag = false;
        sendBle();
      }
      counter = -1;
      break;
    }

    case 6:
    {
      Serial.print("Sensing B filter");
      sensB_filter = analogRead(B_filter);
      if (sensB_filter > threshold)
      {
        Serial.println("B filter reached");
        flag = true;
        sendBle();
      }
      else
      {
        flag = false;
        sendBle();
      }
      counter = -1;
      break;
    }

    case 7:
    {
      Serial.print("Starting heater");
      sensB_filter = analogRead(B_filter);
      if (sensB_filter > threshold)
      {
        counter = 7;
        flag = true;
        sendBle();
        Setpoint = 100;
        while (true)
        {
          unsigned long t0;
          unsigned long t1 = millis();
          int duration = (t1 - t0);
          while (duration < 180001)
          {
            int temperature = analogRead(heat_sens);
            Input = Temperature(temperature);
            counter = 8;
            sendBle();
            Compute();
            dac_output_enable(DAC_GPIO26_CHANNEL);
            dac_output_voltage(DAC_GPIO26_CHANNEL, DAC_voltage);
            t0 = t1;
            t1 = millis();
            duration = (t1 - t0);
          }
          break;
        }
      }
      counter = -1;
      break;
    }

    case 8:
    {
      delay(10);
      break;
    }

    case 9:
    {
      Serial.print("Done");
      counter = 9;
      sendBle();
      counter = -1;
      break;
    }

    default:
      delay(50);
      break;
    }
  }
  else
  {
    int _try = 0;
    while (!deviceConnected)
    {
      if (_try >= 5)
      {
        esp_sleep_enable_timer_wakeup(durationSleep * 1000000);
        esp_deep_sleep_start();
      }
      _try++;
      delay(500);
    }
  }
}

void startBle()
{
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