#include <Arduino.h>
/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini, with some additional code by pcbreflux
   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"
   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.
   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescriptor;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool deviceNotifying = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

//origineel
//#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
//#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
//#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define SERVICE_UUID           "ab8dd464-ca0e-46bd-a1eb-9d9f8080e950" // UART service UUID
#define CHARACTERISTIC_UUID_RX "ab8dd465-ca0e-46bd-a1eb-9d9f8080e950"
#define CHARACTERISTIC_UUID_TX "ab8dd466-ca0e-46bd-a1eb-9d9f8080e950"

/* Motorsturing
// * Example sketch to control a 28BYJ-48 stepper motor with ULN2003 driver board, AccelStepper and Arduino UNO: continuous rotation. More info: https://www.makerguides.com */
// Include the AccelStepper library:
#include <AccelStepper.h>

// Motor pin definitions Links
#define motorPinL1  26     // IN1 on the ULN2003 driver
#define motorPinL2  25     // IN2 on the ULN2003 driver
#define motorPinL3  33     // IN3 on the ULN2003 driver
#define motorPinL4  32     // IN4 on the ULN2003 driver
// Motor pin definitions Rechts
#define motorPinR1   5      // IN1 on the ULN2003 driver
#define motorPinR2  18     // IN2 on the ULN2003 driver
#define motorPinR3  19     // IN3 on the ULN2003 driver
#define motorPinR4  21     // IN4 on the ULN2003 driver
// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepperL = AccelStepper(MotorInterfaceType, motorPinL1, motorPinL3, motorPinL2, motorPinL4);
AccelStepper stepperR = AccelStepper(MotorInterfaceType, motorPinR1, motorPinR3, motorPinR2, motorPinR4);

int MaxSpeed = 1700;    // sneller kunnen de motortjes niet volgen
int Speed    = 1000;    // beginsnelheid
int SLinks;
int SRechts;
bool Noodstop = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        if (rxValue == "voor") {
          SLinks = +1 ;
          SRechts = -1;
        }
        if (rxValue == "achter") {
          SLinks = -1;
          SRechts = +1;
        }
        if (rxValue == "rechts") {
          SLinks = +1;
          SRechts = +1;
        }
        if (rxValue=="links") {
          SLinks = -1;
          SRechts = -1;
        }
        if (rxValue=="sneller") {
          Speed = min( Speed + 100, MaxSpeed );
          Serial.print(Speed);
        }
        if (rxValue=="trager") {
          Speed = max(Speed - 100, 0);
          Serial.print(Speed);
        }
        if (rxValue=="stop") {
          SLinks = 0;
          SRechts = 0;
          Noodstop = true;
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};


class MyDisCallbacks: public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor *pDescriptor) {
      uint8_t* rxValue = pDescriptor->getValue();

      if (pDescriptor->getLength() > 0) {
        if (rxValue[0]==1) {
          //deviceNotifying=true;
        } else {
          deviceNotifying=false;
        }
        Serial.println("*********");
        Serial.print("Received Descriptor Value: ");
        for (int i = 0; i < pDescriptor->getLength(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void Move(int SpeedL, int SpeedR) {
  // Set the speed of the motor in steps per second:
  stepperL.setSpeed(SpeedL);
  stepperR.setSpeed(SpeedR);
  // Step the motor with constant speed as set by setSpeed():
  stepperL.runSpeed();
  stepperR.runSpeed();
}

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("GVD");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pDescriptor = new BLE2902();
  pCharacteristic->addDescriptor(pDescriptor);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pDescriptor->setCallbacks(new MyDisCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  // Set the maximum steps per second:
  stepperL.setMaxSpeed(MaxSpeed);
  stepperR.setMaxSpeed(MaxSpeed);
  // Set Max acceleration
  stepperL.setAcceleration(MaxSpeed / 5); //Accelerate in 1sec to max
  stepperR.setAcceleration(MaxSpeed / 5);

}

void loop() {
  if (deviceConnected && deviceNotifying) {
    Serial.printf("*** Sent Value: %d ***\n", txValue);
    pCharacteristic->setValue(&txValue, 1);
    pCharacteristic->notify();
    txValue++;
    delay(100);
  }
  
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        Serial.println("Disconnected");
        oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        Serial.println("Connecting..");
       
    }
  if(!Noodstop) {
    Move(Speed * SLinks, Speed * SRechts);
  }
  else {
    stepperL.stop() ;
    stepperR.stop() ;
  }
}
