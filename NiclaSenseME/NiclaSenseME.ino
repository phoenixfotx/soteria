
  #include "Nicla_System.h"
  #include "Arduino_BHY2.h"
  #include <ArduinoBLE.h>
  #include <SPI.h>
  #include <SD.h>
  Sd2Card card;
  SdVolume volume;
  SdFile root;
  const int chipSelect = 6;
  #define BLE_SENSE_UUID(val) ("19b10000-" val "-537e-4f6c-d104768a1214")

  const int VERSION = 0x00000000;

  BLEService service(BLE_SENSE_UUID("0000"));

  BLEStringCharacteristic sendStringCharacteristic(BLE_SENSE_UUID("8002"), BLERead | BLENotify | BLEWrite, 20); // 20 is the maximum length
  BLEStringCharacteristic receiveStringCharacteristic(BLE_SENSE_UUID("8003"), BLERead | BLENotify | BLEWrite, 20);

  BLEUnsignedIntCharacteristic versionCharacteristic(BLE_SENSE_UUID("1001"), BLERead);
  BLEFloatCharacteristic temperatureCharacteristic(BLE_SENSE_UUID("2001"), BLERead);
  BLEUnsignedIntCharacteristic humidityCharacteristic(BLE_SENSE_UUID("3001"), BLERead);
  BLEFloatCharacteristic pressureCharacteristic(BLE_SENSE_UUID("4001"), BLERead);


  BLECharacteristic accelerometerCharacteristic(BLE_SENSE_UUID("5001"), BLERead | BLENotify, 3 * sizeof(float));  // Array of 3x 2 Bytes, XY
  BLECharacteristic gyroscopeCharacteristic(BLE_SENSE_UUID("6001"), BLERead | BLENotify, 3 * sizeof(float));    // Array of 3x 2 Bytes, XYZ
  BLECharacteristic quaternionCharacteristic(BLE_SENSE_UUID("7001"), BLERead | BLENotify, 4 * sizeof(float));     // Array of 4x 2 Bytes, XYZW
  BLECharacteristic forcepadCharacteristic(BLE_SENSE_UUID("9004"), BLERead | BLENotify,2 * sizeof(float));     // Array of 3x 2 Bytes, F1,F2

  BLECharacteristic rgbLedCharacteristic(BLE_SENSE_UUID("8001"), BLERead | BLEWrite, 3 * sizeof(byte)); // Array of 3 bytes, RGB

  BLEFloatCharacteristic bsecCharacteristic(BLE_SENSE_UUID("9001"), BLERead);
  BLEIntCharacteristic  co2Characteristic(BLE_SENSE_UUID("9002"), BLERead);
  BLEUnsignedIntCharacteristic gasCharacteristic(BLE_SENSE_UUID("9003"), BLERead);

  // String to calculate the local and device name
  String name;

  Sensor temperature(SENSOR_ID_TEMP);
  Sensor humidity(SENSOR_ID_HUM);
  Sensor pressure(SENSOR_ID_BARO);
  Sensor gas(SENSOR_ID_GAS);
  Sensor forcepad(1001);
  SensorXYZ gyroscope(SENSOR_ID_GYRO);
  SensorXYZ accelerometer(SENSOR_ID_ACC);
  SensorQuaternion quaternion(SENSOR_ID_RV);
  SensorBSEC bsec(SENSOR_ID_BSEC);

const int numReadings =10;

float readings[numReadings];  
int readIndex = 0;          
float total = 0;              
float average = 0;           

float readings1[numReadings];  
int readIndex1 = 0;          
float total1 = 0;              
float average1 = 0;            

int inputPin = A0;
int inputPin1 = A1;

void setup() {

  nicla::begin();
  nicla::leds.begin(); 
  pinMode(0, OUTPUT);
  Serial.begin(1000000);
  nicla::leds.setColor(green);
  delay(5000);
  digitalWrite(0,HIGH);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
    readings1[thisReading] = 0;

    //Serial.println("Start");
  }

    nicla::leds.begin();
    nicla::leds.setColor(green);

    //Sensors initialization
    BHY2.begin(NICLA_STANDALONE);
    temperature.begin();
    humidity.begin();
    pressure.begin();
    gyroscope.begin();
    accelerometer.begin();
    quaternion.begin();
    bsec.begin();
    gas.begin();

    if (!BLE.begin()){
      Serial.println("Failed to initialized BLE!");

      while (1)
        ;
    }
    delay(3000);
    nicla::leds.setColor(cyan);
    delay(2000);
    digitalWrite(0,LOW);
    String address = BLE.address();

    Serial.print("address = ");
    Serial.println(address);

    address.toUpperCase();

    name = "GAP1-";
    name += address[address.length() - 5];
    name += address[address.length() - 4];
    name += address[address.length() - 2];
    name += address[address.length() - 1];

    Serial.print("name = ");
    Serial.println(name);

    BLE.setLocalName(name.c_str());
    BLE.setDeviceName(name.c_str());
    BLE.setAdvertisedService(service);

    // Add all the previously defined Characteristics
    service.addCharacteristic(temperatureCharacteristic);
    service.addCharacteristic(humidityCharacteristic);
    service.addCharacteristic(pressureCharacteristic);
    service.addCharacteristic(versionCharacteristic);
    service.addCharacteristic(accelerometerCharacteristic);
    service.addCharacteristic(gyroscopeCharacteristic);
    service.addCharacteristic(quaternionCharacteristic);
    service.addCharacteristic(bsecCharacteristic);
    service.addCharacteristic(co2Characteristic);
    service.addCharacteristic(gasCharacteristic);
    service.addCharacteristic(forcepadCharacteristic);
    service.addCharacteristic(rgbLedCharacteristic);
    service.addCharacteristic(sendStringCharacteristic);
    service.addCharacteristic(receiveStringCharacteristic);

    // Disconnect event handler
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // Sensors event handlers
    temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
    humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
    pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
    bsecCharacteristic.setEventHandler(BLERead, onBsecCharacteristicRead);
    co2Characteristic.setEventHandler(BLERead, onCo2CharacteristicRead);
    gasCharacteristic.setEventHandler(BLERead, onGasCharacteristicRead);
    sendStringCharacteristic.setEventHandler(BLEWritten, onSendStringCharacteristicWrite);
    receiveStringCharacteristic.setEventHandler(BLEWritten, onReceiveStringCharacteristicWrite);


    rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

    versionCharacteristic.setValue(VERSION);

    BLE.addService(service);
    BLE.advertise();
  
    

}
void loop() {
 
   while (BLE.connected()){
      BHY2.update();
        
        Serial.print(3);Serial.print(",");
        Serial.print(0);Serial.print(",");
        Serial.print(average,1);
        Serial.print(",");
        Serial.println(average1,1);

      if (sendStringCharacteristic.subscribed()) {
        // Here, you can use sendStringCharacteristic.writeValue() to send string data to the connected device.
        // Example:
        String sendData = "Hello, Bluetooth!";
        sendStringCharacteristic.writeValue(sendData);
       }

      if (gyroscopeCharacteristic.subscribed()){
        float x, y, z;

        x = gyroscope.x();
        y = gyroscope.y();
        z = gyroscope.z();

        float gyroscopeValues[3] = {x, y, z};

        gyroscopeCharacteristic.writeValue(gyroscopeValues, sizeof(gyroscopeValues));
      }

      if (accelerometerCharacteristic.subscribed()){
        float x, y, z;
        x = accelerometer.x();
        y = accelerometer.y();
        z = accelerometer.z();

        float accelerometerValues[] = {x, y, z};
        accelerometerCharacteristic.writeValue(accelerometerValues, sizeof(accelerometerValues));
      }

      if (forcepadCharacteristic.subscribed()){
        float F1, F2;
        float average = analogRead(A0)* (8.0 / 1023.0);
        float average1 = analogRead(A1)* (5.0 / 1023.0);
        F1 = average;
        F2 = average1;

        float forcepadValues[] = {F1,F2};
        forcepadCharacteristic.writeValue(forcepadValues, sizeof(forcepadValues));
      }

      if(quaternionCharacteristic.subscribed()){
        float x, y, z, w;
        x = quaternion.x();
        y = quaternion.y();
        z = quaternion.z();
        w = quaternion.w();

        float quaternionValues[] = {x,y,z,w};
        quaternionCharacteristic.writeValue(quaternionValues, sizeof(quaternionValues));
      }

    }
  }
  

  void onSendStringCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
    // Handle received string data
    String receivedData = sendStringCharacteristic.value();
    String sendData = "Hello World";
    sendStringCharacteristic.writeValue(sendData);

     // Add your code to process the received string data as needed
    }

  void onReceiveStringCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
    // Handle received string data
      String receivedData = receiveStringCharacteristic.value();
      Serial.println("Received String: " + receivedData);

    // Add your code to process the received string data as needed
    }

  void blePeripheralDisconnectHandler(BLEDevice central){
    nicla::leds.setColor(red);
  }

  void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
    float temperatureValue = temperature.value();
    temperatureCharacteristic.writeValue(temperatureValue);
  }

  void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
    uint8_t humidityValue = humidity.value() + 0.5f;  //since we are truncating the float type to a uint8_t, we want to round it
    humidityCharacteristic.writeValue(humidityValue);
  }

  void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
    float pressureValue = pressure.value();
    pressureCharacteristic.writeValue(pressureValue);
  }

  void onBsecCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
    float airQuality = float(bsec.iaq());
    bsecCharacteristic.writeValue(airQuality);
  }

  void onCo2CharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
    uint32_t co2 = bsec.co2_eq();
    co2Characteristic.writeValue(co2);
  }

  void onGasCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
    unsigned int g = gas.value();
    gasCharacteristic.writeValue(g);
  }

  void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic){
    byte r = rgbLedCharacteristic[0];
    byte g = rgbLedCharacteristic[1];
    byte b = rgbLedCharacteristic[2];

    nicla::leds.setColor(r, g, b);
}
