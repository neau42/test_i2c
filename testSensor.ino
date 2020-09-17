#include <Wire.h>
#include <VL53L1X.h>

#define PIN_XSHUT_1 PA4  //A4
#define PIN_XSHUT_2 PA5   //A3

#define ADR_SENSOR_1 0x42
//#define ADR_SENSOR_2 0x43


VL53L1X sensor1 = VL53L1X();
VL53L1X sensor2 = VL53L1X();

void prepareSensor(byte pin, bool stat)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, stat);
}

void initSensor(VL53L1X *sensor, byte pin, uint8_t adr)
{
  pinMode(pin, INPUT);
  delay(200);
  sensor->init(true);
  delay(100);
  sensor->setAddress(adr);
  sensor->setTimeout((uint16_t)500);
}

void printAddr()
{
    
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
//    else if (error == 4) {
//      Serial.print("Unknown error at address 0x");
//      if (address < 16) 
//        Serial.print("0");
//      Serial.println(address, HEX);
//    }    
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
    while (1);
  }
  else
    Serial.println("done");
}

void setup() {

  Serial.begin(9600);
  Wire.setSCL(PA9);
  Wire.setSDA(PA10);
  
  Wire.begin();

//  Serial.println("\nshutdown 2 sensors");
  prepareSensor(PIN_XSHUT_1, LOW);
  prepareSensor(PIN_XSHUT_2, LOW);
  delay(100);
//  Serial.println("wake first sensor");
  prepareSensor(PIN_XSHUT_1, HIGH);
//  Serial.print("set address sensor1: ");
//  Serial.println(ADR_SENSOR_1, HEX);
  initSensor(&sensor1, PIN_XSHUT_1, ADR_SENSOR_1);

//  Serial.println("wake second sensor");
  prepareSensor(PIN_XSHUT_2, HIGH);
//  initSensor(&sensor2, PIN_XSHUT_2, ADR_SENSOR_2);
//
  sensor1.setDistanceMode(VL53L1X::Medium);
  sensor1.setMeasurementTimingBudget(33000);
  sensor1.startContinuous(50);
//
//
  sensor2.setDistanceMode(VL53L1X::Medium);
  sensor2.setMeasurementTimingBudget(33000);
//  sensor2.startContinuous(50);

    printAddr();
}

void printDetails(VL53L1X *sensor)
{
  sensor->read();
  Serial.print("range: ");
  Serial.print(sensor->ranging_data.range_mm);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor->ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor->ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(sensor->ranging_data.ambient_count_rate_MCPS);
  }

void loop() {

//  sensor1.startContinuous(50);
  Serial.print("\n\nSensor1:\t");
  
  Serial.print(sensor1.read());
  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
//   printDetails(&sensor1);

//  sensor1.stopContinuous();
//
//  sensor2.startContinuous(50);
//  Serial.print("\nSensor2:\t");
//  printDetails(&sensor1);
//
////  Serial.print(sensor2.read());
////  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
////  Serial.println();
//
//  sensor2.stopContinuous();

  delay(500);
}
