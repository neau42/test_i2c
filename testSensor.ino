#include <Wire.h>
#include <VL53L1X.h>

//Pin de contrôle d'extinction - pour changement d'adresse
#define PIN_XSHUT_1 PA4  //A4 - capteur haut
#define PIN_XSHUT_2 PA5  //A3 - capteur bas

//Futures adresses des capteurs [initialement 0x29]
#define ADR_SENSOR_1 0x42
#define ADR_SENSOR_2 0x24

//Futures adresses des capteurs [initialement 0x29]
#define SCL_Wire PA9
#define SDA_Wire PA10

VL53L1X sensor_up = VL53L1X();
VL53L1X sensor_down = VL53L1X();

void turn_off_sensors(){
  //sensor_up
  pinMode(PIN_XSHUT_1, OUTPUT);
  digitalWrite(PIN_XSHUT_1, LOW);
  //sensor_down
  pinMode(PIN_XSHUT_1, OUTPUT);
  digitalWrite(PIN_XSHUT_2, LOW);
}
void prepareSensor(byte pin, bool stat)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, stat);
}

/** Initialise a sensor and change its I2C address
 *  Input :   *sensor : a ptr of a sensor object 
 *            pin : The hardware pin connect to the sensor XSHUT
 *            adr : The new sensor address for I2C connection
**/
void initSensor(VL53L1X *sensor, byte pin, uint8_t adr)
{
  pinMode(pin, INPUT);
  delay(200);
  sensor->init(true);
  delay(100);
  sensor->setAddress(adr);
  sensor->setTimeout((uint16_t)500);
  sensor->setDistanceMode(VL53L1X::Medium);
  sensor->setMeasurementTimingBudget(33000);
  sensor->startContinuous(50);
}

void setup() {
  Serial.begin(9600);
  
  Wire.setSCL(SCL_Wire);
  Wire.setSDA(SDA_Wire);
  Wire.begin();

  turn_off_sensors();
  delay(100);
  prepareSensor(PIN_XSHUT_1, HIGH);
  initSensor(&sensor_up, PIN_XSHUT_1, ADR_SENSOR_1);
  prepareSensor(PIN_XSHUT_2, HIGH);  
  initSensor(&sensor_down, PIN_XSHUT_2, ADR_SENSOR_2);
}

void loop() {
  Serial.print("\n\nsensor_up:\t");
  Serial.print(sensor_up.read());
  if (sensor_up.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  Serial.print("\nsensor_down:\t");
  Serial.print(sensor_down.read());
  if (sensor_down.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  delay(200);
}
