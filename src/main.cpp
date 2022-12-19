#include <mbed.h>
#include <SimpleFOC.h>
#include <mbed_config.h>
#include <Wire.h>

/*
* I2C Address
*
* E = 0x01
* X = 0x02
* Y = 0x03
* Z = MASTER : Comment out #define SLAVE_ADDR
*/

// #define SLAVE_ADDR 0x01

#define GEARBOX_RATIO (47.666)

using namespace mbed;

BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(8, 11, 9, 12, 10, 13);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

MbedSPI spi(4,3,2);
MbedI2C i2c(0,1);

unsigned long last;
uint8_t buf[4];
#ifdef SLAVE_ADDR
void rcv(int n) {
  Serial.println(n);
  if (n != 6) return;
  // if (Wire.read() != 0x02) return;

  // float val = 0;
  // Wire.readBytes(buf,n);

  // memcpy(&val, buf, 4);
  // Serial.println(val);

  // if (!motor.enabled) motor.enable();
  // motor.target = val;
  // last = millis();

  // while(Wire.available()) {
  for(int i = 0; i < n; i++) {
    Serial.print(Wire.read(), HEX);
    Serial.print(',');
  }
  // };
  Serial.print('\n');
}

// void req() {
//   sensor.update();
//   float val = sensor.getAngle();
//   byte buf[4];
//   memcpy(buf, &val, 4);
//   Wire.write(buf,4);
// }
#endif

void setup() {
  Serial.begin(115200);

  #ifdef SLAVE_ADDR
  i2c.begin(SLAVE_ADDR);
  i2c.onReceive(rcv);
  // i2c.onRequest(req);
  #else
  i2c.begin();
  #endif

  sensor.init(&spi);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.dead_zone = 0;
  driver.init();

  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.voltage_limit = 12;
  motor.voltage_sensor_align = 1;

  motor.init();
  /*
  * Zero Elec
  * E     0.56
  * X     0.79
  * Y     0
  * Z     0
  */
  motor.initFOC();

  // Serial.println(motor.zero_electric_angle);

  motor.target = 0;
  motor.disable();
}

void loop() {
  motor.loopFOC();
  motor.move();

  if (motor.enabled && millis() - last > 100) motor.disable();

  #ifndef SLAVE_ADDR
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    while (Serial.available()) Serial.read();

    Serial.print("I heard : ");
    Serial.println(str);
    
    float val = atof(str.c_str());
    // float val = 0.12345;
    memcpy(buf, &val, 4);
    i2c.beginTransmission(0x01);
    i2c.write(0x02);
    i2c.write(buf,4);
    i2c.write(0x04);
    i2c.endTransmission();
  }
  #endif
}