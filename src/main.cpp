#include <mbed.h>
#include <SimpleFOC.h>
#include <mbed_config.h>
using namespace mbed;


BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(8, 11, 9, 12, 10, 13);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

MbedSPI spi(4,3,2);
MbedI2C i2c(16,17);

float sensor_offset;
void setup() {
    sensor.init(&spi);
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 12;
    driver.voltage_limit = 4;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_sensor_align = 1;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;

    Serial.begin(115200);
    motor.useMonitoring(Serial);

    gpio_pull_up(16);
    gpio_pull_up(17);
    i2c.setClock(400e3);
    i2c.begin();

    motor.target = 0;
    motor.init();
    motor.initFOC();    // TODO : Skip Align

    Serial.println(F("Motor ready"));

    _delay(1000);

    sensor_offset = sensor.getAngle();
}

unsigned long lastLoop, lastCmd;
void loop() {
    // Max 100Hz Command Loop
    if (millis() - lastLoop > 10) {
        // receive command
        float cmd[4];
        if (Serial.available()) lastCmd = millis();
        while (Serial.available()) 
        {
            if (Serial.read() == 0x02) Serial.readBytesUntil(0x04,(uint8_t*)&cmd,sizeof(cmd));
        }
        while (Serial.available())
        {
            Serial.read();
        }

        // i2c communication with slaves
        float ang[4];
        for (int i = 1; i <= 3; i++)
        {
            i2c.beginTransmission(i);
            i2c.write((uint8_t*)&cmd[i],4);
            i2c.endTransmission(true);

            if (i2c.requestFrom(0x01,6))
            {
                while (i2c.available())
                {
                    if (i2c.read() == 0x02)
                    {
                        i2c.readBytesUntil(0x04,(uint8_t*)&ang[i],4);
                    }
                }

                while (i2c.available()) i2c.read(); // clear buffer
            }
        }
        ang[0] = (sensor.getAngle() - sensor_offset) / PI * 180 / 5;

        // send data
        uint8_t buf[18];
        buf[0] = 0x02;
        buf[17] = 0x04;
        memcpy(buf+1,&ang,16);
        Serial.write(buf, 18);

        lastLoop = millis();
    }

    if (millis() - lastCmd) motor.target = 0;

    // Motor Loop
    motor.loopFOC();
    motor.move();
}