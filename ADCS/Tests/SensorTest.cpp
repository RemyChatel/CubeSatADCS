#include "Test.h"
#include "SunSensor.h"
#include "MPU9150.h"

int SensorTest(Serial *pc, I2C *i2c, Timer *t){
    AnalogIn faceX(A0);
    AnalogIn faceY(A1);
    AnalogIn faceZ(A3);
    SunSensor sun(&faceX, &faceY, &faceZ);

    float rsun[3];

    while(1){
        sun.getSunVector(rsun);
        pc->printf("%f %f\n", rsun[0], rsun[1]);
        wait_ms(100);
    }

    return 1;
}