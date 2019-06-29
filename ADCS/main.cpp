#include "mbed.h"
#include "Test.h"
#define TEST_ADS
Serial pc(USBTX, USBRX, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
Timer t;

int main(){
    i2c.frequency(400000);
    t.start();
    #ifdef ADCS_CORES
        return 1;
    #endif
    #ifdef TEST_MATRIX
        return MatrixTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_ASTROLIB
        return AstroLibTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_QUEST
        return QuestTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_FILTER
        return FilterTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_MPU9250
        return MPU9250Test(&pc, &i2c, &t);
    #endif
    #ifdef TEST_SENSOR
        return SensorTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_ADS
        return ADSTest(&pc, &i2c, &t);
    #endif
}