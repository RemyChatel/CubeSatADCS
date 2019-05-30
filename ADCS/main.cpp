#include "mbed.h"
#include "Tests/Test.h"
#define TEST_SIMPLEMATRIX

Serial pc(USBTX, USBRX, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
Timer t;

int main(){
    #ifdef TEST_SIMPLEMATRIX
        return SimpleMatrixTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_ASTROLIB
        return AstroLibTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_QUEST
        return QuestTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_MPU9250
        return MPU9250Test(&pc, &i2c, &t);
    #endif
}