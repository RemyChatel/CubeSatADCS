#include "mbed.h"
#include "Tests/Test.h"
#define TEST_MATRIX
Serial pc(USBTX, USBRX, 115200);
I2C i2c(I2C_SDA, I2C_SCL);
Timer t;

int main(){
    #ifdef ADCS_CORES
        return 1;
    #endif
    #ifdef TEST_SIMPLEMATRIX
        return SimpleMatrixTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_MATRIX
        return MatrixTest(&pc, &i2c, &t);
    #endif
    #ifdef TEST_MATRIXRERUN
        return MatrixTestRerun(&pc, &i2c, &t);
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
    #ifdef TEST_SUNSENSOR
        return SunSensorTest(&pc, &i2c, &t);
    #endif
}