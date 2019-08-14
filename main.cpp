#include "mbed.h"

// Change here the test you want to run
#define TEST_ADS

// Remove the ".test" part of the include if the test functions are not required
#ifdef TEST_MATRIX
    #include "Matrix.test.h"
#endif
#ifdef TEST_ASTROLIB
    #include "AstroLib.test.h"
#endif
#ifdef TEST_QUEST
    #include "Estimators.test.h"
#endif
#ifdef TEST_FILTER
    #include "Filters.test.h"
#endif
#ifdef TEST_IMU
    #include "MPU9150.test.h"
#endif
#ifdef TEST_SUNSENSOR
    #include "SunSensor.test.h"
#endif
#ifdef TEST_ADS
    #include "ADSCore.test.h"
#endif

Serial pc(USBTX, USBRX, 115200);

int main(){
    #ifdef TEST_MATRIX
        return MatrixTest();
    #endif
    #ifdef TEST_ASTROLIB
        return AstroLibTest();
    #endif
    #ifdef TEST_QUEST
        return QuestTest();
    #endif
    #ifdef TEST_FILTER
        return KalmanFilterTest();
    #endif
    #ifdef TEST_SUNSENSOR
        return SunSensorTest();
    #endif
    #ifdef TEST_IMU
        return MPU9150Test();
    #endif
    #ifdef TEST_ADS
        return ADSTest();
    #endif
}