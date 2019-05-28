#ifndef MPU9150_H
#define MPU9150_H
#include "MPU9150_registrer.h"
#include "mbed.h"

// Using the GY-9150 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0
#if ADO
    #define MPU9150_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
    #define MPU9150_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif  

enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};
enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

class MPU9150 {
public:
    MPU9150(PinName sda, PinName scl);
    MPU9150(I2C *i2c);
        
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    char readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

    void getGres();
    void setGres(uint8_t scale);
    void getAres();
    void setAres(uint8_t scale);

    void readAccelData(int16_t * destination);
    void readGyroData(int16_t * destination);
    void readMagData(int16_t * destination);
    int16_t readTempData();
    void getAccel(float acc[3]);
    void getGyro(float gyr[3]);
    void getMag(float mag[3]);
    float getTemp();

    uint8_t initIMU(uint8_t acc_scale, uint8_t gyr_scale);
    void initAK8975A();
    void initMPU9150();
    void resetMPU9150();
    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrateMPU9150();
    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    void MPU9150SelfTest();

    // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    // but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    void MadgwickQuaternionUpdate(float quat[4], float acc[3], float gyr[3], float mag[3], float dt);
    // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
    // measured ones. 
    void MahonyQuaternionUpdate(float quat[4],float acc[3], float gyr[3], float mag[3], float dt);

private:
    I2C *i2c_;
    // Set initial input parameters
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    float aRes_, gRes_, mRes_;      // scale resolutions per LSB for the sensors
    float magCalibration[3] = {0, 0, 0}; // Factory mag calibration
    float magBias[3] = {0, 0, 0};        // Factory mag bias
    float gyroBias[3] = {0, 0, 0}; // Bias corrections for gyro
    float accelBias[3] = {0, 0, 0}; // Bias corrections for accelerometer
    float SelfTest_[6];
    float pi = 3.14159265358979323846f;
}; // class MPU9150
#endif // MPU9150_H