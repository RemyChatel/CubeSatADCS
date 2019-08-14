/**
 * @file   MPU9150.h
 * @defgroup MPUGr MPU9150 Driver
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A drivers for the MPU9150 Inertial Measurement Unit from InvenSense.
 * 
 * @details
 * # Description
 * A driver for the InvenSense4s MPU9150 Inertial Measurement Unit
 * 
 * Adapted from Kris Winer MPU9150AHRS library, 
 * https://os.mbed.com/users/onehorse/code/MPU9150AHRS/
 *
 * This module establishes the I2C communication with the MPU9150 IMU
 * in order to fetch the acceleration, the angular rates and the
 * measured Earth magnetic field in the body frame.
 * 
 * It first allows the initialization of the peripheral and performs
 * some calibration to limit the biases on the sensors. This calibration
 * step performs one-second sampling of all the sensors. Then the average
 * of those measurements is calculated and subtracted to the chosen sensors.
 * The chosen measurements will have their DC bias removed by this method.
 * 
 * Then, the measurements can be fetched using simple functions. The
 * module also features originally two attitude filters, however, they
 * are only suitable for UAV and not for space applications as they
 * rely on the gravity vector.
 * 
 * @see MPU9150
 * @see MPU9150_registers.h
 *
 * # Example code
 * 
 * @see MPU9150.test.cpp
 * 
 * # License
 * <b>(C) Copyright 2019 Remy CHATEL</b>
 * 
 * Licensed Under  GPL v3.0 License
 * http://www.gnu.org/licenses/gpl-3.0.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MPU9150_H
#define MPU9150_H
#include "MPU9150_registers.h"
#include "mbed.h"

// Using the GY-9150 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0 /**< I2C address selector for the IMU */
#if ADO
    #define MPU9150_ADDRESS 0x69<<1  /**< The address of the IMU when AD0 = 1 */
#else
    #define MPU9150_ADDRESS 0x68<<1  /**< The address of the IMU when AD0 = 0 */
#endif
#define Kp 2.0f * 5.0f  /**< Proportional gain for the optimisation */
#define Ki 0.0f         /**< Integral gain for the optimisation */

/**
 * @brief
 * Enumeration for the scale of the accelerometer
 * \enum Ascale
 */
enum Ascale {
    AFS_2G = 0, /**< Describe the 2g scale for the accelerometer */
    AFS_4G,     /**< Describe the 4g scale for the accelerometer */
    AFS_8G,     /**< Describe the 8g scale for the accelerometer */
    AFS_16G     /**< Describe the 16g scale for the accelerometer */
};
/**
 * @brief
 * Enumeration for the scale of the gyroscope
 * /enum Gscale
 */
enum Gscale {
    GFS_250DPS = 0, /**< Describe the 250 deg/s scale for the gyroscope */
    GFS_500DPS,     /**< Describe the 500 deg/s scale for the gyroscope */
    GFS_1000DPS,    /**< Describe the 1000 deg/s scale for the gyroscope */
    GFS_2000DPS     /**< Describe the 2000 deg/s scale for the gyroscope */
};

/**
 * @ingroup MPUGr
 * @brief
 * A drivers for the MPU9150 Inertial Measurement Unit
 * 
 * @class MPU9150
 * 
 * @details
 * # Description
 * The MPU9150 is an Inertial Measurement Unit from InvenSense that provides
 * 9 axis (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer)
 * 
 * @see MPU9150.h
 * @see MPU9150_registers.h
 * 
 * # Dependencies
 * This library was built around the "Mbed" framework to access the harware
 * through an common interface regardless of the device as long as the device
 * is supported by Mbed (https://www.mbed.com/en/)
 */
class MPU9150 {
public:
///@name Constructors
    /**
     * @brief
     * Constructor for MPU9150 that creates its own I2C instance
     * @param sda The SDA pin for the I2C communication
     * @param scl The SCL pin for the I2C communication
     */
    MPU9150(PinName sda, PinName scl);
    
    /**
     * @brief
     * Constructor for MPU9150 that uses a predefined I2C instance
     * @param i2c A reference to the I2C object
     */
    MPU9150(I2C *i2c);

///@name I2C Tools
    
    /**
     * @brief
     * Write a byte on the I2C bus
     * @param address The address of the target
     * @param subAddress The target register
     * @param data The byte to write
     */
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    
    /**
     * @brief
     * Read a byte from the I2C bus
     * @param address The address of the target
     * @param subAddress The register to read
     * @return The read byte
     */
    char readByte(uint8_t address, uint8_t subAddress);
    
    /**
     * @brief
     * Read multiple bytes from the I2C bus
     * @param address The address of the target
     * @param subAddress The registers to read
     * @param count The number of bytes to read
     * @param dest The array where to store the bytes
     */
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

///@name IMU Set up
    /**
     * @brief
     * Get the resolution of the gyroscope and store it
     */
    void getGres();
    
    /**
     * @brief
     * Set the scaling of the gyroscope
     * @param scale The scale to set from the enumeration GScale
     */
    void setGres(uint8_t scale);
    
    /**
     * @brief
     * Get the resolution of the accelerometer and store it
     */
    void getAres();
    
    /**
     * @brief
     * Set the scaling of the accelerometer
     * @param scale The scale to set from the enumeration AScale
     */
    void setAres(uint8_t scale);

    
    /**
     * @brief
     * Initialize the IMU (acc, gyr and mag)
     * @param acc_scale The scale factor of the accelerometer form the enumeration AScale
     * @param gyr_scale The scale factor of the gyroscope form the enumeration GScale
     * @return The address of the IMU if found, zero otherwise
     */
    uint8_t initIMU(uint8_t acc_scale, uint8_t gyr_scale);
    
    /**
     * @brief
     * Initialize the magnetometer
     */
    void initAK8975A();
    
    /**
     * @brief
     * Initialize the accelerometer and the gyroscope
     * @param acc_scale The scale factor of the accelerometer form the enumeration AScale
     * @param gyr_scale The scale factor of the gyroscope form the enumeration GScale
     */
    void initMPU9150(uint8_t acc_scale, uint8_t gyr_scale);
    
    /**
     * @brief
     * Reset the MPU9150 to factory settings
     */
    void resetMPU9150();
    
    /**
     * @brief
     * Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
     * of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
     */
    // 
    void calibrateMPU9150();
    
    /**
     * @brief
     * Accelerometer and gyroscope self test; check calibration wrt factory settings
     */
    void MPU9150SelfTest();

    /**
     * @brief
     * Method to recalibrate the IMU by removing the DC part
     * @param time The total sampling time (in ms) to detect DC
     * @param N The number of sample to take
     */
    void recalibrateIMU(float time, int N);

    /**
     * @brief
     * Allow the user to manually set the avg bias for the acceleration
     * @param new_avg_acc The acceleration bias (m/s)
     */
    void setAvgAcc(float new_avg_acc[3]);
    
    /**
     * @brief
     * Allow the user to manually set the avg bias for the angular rate
     * @param new_avg_gyr The angular rate bias (deg/s)
     */
    void setAvgGyr(float new_avg_gyr[3]);
    
    /**
     * @brief
     * Allow the user to manually set the avg bias for the magnetic field
     * @param new_avg_mag The magnetic field bias (uT)
     */
    void setAvgMag(float new_avg_mag[3]);

///@name Read functions
    /**
     * @brief
     * Fetches the accelerometer count from the IMU
     * @param destination The array where to store the accelerometer count
     */
    void readAccelData(int16_t * destination);
    
    /**
     * @brief
     * Fetches the gyroscope count from the IMU
     * @param destination The array where to store the gyroscope count
     */
    void readGyroData(int16_t * destination);
    
    /**
     * @brief
     * Fetches the magnetometer count from the IMU
     * @param destination The array where to store the magnetometer count
     */
    void readMagData(int16_t * destination);
    
    /**
     * @brief
     * Fetches the temperature count from the IMU
     * @return The temperature count
     */
    int16_t readTempData();
    
    /**
     * @brief
     * Calculates the value of the acceleration measured by the accelerometer in g
     * @param acc The array where to store the acceleration
     */
    void getAccel(float acc[3]);
    
    /**
     * @brief
     * Calculates the value of the rate of turn measured by the gyroscope in deg/s
     * @param gyr The array where to store the rate of turn
     */
    void getGyro(float gyr[3]);
    
    /**
     * @brief
     * Calculates the value of the magnetic field measured by the magnetometer
     * @param mag The array where to store the magnetic field in uT
     */
    void getMag(float mag[3]);
    
    /**
     * @brief
     * Calculates the temperature from the thermistance in the IMU
     * @return The temperature measured by the IMU in deg C
     */
    float getTemp();

///@name Filtering functions
    /**
     * @brief
     * An orientation filter to be used in AHRS applications
     * @details
     * Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
     * (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
     * which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
     * device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
     * The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
     * but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
     * @param quat The array that holds the quaternion
     * @param acc The acceleration in g
     * @param gyr The rate of turn in rad/s
     * @param mag The magnetic field in uT
     * @param dt The time ellapsed since last filter update
     */
    void MadgwickQuaternionUpdate(float quat[4], float acc[3], float gyr[3], float mag[3], float dt);

    /**
     * @brief
     * Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
     * measured ones.
     * @param quat The array that holds the quaternion
     * @param acc The acceleration in g
     * @param gyr The rate of turn in rad/s
     * @param mag The magnetic field in uT
     * @param dt The time ellapsed since last filter update
     */
    void MahonyQuaternionUpdate(float quat[4],float acc[3], float gyr[3], float mag[3], float dt);

private:
    I2C *i2c_;                  ///< I2C instance to use
    // Set initial input parameters
    uint8_t ascale;             ///< Accelerometer scale (AFS_2G, AFS_4G, AFS_8G, AFS_16G)
    uint8_t gscale;             ///< Gyroscope scale (GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS)
    float aRes_, gRes_, mRes_;  ///< Scale resolutions per LSB for the sensors
    float magCalibration[3];    ///< Factory mag calibration
    float magBias[3];           ///< Factory mag bias
    float gyroBias[3];          ///< Bias corrections for gyro
    float accelBias[3];         ///< Bias corrections for accelerometer
    float avg_acc[3], avg_gyr[3], avg_mag[3]; ///< DC bias corrections
    float SelfTest_[6];
}; // class MPU9150
#endif // MPU9150_H