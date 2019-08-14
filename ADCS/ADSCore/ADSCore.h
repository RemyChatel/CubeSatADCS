/**
 * @file ADSCore.h
 * @defgroup ADSCoreGr ADSCore
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A library for Spacecraft Attitude Determination
 * 
 * @details
 * # Description
 * This library provide tools to perform the attitude determination of a spacecraft
 * based on the fusion of multiple sensors (IMU and Sun sensor) using the QuEst
 * algorithm and then filtering of the computed attitude using a Kalman Filter.
 * 
 * @see ADSCore.h
 * 
 * # Dependencies and data type
 * ![Dependency graph](_a_d_s_core_8h__incl.png)
 * This library depends on:
 * - the Mbed framework (https://www.mbed.com/en/)
 * - <std::cmath> in order to perform cos, sin and sqrt operations.
 * - <std::vector> for the matrix algebra
 * 
 * @attention This library uses float (32-bits) and not double (64-bits)
 * to make best use of the Floating Point Unit of 32-bits microcontrollers.
 * 
 * # Example code
 * @see ADSCore.test.cpp
 * 
 * # References
 * - "Kalman Filtering and the Attitude Determination and Control Task",
 * by Hale, Vergez, Meerman and Hashida
 * - "Spacecraft Dynamic and Control: An introduction",
 * by A. de Ruiter, C. Damaren and J Forbes,
 * - "Fundamentals of Astrodynamics and Applications", by D. Vallado,
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
 
#ifndef ADSCORE_H
#define ADSCORE_H

#define DEG2RAD 3.1415926535f/180.0f    ///< The conversion from degrees to radians
#define RAD2DEG 180.0f/3.1415926535f    ///< The conversion from radians to degrees

#define ADSCore_NSENSOR 2               ///< The number of sensor used for Quest algorithm
#define ADSCore_TOLERANCE 1e-5          ///< The tolerance of the Quest algorithm
#define ADSCore_USE_GND                 ///< Trigger the use of the Ground model instead of the orbital model
#define ADSCore_USE_PRINTF              ///< Enable the use of debug printf inside the object

#include "mbed.h"
#include "Matrix.h"
#include "AstroLib.h"
#include "Estimators.h"
#include "Filters.h"
#include "MPU9150.h"
#include "SunSensor.h"

/**
 * @ingroup ADSCoreGr
 * @brief
 * Accesses, processes and filters the data from sensors to output an attitude quaternion
 * of the Spacecraft
 * 
 * @class ADSCore
 * 
 * @details
 * # Description
 * In order to determine the attitude (the rotation between the Earth Centered Inertial
 * frame and the body frame), sensors are used to get a set of measurements in the body
 * frame while models are used to provide the same measurements in the ECI frame.
 * 
 * The QuEst algorithm is used to find the best rotation quaternion that links each pair
 * of body-ECI vectors.
 * 
 * Then a Kalman Filter is applied to the output quaterion to filter out any noise according
 * to the dynamic model of the satellite.
 * 
 * 
 * @see ADSCore.h
 * @nosubgrouping
 */
class ADSCore{
public:
///@name Constructors
    /**
     * @brief
     * Default constructor for the Attitude Determination core
     */
    ADSCore();

    /**
     * @brief
     * Constructor for the Attitude Determination core with a given I2C port and user-defined Analog pins
     * @param i2c The I2C object to pass on to the IMU
     * @param sunX The Analog pin for the X face photodiode
     * @param sunY The Analog pin for the Y face photodiode
     * @param sunZ The Analog pin for the Z face photodiode
     */
    ADSCore(I2C* i2c, PinName sunX, PinName sunY, PinName sunZ);

    /**
     * @brief
     * Constructor for the Attitude Determination core with given I2C pins and user-defined Analog pins
     * @param sda The Serial DAta pin of the I2C
     * @param scl The Serial CLock of the I2C
     * @param sunX The Analog pin for the X face photodiode
     * @param sunY The Analog pin for the Y face photodiode
     * @param sunZ The Analog pin for the Z face photodiode
     */
    ADSCore(PinName sda, PinName scl, PinName sunX, PinName sunY, PinName sunZ);

///@name Object initialization
    /**
     * @brief
     * Initialize the sensor (IMU and Sun sensors) by calibrating the sensors and removing DC biases
     */
    void initSensors();

    /**
     * @brief
     * Initialize the orbit model with the given date and orbital parameters
     * @param parameters The array with the orbital parameters (units: m and rad)
     * [axis, ecc, inc, right ascension, argument of perigee, true anomaly] or 
     * [latitude, longitude, altitude, mag_North, mag_East, mag_Down] for Ground version
     * @param date The starting date in the format [2019, 12, 31, 23, 59, 59]
     */
    void initOrbit(float parameters[6], int date[6]);

    /**
     * @brief
     * Sets the variances of the sensors for the Quest algorithm
     * @param sigma_mag The variance of the magnetometer
     * @param sigma_sun The variance of the Sun sensor
     */
    void initQuest(float sigma_mag, float sigma_sun);

    /**
     * @brief
     * Sets the Kalman filters variances, inertia matrices and initial position when there is no reaction wheels
     * @param sigma_q_eta The variance of the scalar part of the quaternion
     * @param sigma_q_espilon The variance of the vector part of the quaternion
     * @param sigma_gyr The variance of the gyroscope
     * @param dt The expected time integration step (s)
     * @param I_sat The inertia matrix (or tensor) of the satellite (kg.m2)
     * @param q_init The value of the atitude quaternion at start-up
     * @param w_init The value of the angular rates at start-up (rad/s)
     */
    void initKalman(float sigma_q_eta, float sigma_q_espilon, float sigma_gyr, float dt, Matrix I_sat, Matrix q_init, Matrix w_init);

    /**
     * @brief
     * Sets the Kalman filters variances, inertia matrices and initial position as well as the proprieties of the Control system
     * @param sigma_q_eta The variance of the scalar part of the quaternion
     * @param sigma_q_espilon The variance of the vector part of the quaternion
     * @param sigma_gyr The variance of the gyroscope
     * @param dt The expected time integration step (s)
     * @param I_sat The inertia matrix (or tensor) of the satellite (kg.m2)
     * @param q_init The value of the atitude quaternion at start-up
     * @param w_init The value of the angular rates at start-up (rad/s)
     * @param I_wheel_init The inertia matrix (or tensor) of the reactions wheels
     */
    void initKalman(float sigma_q_eta, float sigma_q_espilon, float sigma_gyr, float dt, Matrix I_sat, Matrix q_init, Matrix w_init, Matrix I_wheel_init);

///@name Access
    /**
     * @brief
     * Gets the predicted atitude quaternion
     * @return The predicted attitude quaternion
     */
    Matrix getQ() const;
    
    /**
     * @brief
     * Gets the predicted angular rates (rad/s)
     * @return The predicted angular rates
     */
    Matrix getW() const;
    
    /**
     * @brief
     * Gets the measured angular rates (deg/s)
     * @return The measured angular rates
     */
    Matrix getGyrb() const;
    
    /**
     * @brief
     * Gets the measurements in the body frame of the n-th sensor
     * @param n The index of the sensor to fetch
     * @return The measurements in the body frame of the n-th sensor
     */
    Matrix getSensorBody(int n);
    
    /**
     * @brief
     * Gets the measurements in the ECI frame of the n-th sensor
     * @param n The index of the sensor to fetch
     * @return The measurements in the ECI frame of the n-th sensor
     */
    Matrix getSensorECI(int n);
    
    /**
     * @brief
     * Gets the Kalman filter object reference for external access
     * @return The Kalman filter object reference
     */
    const Filters::KalmanFilter& getKalman() const;

    #ifdef ADSCore_USE_GND
    /**
     * @brief
     * Gets the Ground object reference for external access
     * @return The Ground object reference
     */
    const AstroLib::Ground& getOrbit() const;
    #else
    /**
     * @brief
     * Gets the Orbit object reference for external access
     * @return The Orbit object reference
     */
    AstroLib::Orbit& getOrbit() const;
    #endif

///@name Updaters
    /**
     * @brief
     * Updates the attitude of the spacecraft by combining the output the different
     * sensors and filtering the resulting attitude measurement.
     * @return The attitude quaternion
     */
    Matrix update();

    /**
     * @brief
     * Updates the attitude of the spacecraft by combining the output the different
     * sensors and filtering the resulting attitude measurement knowing the action
     * of the Control system
     * @param w_rw_prev The rotation speed of the reaction wheels (rad/s)
     * @param T_bf_prev The torque applied by the Control actuactors that are NOT the reaction wheels
     * @param T_rw_prev The torque applied by the reaction wheels
     * @return The attitude quaternion
     */
    Matrix update(Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev);

private:
    /**
     * @brief
     * Fetches the data from the sensors and the orbit model
     */
    void fetchSensors();

    Timer time;                     ///< Timer to keep track of loop time
    I2C* _i2c;                      ///< The I2C bus for the IMU

    MPU9150 imu;                    ///< The Inertial Measurement Unit
    SunSensor sun;                  ///< The analog Sun sensor

    #ifdef ADSCore_USE_GND
    AstroLib::Ground orbit;         ///< The "ground" version of Orbit for lab testing
    #else
    AstroLib::Orbit orbit;          ///< The orbit of the satellite
    #endif

    Filters::KalmanFilter kalman;

    Matrix q;                       ///< The output atitude quaternion
    Matrix w;                       ///< The output angular rates (rad/s)

    float vecf[3];                  ///< Temporary array storage for vectors    
    Matrix gyrb;                    ///< Gyrometer output   
    Matrix sbod[ADSCore_NSENSOR];   ///< The array of Matrices of the body frame measurements
    Matrix seci[ADSCore_NSENSOR];   ///< The array of Matrices of the ECi  frame measurements

    float last_update;              ///< Time since last update
    float omega[ADSCore_NSENSOR];   ///< Weight of the sensor for Quest
}; // End class ADSCore
#endif // ADSCORE_H