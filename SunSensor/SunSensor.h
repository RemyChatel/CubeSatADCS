/**
 * @file   SunSensor.h
 * @defgroup SunSensorGr Sun Sensor Driver
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A class to handle analog Sun sensor (three photodiode)
 * 
 * @details
 * # Description
 * ## Overview
 * This library provide an interface to read the analog output of
 * a set of three photodiodes that make a Sun sensor
 * 
 * Sun Sensors are a very common sensor technology for spacecraft. 
 * The Sun being several orders of magnitude brighter than any other
 * celestial object, and its radius being small when looked at from
 * Earth orbit, it is a very useful sensor.
 * 
 * Using ephemerides, the position of the Sun relative to the
 * spacecraft can be translated into an absolute orientation in the
 * ECI, for instance.
 * 
 * However, it does not work during eclipse periods when the
 * spacecraft passes during its orbit behind the Earth.
 * 
 * Different technologies exist that provide a wide range of accuracy:
 * from the simple photodiodes and linear arrays to CCD array sensors.
 * 
 * @see SunSensor
 * 
 * ## Detailed description
 * The Sun sensor module simply reads the analogue value from the three
 * photodiodes that make the sensor and then combine them in a Sun vector
 * (measured in the body frame of the spacecraft). It relies on Mbed to
 * access the analogue pins.
 * 
 * The current Sun Sensor model is simple: the three photodiodes are placed
 * perpendicularly to each other. 
 * 
 * Each photodiode senses the light from the Sun with an amplitude related
 * to the angle of the Sun following the relation:
 * 
 * @f[
 *     V_i = V_0 \cos \alpha
 * @f]
 * 
 * If we reduce the number of photodiodes to two, the direction of the Sun is given by:
 * 
 * @f[
 * \mathbf{s}_{s}^{*}=
 * \left[\begin{array}{lll}
 *     1 & \tan \alpha_{1} / \tan \alpha_{2} & \tan \alpha_{1}
 * \end{array}\right]^{\top}
 * @f]
 * 
 * Then the vector is normalized:
 * 
 * @f[
 *     \mathbf{\hat{s}} = \frac{\mathbf{s}_{s}^{*}}{\mathbf{s}_{s}^{*\top}\mathbf{s}_{s}^{*}}
 * @f]
 * 
 * Now, two elements of the Sun direction are measured and the third element
 * can be found using a third photodiode, then it is possible to fully determine
 * the direction of the Sun.
 * 
 * In this project's configuration, the three photocells are placed orthogonally
 * and output a voltage, so the Sun direction vector can be expressed in the
 * following way:
 * 
 * @f[
 * \mathbf{\hat{s}}=
 *     \left[\begin{array}{ccc}
 *         \frac{V_i}{V_{max,i}} & \frac{V_j}{V_{max,j}} & \frac{V_k}{V_{max,k}}
 *     \end{array}\right]^\top
 * @f]
 * 
 * This system is the most simple configuration of Sun sensor, and the more
 * sophisticated system can be made, notably using a linear array of photodiode
 * under a slit. It was not done as the main focus of the project was the algorithms. 
 * # Example code
 * 
 * @see SunSensor.test.h
 * 
 * 
 * # References
 * - Spacecraft Dynamics and Control, by Chris Hall
 * - Photodiode Placement & Algorithms for CubeSat Attitude Determination, by
 * Springmann, John C and Cutler, James W
 * 
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

#ifndef SUNSENSOR_H
#define SUNSENSOR_H
#include "mbed.h"

/**
 * @ingroup SunSensorGr
 * @brief
 * A class to handle analog Sun sensor (three photodiode)
 * 
 * @class SunSensor
 * 
 * @details
 * # Description
 * This class handle the output of an analog Sun sensor that
 * uses three photodiodes places orthogonaly from each others
 * 
 * @see SunSensor.h
 * 
 * # Dependencies
 * This library was built around the "Mbed" framework to access the harware
 * through an common interface regardless of the device as long as the device
 * is supported by Mbed (https://www.mbed.com/en/)
 */
class SunSensor{
public:
    /* Public functions */
    /**
     * SunSensor Constructor
     * 
     * Default pins are A0, A1 and A2 for X, Y and Z
     */ 
    SunSensor();
    
    /**
     * SunSensor Constructor
     * @param pinX The pin of the X face sensor
     * @param pinY The pin of the Y face sensor 
     * @param pinZ The pin of the Z face sensor
     */
    SunSensor(PinName pinX, PinName pinY, PinName pinZ);
    
    /**
     * SunSensor Destructor
     */
    ~SunSensor(void);
    
    /**
     * Provides the sun vector in the body frame
     * @param rsun A table to hold the sun vector
     */
    void getSunVector(float rsun[3]);
    
    /**
     * Provides the X face light reading (normalised to 1.0)
     */
    float getXface();
    
    /**
     * Provides the Y face light reading (normalised to 1.0)
     */
    float getYface();
    
    /**
     * Provides the Z face light reading (normalised to 1.0)
     */
    float getZface();
    
private:
    /* Private variables */
    AnalogIn faceX_;
    AnalogIn faceY_;
    AnalogIn faceZ_;
    AnalogIn dummy_; // added to properly wake up the adc before using it
};
#endif