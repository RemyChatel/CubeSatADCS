/**
 * @file   SunSensor.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @defgroup SunSensorGr Sun Sensor Driver
 * 
 * @brief
 * A class to handle analog Sun sensor (three photodiode)
 * 
 * @details
 * # Description
 * This library provide an interface to read the analog output of
 * a set of three photodiodes that make a Sun sensor
 * 
 * @see SunSensor
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
 * @{
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
 * @}
 */

class SunSensor{
public:
    /* Public functions */
    
    SunSensor();
    
    /**
     * SunSensor Constructor
     * @param analogX The pin of the X face sensor
     * @param analogY The pin of the Y face sensor 
     * @param analogZ The pin of the Z face sensor
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
};
#endif